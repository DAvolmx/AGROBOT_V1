"""
Cámara simulada en la GPU de la Jetson (PyTorch + CUDA).

Hace exactamente lo mismo que `Mundo.camara()` en CPU, paso por paso, pero con
tensores: el terreno pintado (`Mundo.raster_suelo`) se sube UNA vez a la GPU y
cada fotograma solo proyecta, mezcla y degrada. En la Orin Nano pasa de 50 ms
(640x480 en CPU) a unos pocos ms, lo que permite renderizar a la resolución
REAL de la NexiGo (640x480) y dejar que el detector la reduzca a 320x240 como
en el robot, en vez de fabricar directamente una imagen pequeña.

La PyTorch instalada (cu130) avisa de que no trae kernels para la Orin
(CC 8.7), pero funciona: se comprobó matmul, conv y grid_sample el
2026-09-30. Si algo falla en la GPU, `Mundo.camara` vuelve al render en CPU
para siempre y lo dice una vez en el log.

Desactivar con AGROBOT_SIM_GPU=0.
"""
from __future__ import annotations

import math
from collections import OrderedDict

import numpy as np
import torch
import torch.nn.functional as F

import mundo as M

# Mundos cuyo terreno se guarda en la GPU a la vez (~64 MB cada uno).
MAX_TEXTURAS = 3
MAX_GEOMETRIAS = 16
# Pesos de luminancia en BGR (los de cv2.cvtColor BGR2GRAY).
_GRIS_BGR = (0.114, 0.587, 0.299)


class RenderGPU:
    def __init__(self, dispositivo: str = "cuda"):
        self.dev = torch.device(dispositivo)
        self._texturas: OrderedDict = OrderedDict()
        self._geos: OrderedDict = OrderedDict()
        self._vinetas: dict = {}
        self._gen = torch.Generator(device=self.dev)
        self._gris = torch.tensor(_GRIS_BGR, device=self.dev).view(1, 3, 1, 1)
        self._bruma = torch.tensor(M.COLOR_BRUMA, dtype=torch.float32, device=self.dev).view(1, 3, 1, 1)

    # ------------------------------------------------------------- cachés
    def _textura(self, mundo):
        niveles, (x0, y1) = mundo.raster_suelo()
        clave = mundo._cache_raster[0]
        ent = self._texturas.get(id(mundo))
        if ent is None or ent["clave"] != clave:
            # uint8 HxWx4 (BGR + sombra) -> 1x4xHxW en half: la mitad de memoria
            # y grid_sample en half va bien en la Orin.
            tens = [torch.from_numpy(np.ascontiguousarray(tex)).to(self.dev)
                    .permute(2, 0, 1).unsqueeze(0).half() for tex, _ in niveles]
            ent = {"clave": clave, "niveles": tens, "res": [r for _, r in niveles],
                   "x0": float(x0), "y1": float(y1)}
            self._texturas[id(mundo)] = ent
            while len(self._texturas) > MAX_TEXTURAS:
                self._texturas.popitem(last=False)
        self._texturas.move_to_end(id(mundo))
        return ent

    def _geometria(self, g: dict):
        """Sube a la GPU la geometría de cámara que ya calculó la CPU
        (`Mundo._geometria_camara`), una vez por geometría."""
        clave = id(g)
        ent = self._geos.get(clave)
        if ent is not None and ent["g"] is g:
            self._geos.move_to_end(clave)
            return ent
        t = lambda a: torch.from_numpy(np.ascontiguousarray(a, dtype=np.float32)).to(self.dev)
        pesos = g.get("pesos_lod")
        ent = {"g": g, "xr": t(g["xr"]), "yr": t(g["yr"]), "dist": t(g["dist"]),
               "inv_dist": t(g["inv_dist"]), "dist_seg": t(g["dist_seg"]),
               "niebla": t(g["niebla"][..., 0]), "no_suelo": torch.from_numpy(g["no_suelo"]).to(self.dev),
               "cielo": t(g["cielo"]).permute(2, 0, 1).unsqueeze(0),
               "pared_fila": t(g["pared_fila"]).permute(1, 0).unsqueeze(0).unsqueeze(-1),
               "pesos": None if pesos is None else pesos}
        self._geos[clave] = ent
        while len(self._geos) > MAX_GEOMETRIAS:
            self._geos.popitem(last=False)
        return ent

    def _pesos_lod(self, mundo, g, ge, res0, n_niveles):
        """Los mismos pesos de nivel de detalle que `_suelo_proyectado`."""
        if ge.get("lod_res") != res0:
            huella = g["dist"] ** 1.5 / (g["fx"] * math.sqrt(max(g["h"], 0.05)))
            lod = np.clip(np.log2(np.maximum(huella / res0, 1.0)), 0, n_niveles - 1)
            w_a = np.clip(1.0 - lod, 0, 1).astype(np.float32)
            w_b = np.clip(2.0 - lod, 0, 1).astype(np.float32)
            ge["w_a"] = torch.from_numpy(w_a).to(self.dev)
            ge["w_b"] = torch.from_numpy(w_b).to(self.dev)
            ge["usa_1"], ge["usa_2"] = bool((w_a < 1).any()), bool((w_b < 1).any())
            ge["lod_res"] = res0
        return ge["w_a"], ge["w_b"], ge["usa_1"], ge["usa_2"]

    def _vineta(self, alto, ancho, fuerza):
        clave = (alto, ancho, round(fuerza, 4))
        v = self._vinetas.get(clave)
        if v is None:
            yy, xx = torch.meshgrid(torch.arange(alto, device=self.dev, dtype=torch.float32),
                                    torch.arange(ancho, device=self.dev, dtype=torch.float32),
                                    indexing="ij")
            r = torch.hypot((xx - ancho / 2) / (ancho / 2), (yy - alto / 2) / (alto / 2))
            v = (1 - fuerza * r.clamp(0, 1.4) ** 2).clamp(0, 1).view(1, 1, alto, ancho)
            if len(self._vinetas) > 8:
                self._vinetas.clear()
            self._vinetas[clave] = v
        return v

    # -------------------------------------------------------------- render
    @torch.no_grad()
    def render(self, mundo, x, y, theta, geo, ancho, alto, rng, velocidad, realismo,
               cabeceo_deg, omega):
        g = mundo._geometria_camara(geo, ancho, alto, cabeceo_deg)
        ge = self._geometria(g)
        tex = self._textura(mundo)
        c, s = math.cos(theta), math.sin(theta)
        px = x + ge["xr"] * c - ge["yr"] * s
        py = y + ge["xr"] * s + ge["yr"] * c

        # --- suelo: muestreo con niveles de detalle (como _suelo_proyectado)
        w_a, w_b, usa_1, usa_2 = self._pesos_lod(mundo, g, ge, tex["res"][0], len(tex["niveles"]))

        def muestra(k):
            t, res = tex["niveles"][k], tex["res"][k]
            h_t, w_t = t.shape[-2:]
            mx = (px - tex["x0"]) / res - 0.5
            my = (tex["y1"] - py) / res - 0.5
            rejilla = torch.stack(((2 * mx + 1) / w_t - 1, (2 * my + 1) / h_t - 1), dim=-1)
            return F.grid_sample(t, rejilla.unsqueeze(0).half(), mode="bilinear",
                                 padding_mode="reflection", align_corners=False).float()

        mezcla = muestra(0)
        if usa_1:
            mezcla = mezcla * w_a + muestra(1) * (1 - w_a)
            if usa_2:
                mezcla = mezcla * w_b + muestra(2) * (1 - w_b)
        img = mezcla[:, :3]
        if realismo > 0:
            img = img * (1 - mezcla[:, 3:] * (mundo.sombra_oscuridad * realismo / 255.0))

        # --- niebla de distancia
        niebla = ge["niebla"].view(1, 1, alto, ancho)
        img = img * (1 - niebla) + self._bruma * niebla

        # --- brillo del sol sobre el suelo húmedo
        if realismo > 0 and mundo.seco < 0.8:
            rel = math.radians(mundo.sol_acimut) - theta
            altura_sol = math.sin(math.radians(mundo.sol_altura)) * g["h"] / ge["dist_seg"]
            d = ((ge["xr"] * math.cos(rel) + ge["yr"] * math.sin(rel)) * ge["inv_dist"]
                 + altura_sol).clamp(0, 1)
            img = img * (1 + (d ** 24 * ((1 - mundo.seco) * 0.18))).view(1, 1, alto, ancho)

        # --- paredes (la distancia por columna la da el LIDAR de la CPU)
        if len(mundo.paredes):
            dist_pared = torch.from_numpy(
                mundo.rayos(x, y, theta, g["angulos"], alcance=30.0, solo_paredes=True)
                .astype(np.float32)).to(self.dev)
            tapado = ge["dist"] > dist_pared.view(1, ancho)
            nf = (1.0 - torch.exp(-0.04 * (dist_pared - 2).clamp(min=0))).clamp(0, 0.5).view(1, 1, 1, ancho)
            pared = ge["pared_fila"] * (1 - nf) + self._bruma * nf
            img = torch.where(tapado.view(1, 1, alto, ancho), pared.expand_as(img), img)

        # --- cielo
        img = torch.where(ge["no_suelo"].view(1, 1, alto, ancho), ge["cielo"], img).clamp(0, 255)

        if realismo > 0:
            img = self._degradar(mundo, img, rng, velocidad, realismo, omega, g["fx"], alto, ancho)

        out = img.round().clamp(0, 255).to(torch.uint8)[0].permute(1, 2, 0).contiguous().cpu().numpy()
        calidad = int(mundo.camara_params.get("jpeg", 0)) if realismo > 0 else 0
        if calidad > 0:
            import cv2
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, calidad])
            if ok:
                out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return out

    def _degradar(self, mundo, img, rng, velocidad, realismo, omega, fx, alto, ancho):
        """Lo mismo que `Mundo._degradar` (salvo el JPEG, que va en CPU)."""
        cam = mundo.camara_params
        # 1. Movida: vertical al avanzar, horizontal al girar.
        px_avance = M.DESENFOQUE_POR_VELOCIDAD * abs(velocidad) * realismo * (ancho / 320)
        if px_avance > 1.2:
            k = int(min(21, px_avance)) | 1
            img = F.avg_pool2d(F.pad(img, (0, 0, k // 2, k // 2), mode="reflect"), (k, 1), stride=1)
        px_giro = abs(omega) * M.EXPOSICION_CAMARA_S * fx * realismo
        if px_giro > 1.2:
            k = int(min(31, px_giro)) | 1
            img = F.avg_pool2d(F.pad(img, (k // 2, k // 2, 0, 0), mode="reflect"), (1, k), stride=1)
        # 2. Exposición y balance de blancos.
        ganancia = float(cam.get("ganancia", 1.0))
        balance = cam.get("balance", (1.0, 1.0, 1.0))
        if ganancia != 1.0 or tuple(balance) != (1.0, 1.0, 1.0):
            img = img * torch.tensor([ganancia * b for b in balance], device=self.dev).view(1, 3, 1, 1)
        img = img.clamp(0, 255)
        # 3. Viñeteo.
        vineta = M.VIÑETEO * float(cam.get("vineta", 1.0)) * realismo
        if vineta > 0:
            img = img * self._vineta(alto, ancho, vineta)
        # 4. Ruido, más en lo oscuro, con grano a media resolución.
        if rng is not None:
            self._gen.manual_seed(int(rng.integers(0, 2 ** 62)))
            grano = torch.randn((1, 3, alto // 2, ancho // 2), generator=self._gen, device=self.dev)
            grano = F.interpolate(grano, size=(alto, ancho), mode="bilinear", align_corners=False)
            luz = (img * self._gris).sum(dim=1, keepdim=True)
            sigma = ((M.RUIDO_EN_SOMBRA - (M.RUIDO_EN_SOMBRA - M.RUIDO_BASE) / 255.0 * luz)
                     * realismo * float(cam.get("ruido", 1.0)))
            img = img + grano * sigma
        return img
