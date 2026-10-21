"""Genera agrobot_c3_suave_unico/agrobot_c3_suave_unico.ino: el mismo firmware en UN solo archivo
(suave_model.h incrustado). Fuente de verdad: agrobot_c3_suave/ (con sus tests contra Python)."""
import pathlib
raiz = pathlib.Path(__file__).resolve().parent
ino = (raiz / "agrobot_c3_suave" / "agrobot_c3_suave.ino").read_text()
hdr = (raiz / "agrobot_c3_suave" / "suave_model.h").read_text().replace("#pragma once\n", "")
marca = '#include "suave_model.h"\n'
assert marca in ino
salida = ino.replace(marca, "\n// ===== MEZCLA Y RAMPAS (antes suave_model.h, incrustado para un solo archivo) =====\n" + hdr + "\n")
d = raiz / "agrobot_c3_suave_unico"
d.mkdir(exist_ok=True)
(d / "agrobot_c3_suave_unico.ino").write_text(salida)
print(len(salida.splitlines()), "líneas")

# ---- FUSION: mismo trato (fusion_model.h incrustado) ----
ino = (raiz / "agrobot_c3_fusion" / "agrobot_c3_fusion.ino").read_text()
hdr = (raiz / "agrobot_c3_fusion" / "fusion_model.h").read_text().replace("#pragma once\n", "")
marca = '#include "fusion_model.h"\n'
assert marca in ino
salida = ino.replace(marca, "\n// ===== MEZCLA, RAMPAS Y RAFAGAS (antes fusion_model.h, incrustado para un solo archivo) =====\n" + hdr + "\n")
d = raiz / "agrobot_c3_fusion_unico"
d.mkdir(exist_ok=True)
(d / "agrobot_c3_fusion_unico.ino").write_text(salida)
print("fusion:", len(salida.splitlines()), "líneas")
