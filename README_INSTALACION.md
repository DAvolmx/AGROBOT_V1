# 🤖 AGROBOT v2.0 — INSTALACIÓN

## ⚡ ÚNICO COMANDO (TODO se instala automáticamente)

```bash
sudo bash INSTALAR_AGROBOT.sh
```

**Eso es TODO.** El script hará:
- ✅ Instalar dependencias
- ✅ Copiar configuración a /etc
- ✅ Instalar servicios systemd
- ✅ Habilitar autostart
- ✅ Iniciar servicios
- ✅ Potencia WiFi al máximo
- ✅ Verificar todo

---

## 📡 Después de instalar:

**Conectar a WiFi:**
- SSID: `JETSON_AP`
- Contraseña: `12345678`

**Acceder a Dashboard:**
- URL: `http://192.168.12.1:8000`

---

## 🔍 Ver logs:

```bash
sudo journalctl -u agrobot-hostapd.service -f
```

---

## ✨ Listo.

