# Reinstalar el firmware del gateway HT-M02 V2

**Fuente oficial:** https://resource.heltec.cn/download/HT-M02_V2/

| Archivo | Tamaño | Para qué |
|---|---|---|
| `HT-M02firmware.zip` | 1,8 GB (2023-08-14) | Paquete completo: RKDevTool v2.8, drivers, `update-pro-rk3566-debian-20220515-230650.img` y `HT-M02-boot-20230517.img` |
| `HT-M02 Burning Guide.pdf` | copia aquí | Paso a paso (en inglés) |
| `HT-M02_flash_firmware.pdf` | copia aquí | Otra versión de la guía |

## Qué hace falta
- Un PC con **Windows** (la guía usa RKDevTool y los drivers Rockchip).
- Un **cable USB-C de datos**. El puerto USB-C está **bajo la tapa lateral**: no es de alimentación.
- Destornillador, cable Ethernet y la fuente DC del gateway.

## Pasos (resumen de la guía)
1. En Windows, instalar `DriverAssitant_v5.1.1/DriverInstall.exe`.
2. Abrir la tapa lateral y conectar el USB-C al PC.
3. Abrir `RKDevTool_Release_v2.8/RKDevTool.exe`.
4. **Mantener apretado REC**, conectar la corriente y soltar REC cuando aparezca «Found a LOADER device».
5. En «Upgrade Firmware» → «Firmware», elegir `update-pro-rk3566-debian-20220515-230650.img`.
6. Hacer clic en «Erase Flash» y confirmar. Después, en «Upgrade», esperar al 100 %.
7. Esperar a que reinicie y dejarlo enfriar 2–3 min. Desconectar la corriente y repetir REC + corriente hasta ver «LOADER».
8. En «Download Image» marcar **solo** la fila 7 «Boot». Hacer clic en «Device Partition Table» hasta que la dirección sea `0x00008000`.
9. Elegir `HT-M02-boot-20230517.img` en la fila Boot y hacer clic en «Execute» hasta llegar al 100 %.
10. Conectarlo al router por cable y buscar su IP. Entrar por SSH como `root` con la clave inicial que indica la guía.
11. Inicializar según el hardware (el nuestro tiene Wi-Fi):
    `rm -rf init_M02* && wget http://minerback.heltec.cn/download/init_M02_extraServer.sh && chmod +x init_M02_extraServer.sh && ./init_M02_extraServer.sh WIFI`
12. Reinicia con la red Wi-Fi propia `HT-M02-AP` (clave de fábrica, ver la guía de Heltec) y SSH `root` con la clave de fábrica de la guía. Actualizar la página de configuración:
    `rm -rf update_M02* && wget http://minerback.heltec.cn/download/update_M02.sh && chmod +x update_M02.sh && ./update_M02.sh`

## Después: dejarlo como estaba en AgroBot
- Modo **WIFI Client + ETH LAN**, Wi-Fi `LoRa_Wan`, IP estática `192.168.0.11` (máscara /24, puerta de enlace `192.168.0.1`).
- Región **AU915**, sub-banda canales 0–7 + 64 (`au915_0`).
- Servidor de paquetes `192.168.0.20:1700` (la Pi por Wi-Fi; `192.168.0.10` si la Pi va por cable).
- Comprobar en ChirpStack que el gateway `065f31fffe49b5a6` vuelve a «visto» y que llegan uplinks del nodo `f59d08345985b27d`.
