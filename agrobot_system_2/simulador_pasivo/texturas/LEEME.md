# Texturas opcionales

El visor 3D **no necesita nada de aquí**: el terreno se texturiza con la
imagen que genera el propio simulador (`/textura.jpg`) más un mapa de relieve
sacado de ella (`/relieve.jpg`). Eso tiene una ventaja sobre un paquete
descargado: el suelo que se ve en 3D es exactamente el mismo que ve la cámara
del robot y el que aparece en el mapa 2D.

Si aun así querés usar texturas PBR (por ejemplo de polyhaven.com, que son
CC0), dejá aquí los archivos con estos nombres y el visor los usará como
detalle encima del terreno:

    tierra_diffuse.jpg   tierra_normal.jpg
    paja_diffuse.jpg     paja_normal.jpg

Van en `.gitignore` a propósito: son varios MB y el repo ya pesa 90 MB por
los modelos de IA.
