# Guion del video final de AgroBot

## Diapositiva 1. AgroBot

AgroBot es un robot agrícola autónomo, desarrollado como trabajo de titulación en la Universidad Técnica de Ambato.

Recorre cultivos de col y de arándano, mapea el terreno, busca plagas y enfermedades con visión artificial, y envía sus datos por LoRaWAN.

En este video resumo cómo está construido, cómo fusiona sus sensores, cómo son sus redes neuronales y qué mostraron las pruebas de campo.

## Diapositiva 2. Problema y objetivo

El problema es que las plagas y enfermedades se detectan tarde, porque la revisión se hace a mano, planta por planta.

El objetivo fue construir un robot que recorra el cultivo por sí solo, detecte plagas con una cámara y avise a distancia, usando hardware de bajo costo.

## Diapositiva 3. El robot

Este es AgroBot en el invernadero de Vitro Plantas. Tiene un chasis soldado de doce kilos y cuatro ruedas que giran como un tanque, con un eje pasivo que copia el terreno.

El cerebro es una Jetson Orin Nano, que corre ROS dos, el mapeo y las redes neuronales. Una placa ESP32 controla los motores y el servo de la cámara.

Arriba lleva un LIDAR de trescientos sesenta grados, una cámara con servo de inclinación, y un nodo LoRa con una unidad inercial.

## Diapositiva 4. Qué sensor se usa para cada cosa

Cada sensor tiene un papel. El LIDAR construye el mapa, mide cuánto avanza el robot y evita choques.

La cámara detecta plantas y plagas, sigue el sendero y también estima movimiento. La unidad inercial mide el giro.

Y el nodo LoRa lleva la telemetría y las alertas hasta una Raspberry Pi con ChirpStack.

## Diapositiva 5. Fusión de sensores

Así se fusionan. Un filtro de Kalman extendido, a veinte hercios, combina la velocidad de avance del LIDAR y de la cámara con la velocidad de giro de la unidad inercial.

Cuando el robot está quieto, un detector de reposo fija la velocidad en cero, para que la posición no se desplace sola.

Encima, el SLAM corrige la deriva con el mapa, y un vigía de pose descarta los saltos imposibles. Desde ayer, también se reubica solo después de un reinicio.

## Diapositiva 6. Autonomía

Para moverse, un explorador avanza por pulsos hacia los huecos que ve el LIDAR, y la cámara le indica dónde está el sendero.

Si algo se acerca a menos de veinte centímetros, la anticolisión retrocede o detiene el robot. Para ir a un punto del mapa usa Nav2.

Y cuando el operador corrige el rumbo con el mando, el robot guarda esa corrección y ajusta su comportamiento.

## Diapositiva 7. Simulador

Antes de salir al campo, todo se prueba en un simulador que ejecuta el mismo código del robot, con un LIDAR y una cámara simulados.

Aquí se ven sus cuatro vistas: el mundo en tres dimensiones, la cámara, el sendero y el mapa del SLAM, mientras una búsqueda evolutiva ajusta los parámetros.

## Diapositiva 8. Visión: qué modelos corren

En visión corren dos detectores a la vez: uno de contexto, que siempre está activo, y uno del cultivo elegido.

Para coles se usa PLAGASX, con once clases de enfermedades y plagas. Para arándano, ARANDANOS, que distingue fruto maduro e inmaduro y ocho plagas.

Cada modelo carga sus propios filtros, porque los que servían para coles descartaban todos los arándanos.

## Diapositiva 9. Arquitectura de la red: YOLO11s

Esta es la red YOLO once ese, la de ARANDANOS y la de contexto, dibujada desde sus capas reales. Tiene nueve coma cuatro millones de parámetros.

El backbone reduce la imagen cinco veces y extrae rasgos. El neck los mezcla de arriba abajo y de abajo arriba, y la cabeza detecta en tres escalas: objetos chicos, medianos y grandes.

Incluye un bloque de atención, C2PSA, que ayuda a encontrar objetos pequeños entre el follaje. PLAGASX usa la versión anterior, YOLO ocho ese.

## Diapositiva 10. Rendimiento en la Jetson

Medido en la Jetson, el modelo de contexto tarda sesenta y tres milisegundos, PLAGASX cuarenta y uno, y ARANDANOS ciento treinta y cinco, porque trabaja a novecientos sesenta píxeles.

Si se baja a seiscientos cuarenta va al doble de rápido, pero pierde once puntos de precisión. Por eso la optimización correcta es TensorRT, no achicar la imagen.

## Diapositiva 11. Comunicaciones

El robot crea su propia red Wi-Fi con una antena USB, y se ve también por VPN desde cualquier lugar.

La telemetría viaja por LoRaWAN en la banda AU915, la de uso libre en Ecuador, hasta un gateway y una Raspberry Pi con ChirpStack.

En la prueba de coles llegaron los treinta y cuatro paquetes, con una señal entre menos cincuenta y tres y menos ochenta y dos dBm.

## Diapositiva 12. Ensayos interiores

En interior, el SLAM recorrió veintitrés metros y volvió al punto de partida con un error de treinta y seis centímetros, cerca del uno coma cinco por ciento.

El detector de macetas acertó veinte de veinte cuadros sin falsos positivos, y en un recorrido en L identificó la especie diecisiete de diecisiete veces.

## Diapositiva 13. Prueba en la huerta de coles

En la huerta de coles el robot operó casi dieciséis minutos y recorrió unos cuarenta y un metros.

El SLAM generó un mapa de trece por nueve metros, con una vuelta completa alrededor de los canteros, y la localización se recuperó sola en veintiocho segundos tras un salto al arrancar.

El modelo de contexto reconoció las coles, y PLAGASX solo dio avisos débiles, por debajo del umbral de alerta.

## Diapositiva 14. Prueba en Vitro Plantas (arándano)

En el invernadero de Vitro Plantas el vivero no tenía fruto, y el modelo marcó plagas falsas sobre el suelo y las bandejas.

Un filtro que descarta plagas que ocupan más de un cuarto de la imagen las bajó de once a cero, sin perder ninguno de los mil doscientos frutos reales de prueba.

En la plantación adulta, en cambio, detectó bien los racimos de fruto verde.

## Diapositiva 15. Plagas en fotos de campo

Con fotos de campo encontramos mosca blanca en la col, pudrición y manchas café en acelga. El modelo actual las confundía con mildiu.

Probamos un filtro clásico de imagen para resaltar las colonias de mosca blanca. Sirve para verlas y para pre etiquetar, pero no como detector, porque también marca brillos en hojas mojadas.

## Diapositiva 16. Aumentación y reentrenamiento

Con esas fotos armamos un nuevo conjunto de datos, con doce variantes aumentadas de cada imagen, y reforzamos la aumentación del entrenamiento: rotación, escala, mezcla de imágenes y borrado aleatorio.

PLAGASX diez y ARANDANOS dos se están entrenando en Kaggle. Sus métricas se agregarán al informe final cuando terminen.

## Diapositiva 17. Conclusiones y trabajo futuro

En conclusión, AgroBot funciona como sistema completo en campo: mapea, se localiza, detecta, guarda datos y los envía por LoRaWAN.

El límite actual es la percepción, que necesita más fotos propias etiquetadas. Como trabajo futuro quedan TensorRT, unir los modelos, y medir el alcance LoRa a dos o tres kilómetros.

Gracias por su atención.

