Test de la lógica de motores del firmware, sin hardware.

Copia las constantes y las funciones driveMotor()/applyMotors() del .ino y
captura lo que se escribiría en cada pin, para verificar el reparto
skid-steer, los sentidos y el presupuesto de canales PWM del ESP32-C3.

    g++ -std=c++17 -o motor_logic_test motor_logic_test.cpp && ./motor_logic_test

OJO: las funciones están COPIADAS del .ino, no incluidas desde él (un .ino
no se puede compilar solo). Si cambiás driveMotor() o applyMotors() en el
firmware, actualizá también este archivo o el test dejará de reflejar la
realidad.
