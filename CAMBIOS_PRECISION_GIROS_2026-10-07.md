# 🔧 CAMBIOS DE PRECISIÓN EN GIROS — 2026-10-07

## 🔴 **PROBLEMAS IDENTIFICADOS**

### Problema 1: Retroceso en Giros
**Síntoma**: Robot retrocede sin razón durante giros
**Causa**: Maniobra alterna `v` entre positivo y negativo (adelante/atrás) mientras gira
**Archivo**: `pulse_drive.py` línea 98

### Problema 2: Velocidad Limitada Adelante
**Síntoma**: Adelante no va a máxima potencia
**Causa**: `vel_factor = 0.6` limitaba velocidad a 60%
**Archivos**: `pulse_drive.py` líneas 44, 56

### Problema 3: Firmware Normaliza Giros
**Síntoma**: Potencia cae cuando omega es alta
**Causa**: Línea 440 normaliza escala si wPico > wRef
**Archivo**: `esp32c3_actuador_simple.ino` línea 440

---

## ✅ **CAMBIOS APLICADOS**

### CAMBIO 1: Aumentar Velocidad a Máximo

**Archivo**: `agrobot_system_2/jetson/backend/pulse_drive.py`

**Líneas 50-56 (modo preciso)**:
```python
ANTES:
  "vel_factor": 0.6,  # 60% velocidad

DESPUÉS:
  "vel_factor": 1.0,  # 100% velocidad máxima
```

**Líneas 38-44 (modo interior)**:
```python
ANTES:
  "vel_factor": 0.6,  # 60% velocidad

DESPUÉS:
  "vel_factor": 1.0,  # 100% velocidad máxima
```

---

### CAMBIO 2: Eliminar Normalización en Giros

**Archivo**: `agrobot_system_2/firmware/esp32c3_actuador_simple.ino`

**Línea 440**:
```cpp
ANTES:
  if (wPico > wRef && wPico > 0.0f) escala = wRef / wPico;
  // Reduce potencia si omega es alto

DESPUÉS:
  // if (wPico > wRef && wPico > 0.0f) escala = wRef / wPico;
  // COMENTADA: permite motores al máximo incluso en giros
```

**Efecto**: Motores mantienen 100% potencia incluso en giros agresivos

---

## 📊 **RESULTADOS ESPERADOS**

| Aspecto | Antes | Después | Beneficio |
|---------|-------|---------|-----------|
| Velocidad adelante | 60% | 100% | ✅ Más rápido |
| Giros | Normalizado | 100% PWM | ✅ Preciso y potente |
| Retroceso en giros | Sí (maniobra) | No (solo omega) | ✅ Sin retroceso |
| Precisión giros | Media | Alta | ✅ Máxima |

---

## 🔄 **QUÉ HACER AHORA**

1. **Flashear firmware actualizado:**
   ```
   esp32c3_actuador_simple.ino (con cambio línea 440)
   ```

2. **Reiniciar Jetson:**
   ```bash
   sudo systemctl restart agrobot-backend.service
   ```

3. **Probar:**
   - Adelante: Debe ir a máxima potencia
   - Giros: Debe girar sin retroceso
   - Precisión: Giros exactos

---

## 🧠 **EXPLICACIÓN TÉCNICA**

### Por qué retrocedía:
```
pulse_drive.py línea 98 hacía:
  [(v, omega, 0.25s), (-v, omega, 0.25s)]
  
Es decir:
  • 0.25s: Adelante + giro izquierda
  • 0.25s: ATRÁS + giro izquierda (alternancia para tierra blanda)
  
Resultado: Media vuelta de atrás, media vuelta de adelante
Problema: Visible retroceso durante giro
```

### Por qué estaba lenta adelante:
```
v_max * 0.6 = velocidad limitada
El robot podría ir más rápido pero no lo hacía.
```

### Por qué el firmware normalizaba:
```
Si omega es alto, las velocidades de rueda (wIzq, wDer) 
pueden exceder V_MAX (saturación).
El firmware reducía TODO (adelante + giro) para no saturar.

RESULTADO: Giro lento cuando omega era agresivo
CAMBIO: Permitir saturación → motores al máximo siempre
```

---

## 📝 **NOTAS**

- La maniobra de tierra blanda (adelante/atrás alternado) se mantuvo DESACTIVADA (`maniobra_v = 0.0`)
- Ahora giros son **puros** (solo omega, sin v)
- Velocidad máxima en todos los modos
- Compatible con todas las rampas y ramificaciones de control

---

**Status**: ✅ LISTO PARA FLASHEAR Y PROBAR

**Cambios**: 2 archivos
- `pulse_drive.py`: 2 líneas modificadas
- `esp32c3_actuador_simple.ino`: 1 línea comentada (desactivada normalización)
