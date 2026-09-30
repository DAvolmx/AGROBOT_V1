#pragma once
// Stub de esp_system.h: solo lo que usa el sketch.
typedef enum { ESP_RST_UNKNOWN = 0, ESP_RST_POWERON, ESP_RST_BROWNOUT = 9 } esp_reset_reason_t;
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
