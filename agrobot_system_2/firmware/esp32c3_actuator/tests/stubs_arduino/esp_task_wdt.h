#pragma once
// Stub de esp_task_wdt.h: solo lo que usa el sketch. Registra si se activó.
#include <cstdint>
typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#endif
typedef struct { uint32_t timeout_ms; uint32_t idle_core_mask; bool trigger_panic; } esp_task_wdt_config_t;
inline bool stub_wdt_activo = false;
inline uint32_t stub_wdt_ms = 0;
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *c) { stub_wdt_ms = c->timeout_ms; return ESP_OK; }
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t *c) { stub_wdt_ms = c->timeout_ms; return ESP_OK; }
inline esp_err_t esp_task_wdt_init(uint32_t s, bool) { stub_wdt_ms = s * 1000; return ESP_OK; }
inline void enableLoopWDT() { stub_wdt_activo = true; }
