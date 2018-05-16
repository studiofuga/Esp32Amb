#ifndef GATTSERVER_H
#define GATTSERVER_H

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

void gattserver_init();
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);
void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


#endif
