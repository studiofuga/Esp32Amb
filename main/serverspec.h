#ifndef SERVERSPEC_H
#define SERVERSPEC_H


#define SRV_APP_ID                     0x55
#define EXCAMPLE_DEVICE_NAME                      "AmbEsp32"

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

extern esp_ble_adv_params_t srv_adv_params;
extern esp_ble_adv_data_t srv_adv_config;
extern esp_ble_adv_data_t srv_scan_rsp_config;


#endif
