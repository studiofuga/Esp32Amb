#ifndef SERVERSPEC_H
#define SERVERSPEC_H

#include "esp_system.h"
#include "esp_bt.h"         // BT Controller and VHCI config
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"    // GAP services implementation
#include "esp_gatts_api.h"      // GATT services implementation

#define SRV_APP_ID            0x19     // Service Application Id on the stack
#define DEVICE_NAME           "AmbientMonitor"           // the device name as advertized
#define NUM_PROFILES 1              // Number of implemented profiles
#define PRF_IDX_BATTERY 0           // Our Battery Profile is #0

#define SVC_INST_ID                    0    // Not clear what it is

enum BATTERY_SVC_SPEC {
    SRV_IDX_BATTERY_SVC,            // Service index

    SRV_IDX_BATTERY_LEVEL_CHAR,     // Battery Level Characteristics
    SRV_IDX_BATTERY_LEVEL_VALUE,     // Battery Level Value
    SRV_IDX_BATTERY_LEVEL_NFYCFG,     // Battery Level Notification configuration

    SRV_IDX_BATTERY_LEVEL_PRESENTATION_CHAR,    // Battery Level presentation Characteristics
    SRV_IDX_BATTERY_LEVEL_PRESENTATION_VAL,    // Battery Level presentation Value

    SRV_ATTR_NB      // Last - Number of Characteristics
};

#define SRV_IDX_PRIMARY_SERVICE SRV_IDX_BATTERY_SVC

struct gatts_profile_inst {                     // This is local to the profile implementation
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

extern void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

extern void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

extern struct gatts_profile_inst profiles_tab[NUM_PROFILES];    // profile tab
extern esp_ble_adv_data_t srv_adv_config;                       // Advertising data

extern const esp_gatts_attr_db_t srv_gatt_db[];

#endif
