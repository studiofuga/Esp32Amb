//// must be included by gattserver.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"

#include "string.h"

#include "gattserver.h"
#include "serverspec.h"


static uint8_t sec_service_uuid[16] = {
        /* LSB <--------------------------------------------------------------------------------> MSB */
        //first uuid, 16bit, [12],[13] is the value
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};

static uint8_t test_manufacturer[3]={'E', 'S', 'P'};

esp_ble_adv_data_t srv_adv_config = {
        .set_scan_rsp = false,
        .include_txpower = true,
        .min_interval = 0x100,
        .max_interval = 0x100,
        .appearance = 0x00,
        .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
        .p_manufacturer_data =  NULL, //&test_manufacturer[0],
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(sec_service_uuid),
        .p_service_uuid = sec_service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
esp_ble_adv_data_t srv_scan_rsp_config = {
        .set_scan_rsp = true,
        .include_name = true,
        .manufacturer_len = sizeof(test_manufacturer),
        .p_manufacturer_data = test_manufacturer,
};

esp_ble_adv_params_t srv_adv_params = {
        .adv_int_min        = 0x100,
        .adv_int_max        = 0x100,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};



#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;


///Attributes 
/*
 *  PROFILE ATTRIBUTES variables
 ****************************************************************************************
 */

/// Battery Sensor Service
static const uint16_t battery_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint16_t battery_measure_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static uint8_t battery_measure_value = 0x00;

/// Temperature
static const uint16_t ambient_svc = 0xFF00;		// custom
static const uint16_t ambient_temp_uuid = 0xFF01;	// custom
static uint8_t ambient_temp_value[2] = { 0x00, 0x00 };


/// Full Database Description - Used to add attributes into the database
const esp_gatts_attr_db_t srv_gatt_db[SRV_ATTR_NB] =
{
    // Service Declaration
    [SRV_BATT_SVC]                    =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(battery_svc), (uint8_t *)&battery_svc}},

    // Battery Measurement Characteristic Declaration
    [SRV_BATT_MEAS_UUID]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify}},

    // Battery Measurement Characteristic Value
    [SRV_BATT_MEAS_VALUE]             =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&battery_measure_uuid, ESP_GATT_PERM_READ,
      1,0, NULL}},	// len of meas = 1

    // Battery Measurement Characteristic - Client Characteristic Configuration Descriptor
    [SRV_BATT_NTFY_CFG]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(battery_measure_value), (uint8_t *)&battery_measure_value}},
};

uint16_t svc_handle_table[SRV_ATTR_NB];

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

#endif

