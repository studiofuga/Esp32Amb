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

