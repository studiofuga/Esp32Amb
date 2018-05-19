#include "serverspec.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"         // BT Controller and VHCI config
#include "esp_gap_ble_api.h"    // GAP services implementation
#include "esp_gatts_api.h"      // GATT services implementation
#include "esp_bt_defs.h"
#include "esp_bt_main.h"        // BlueDroid initialization

#include <string.h>

#define GATTS_TABLE_TAG "SEC_GATTS"


void gatts_battery_profile_event_handler(esp_gatts_cb_event_t event,
                                         esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


struct gatts_profile_inst profiles_tab[NUM_PROFILES] = {
        [PRF_IDX_BATTERY] = {
                .gatts_cb = gatts_battery_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        }
};

uint16_t svc_handle_table[SRV_ATTR_NB];

static uint8_t service_uuid[16] = {
        /* LSB <--------------------------------------------------------------------------------> MSB */
        //first uuid, 16bit, [12],[13] is the value
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};

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
        .service_uuid_len = sizeof(service_uuid),
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
esp_ble_adv_data_t srv_scan_rsp_config = {
        .set_scan_rsp = true,
        .include_name = true,
        .manufacturer_len = 0, //sizeof(test_manufacturer),
        .p_manufacturer_data = NULL, // test_manufacturer,
};

// Service Adv Parameters
esp_ble_adv_params_t srv_adv_params = {
        .adv_int_min        = 0x100,
        .adv_int_max        = 0x100,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// ----- Profile data -------

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;


// Variables

/// Battery Sensor Service
static const uint16_t battery_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint16_t battery_measure_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static uint8_t battery_measure_value = 0x00;
static const uint8_t battery_measurement_ccc[2]      = {0x00, 0x00};

/// Temperature
static const uint16_t ambient_svc = 0xFF00;        // custom
static const uint16_t ambient_temp_uuid = 0xFF01;    // custom
static uint8_t ambient_temp_value[2] = {0x00, 0x00};


/// Full Database Description - Used to add attributes into the database
/// The esp_gatts_attr_db_t structure has two members:
/// The attr_control is the auto-respond parameter which can be set as ESP_GATT_AUTO_RSP to allow the BLE stack to take
/// care of responding messages when read or write events arrive. The other option is ESP_GATT_RSP_BY_APP which allows
/// to manually respond to messages using the esp_ble_gatts_send_response() function.
/// The att_desc is the attribute description.
/// uint16_t uuid_length;      /*!< UUID length */
/// uint8_t  *uuid_p;          /*!< UUID value */
/// uint16_t perm;             /*!< Attribute permission */
/// uint16_t max_length;       /*!< Maximum length of the element*/
/// uint16_t length;           /*!< Current length of the element*/
/// uint8_t  *value;           /*!< Element value array*/
const esp_gatts_attr_db_t srv_gatt_db[SRV_ATTR_NB] =
        {
                // Service Declaration
                [SRV_IDX_BATTERY_SVC]                    =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & primary_service_uuid, ESP_GATT_PERM_READ,
                                 sizeof(uint16_t), sizeof(battery_svc), (uint8_t * ) & battery_svc}},

                // Battery Measurement Characteristic Declaration
                [SRV_IDX_BATTERY_LEVEL_CHAR]            =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & character_declaration_uuid, ESP_GATT_PERM_READ,
                                 CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t * ) & char_prop_read_notify}},

                // Battery Measurement Characteristic Value
                [SRV_IDX_BATTERY_LEVEL_VALUE]             =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & battery_measure_uuid, ESP_GATT_PERM_READ,
                                 sizeof(battery_measure_value), sizeof(battery_measure_value),
                                 (uint8_t * ) & battery_measure_value}},

                // Battery Measurement Characteristic - Client Characteristic Configuration Descriptor
                [SRV_IDX_BATTERY_LEVEL_NFYCFG]        =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & character_client_config_uuid,
                                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                 sizeof(uint16_t), sizeof(battery_measurement_ccc),
                                 (uint8_t * ) & battery_measurement_ccc}},

#if 0
                // Battery Measurement Characteristic Declaration
                [SRV_IDX_BATTERY_LEVEL_PRESENTATION_CHAR]            =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & character_declaration_uuid, ESP_GATT_PERM_READ,
                                 CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t * ) & char_prop_notify}},

                // Battery Measurement Characteristic Value
                [SRV_IDX_BATTERY_LEVEL_PRESENTATION_VAL]             =
                        {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t * ) & battery_measure_uuid, ESP_GATT_PERM_READ,
                                 1, 0, NULL}},    // len of meas = 1
#endif
        };


//------- Functions implementations ------------

static char *esp_key_type_to_str(esp_ble_key_type_t key_type);

static void show_bonded_devices(void);

static void remove_all_bonded_devices(void);

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

void gatts_battery_profile_event_handler(esp_gatts_cb_event_t event,
                                         esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            //generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(srv_gatt_db, gatts_if,
                                          SRV_ATTR_NB, SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
            /* start security connect with peer device when receive the connect event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
            /* start advertising again when missing the connect */
            esp_ble_gap_start_advertising(&srv_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
            if (param->create.status == ESP_GATT_OK) {
                if (param->add_attr_tab.num_handle == SRV_ATTR_NB) {
                    memcpy(svc_handle_table, param->add_attr_tab.handles,
                           SRV_ATTR_NB * sizeof(svc_handle_table[0]));
                    esp_ble_gatts_start_service(
                            svc_handle_table[SRV_IDX_PRIMARY_SERVICE]);     // <- This is the primary Svc
                } else {
                    ESP_LOGE(GATTS_TABLE_TAG,
                             "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                             param->add_attr_tab.num_handle, SRV_ATTR_NB);
                }
            } else {
                ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
            }
            break;
        }

        default:
            break;
    }

}

// -------- Standard Event Handlers ------

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&srv_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&srv_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
                break;
            }
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
            break;
        case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
            //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
            break;
        case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
            break;
        case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
            break;
        case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
            break;
        case ESP_GAP_BLE_NC_REQ_EVT:
            /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
            show the passkey number to the user to confirm it with the number displayed by peer deivce. */
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d",
                     param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            /* send the positive(true) security response to the peer device to accept the security request.
            If not accept the security request, should sent the security response with negative(false) accept value*/
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
            ///show the passkey number to the user to input it in the peer deivce.
            ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%d", param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_KEY_EVT:
            //shows the ble key info share with peer device to the user.
            ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT: {
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x", \
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                     (bd_addr[4] << 8) + bd_addr[5]);
            ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
            ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
            show_bonded_devices();
            break;
        }
        case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
            ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d",
                     param->remove_bond_dev_cmpl.status);
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
            ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
            esp_log_buffer_hex(GATTS_TABLE_TAG, (void *) param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
            ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
            break;
        }
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
            if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x",
                         param->local_privacy_cmpl.status);
                break;
            }

            esp_err_t ret = esp_ble_gap_config_adv_data(&srv_adv_config);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= ADV_CONFIG_FLAG;
            }

            ret = esp_ble_gap_config_adv_data(&srv_scan_rsp_config);
            if (ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            }

            break;
        default:
            break;
    }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                         esp_ble_gatts_cb_param_t *param)
{
    int idx;

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            for (idx = 0; idx < NUM_PROFILES; ++idx)
                profiles_tab[idx].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    for (idx = 0; idx < NUM_PROFILES; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE ||
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            gatts_if == profiles_tab[idx].gatts_if) {
            if (profiles_tab[idx].gatts_cb) {
                profiles_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch (key_type) {
        case ESP_LE_KEY_NONE:
            key_str = "ESP_LE_KEY_NONE";
            break;
        case ESP_LE_KEY_PENC:
            key_str = "ESP_LE_KEY_PENC";
            break;
        case ESP_LE_KEY_PID:
            key_str = "ESP_LE_KEY_PID";
            break;
        case ESP_LE_KEY_PCSRK:
            key_str = "ESP_LE_KEY_PCSRK";
            break;
        case ESP_LE_KEY_PLK:
            key_str = "ESP_LE_KEY_PLK";
            break;
        case ESP_LE_KEY_LLK:
            key_str = "ESP_LE_KEY_LLK";
            break;
        case ESP_LE_KEY_LENC:
            key_str = "ESP_LE_KEY_LENC";
            break;
        case ESP_LE_KEY_LID:
            key_str = "ESP_LE_KEY_LID";
            break;
        case ESP_LE_KEY_LCSRK:
            key_str = "ESP_LE_KEY_LCSRK";
            break;
        default:
            key_str = "INVALID BLE KEY TYPE";
            break;

    }

    return key_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);

    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *) dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

