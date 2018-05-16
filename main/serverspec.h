#ifndef SERVERSPEC_H
#define SERVERSPEC_H


#define SRV_APP_ID                     0x55
#define EXCAMPLE_DEVICE_NAME                      "AmbEsp32"

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

#define SVC_INST_ID                    0

#define PROFILE_APP_IDX                     0
#define PROFILE_NUM                         1

extern esp_ble_adv_params_t srv_adv_params;
extern esp_ble_adv_data_t srv_adv_config;
extern esp_ble_adv_data_t srv_scan_rsp_config;
extern const esp_gatts_attr_db_t srv_gatt_db[];
extern uint16_t svc_handle_table[];

struct gatts_profile_inst {
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

extern struct gatts_profile_inst profile_tab[];

enum
{
		// here are the list of the attributes. Keep SRV_ATTR_NB as last
    SRV_BATT_SVC,
	SRV_BATT_MEAS_UUID, SRV_BATT_MEAS_VALUE,
	SRV_BATT_NTFY_CFG,

    SRV_ATTR_NB,
};



#endif
