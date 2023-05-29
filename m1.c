#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "ssd1306.h"

#define GATTS_TAG "GATTS_DEMO"
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SVC_INST_ID 0

// UUID và thông tin dịch vụ BLE
#define SERVICE_UUID           0x00E0
#define CHARACTERISTIC_UUID    0x01E0
#define DEVICE_NAME            "Nhom 3 - lop 1"
#define MAX_ID_LENGTH          8

// Khai báo OLED
#define OLED_SCL_PIN          GPIO_NUM_22
#define OLED_SDA_PIN          GPIO_NUM_21

uint16_t gatts_if = ESP_GATT_IF_NONE;
esp_gatt_if_t app_gatt_if = ESP_GATT_IF_NONE;

// Biến lưu trữ ID từ điện thoại
char member_ids[3][MAX_ID_LENGTH];
int received_id_count = 0;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(ESP_BLE_ADV_DATA_RAW);
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(ESP_BLE_ADV_DATA_RAW);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data_raw((uint8_t *)param->reg.app_id, sizeof(param->reg.app_id));
            break;
        case ESP_GATTS_CONNECT_EVT:
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            break;
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            if (param->write.len > 0)
            {
                
                if (param->write.handle == handle_table[IDX_CHAR_VAL])
                {
                    char* id = (char*)param->write.value;
                    int id_length = param->write.len;
                    if (received_id_count < 3 && id_length <= MAX_ID_LENGTH)
                    {
                        strncpy(member_ids[received_id_count], id, id_length);
                        received_id_count++;

                        // Hiển thị ID lên OLED
                        ssd1306_clear_screen();
                        ssd1306_draw_string(0, 0, "Team Member IDs:");
                        for (int i = 0; i < received_id_count; i++)
                        {
                            char line[MAX_ID_LENGTH + 1];
                            strncpy(line, member_ids[i], MAX_ID_LENGTH);
                            line[MAX_ID_LENGTH] = '\0';
                            ssd1306_draw_string(0, (i+1)*8, line);
                        }
                        ssd1306_refresh();

                        vTaskDelay(1000 / portTICK_PERIOD_MS);  
                    }
                }
            }
            break;
        default:
            break;
    }
}

void ble_gap_init()
{
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(ESP_APP_ID);

    esp_ble_gap_set_device_name(DEVICE_NAME);

    uint8_t service_uuid[ESP_UUID_LEN_16] = {LO_UINT16(SERVICE_UUID), HI_UINT16(SERVICE_UUID)};
    esp_ble_gap_config_adv_data_raw(service_uuid, sizeof(service_uuid));
}

void oled_init()
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &i2c_cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    ssd1306_init();
    ssd1306_set_fixed_font(ssd1306xled_font6x8);
    ssd1306_clear_screen();
    ssd1306_refresh();
}

void app_main()
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_gap_init();
    oled_init();
}
