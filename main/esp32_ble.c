#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "BLE_SCAN";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
};

static void esp_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	
	switch (event) {
		
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
			ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
			// Scan parameters set, so start scanning for 30 seconds. 
			uint32_t duration = 30;
			esp_ble_gap_start_scanning(duration);
			break;
		}
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
			// Scan start complete event used to indicate start successful or not
			if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
				ESP_LOGE(TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
				break;
			}
			ESP_LOGI(TAG, "Scanning...");
			break;
			
		case ESP_GAP_BLE_SCAN_RESULT_EVT: {
			//ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_RESULT_EVT");
			esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
			switch (scan_result->scan_rst.search_evt) {
				case ESP_GAP_SEARCH_INQ_RES_EVT:
			        	//ESP_LOGI(TAG, "Inquiry result for a peer device");
						ESP_LOGI(TAG,"%02X:%02X:%02X:%02X:%02X:%02X RSSI:%02ddBm",	
								scan_result->scan_rst.bda[0],
								scan_result->scan_rst.bda[1],
								scan_result->scan_rst.bda[2],
								scan_result->scan_rst.bda[3],
								scan_result->scan_rst.bda[4],
								scan_result->scan_rst.bda[5],
								scan_result->scan_rst.rssi);
						break;
		        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
		        		ESP_LOGI(TAG, "Inquiry complete.");
		            	break;
		        default:
		        		ESP_LOGE(TAG, "Unknown Search Event");
		        		break;
			}
			break;
		}
		case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
			ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
			if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
				ESP_LOGE(TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
				break;
			}
			ESP_LOGI(TAG, "Scan Stopped...");
			break;
		
		default:
			ESP_LOGE(TAG, "Unknown GAP callback event %x", event);
			break;
	}
}

void app_main(void)
{
	ESP_LOGI(TAG, "ESP32_BLE Test Program");

	esp_err_t ret;
	
    // Initialize NVS. Required for RF parameters.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_callback);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    
}