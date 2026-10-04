#include "bluetooth.h"
#include "esp_bt.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

// 9020013050045701DCPNFD45 solid chave

#define TAG "bluetooth"

static void bt_gap_callback(esp_bt_gap_cb_event_t event,
                            esp_bt_gap_cb_param_t *param) {
  const char *BT_TAG = "esp_bt_gap_cb";

  switch (event) {
  case ESP_BT_GAP_DISC_RES_EVT: {
    for (int i = 0; i < param->disc_res.num_prop; i++) {
      esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
      ESP_LOGI(BT_TAG, "Propriedade %d: tipo=%d len=%d", i, prop->type,
               prop->len);

      switch (prop->type) {
      case ESP_BT_GAP_DEV_PROP_BDNAME: {
        char bd_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

        memset(bd_name, 0, sizeof(bd_name));

        int len = prop->len;

        if (len > ESP_BT_GAP_MAX_BDNAME_LEN) {
          len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        memcpy(bd_name, prop->val, len);

        bd_name[len] = '\0';

        ESP_LOGI(BT_TAG, "BDNAME: %s", bd_name);
        break;
      }
      case ESP_BT_GAP_DEV_PROP_COD:
        ESP_LOGI(BT_TAG, "CoD: 0x%06" PRIX32, *(uint32_t *)prop->val);
        break;

      case ESP_BT_GAP_DEV_PROP_EIR:
        ESP_LOGI(BT_TAG, "EIR recebido, %d bytes", prop->len);

        ESP_LOG_BUFFER_HEX(BT_TAG, prop->val, prop->len);
        break;

      case ESP_BT_GAP_DEV_PROP_RSSI:
        ESP_LOGI(BT_TAG, "RSSI: %d dBm", *(int8_t *)prop->val);
        break;
      }
    }
    break;
  }
  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
      ESP_LOGI(BT_TAG, "busca iniciada");
      break;
    }

    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      ESP_LOGI(BT_TAG, "busca finalizada");
      break;
    }
  }

  default:
    break;
  }
}
void init_bluetooth() {

  ESP_LOGI(TAG, "iniciando nvs");
  esp_err_t nvs_status = nvs_flash_init();

  if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());

    nvs_status = nvs_flash_init();
  }

  ESP_ERROR_CHECK(nvs_status);

  // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_default_config =
      BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  esp_err_t bt_controller_status = esp_bt_controller_init(&bt_default_config);

  if (bt_controller_status != ESP_OK) {
    ESP_LOGE(TAG, "erro ao configurar o controller bluetooth: %s",
             esp_err_to_name(bt_controller_status));
    return;
  }

  bt_controller_status = esp_bt_controller_enable(ESP_BT_MODE_BTDM);

  if (bt_controller_status != ESP_OK) {
    ESP_LOGE(TAG, "erro ao iniciar o controller %s",
             esp_err_to_name(bt_controller_status));
    return;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

  esp_err_t bluedroid_status = esp_bluedroid_init_with_cfg(&bluedroid_cfg);

  if (bluedroid_status != ESP_OK) {
    ESP_LOGE(TAG, "falha ao inicializar bluedroid: %s",
             esp_err_to_name(bluedroid_status));
    return;
  }

  bluedroid_status = esp_bluedroid_enable();

  if (bluedroid_status != ESP_OK) {
    ESP_LOGE(TAG, "erro ao iniciar bluedroid: %s",
             esp_err_to_name(bluedroid_status));
    return;
  }

  ESP_LOGI(TAG, "bluetooth pronto");

  bluedroid_status = esp_bt_gap_register_callback(bt_gap_callback);

  if (bluedroid_status != ESP_OK) {
    ESP_LOGE(TAG, "falha ao registrar callback gap: %s",
             esp_err_to_name(bluedroid_status));

    return;
  }

  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}
