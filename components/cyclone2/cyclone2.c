#include "cyclone2.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hidh.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if !CONFIG_IDF_TARGET_ESP32
#error                                                                         \
    "Este driver requer ESP32 original com Bluetooth Classic (ESP32-WROOM/DevKit)."
#endif
#if !CONFIG_BT_BLUEDROID_ENABLED || !CONFIG_BT_CLASSIC_ENABLED ||              \
    !CONFIG_BT_HID_HOST_ENABLED
#error "Habilite Bluedroid, Bluetooth Classic e HID Host no menuconfig."
#endif

static const char *TAG = "CYCLONE2";
static uint8_t s_mac[6];
static cyclone2_callback_t s_callback;
static void *s_callback_arg;
static atomic_bool s_started = ATOMIC_VAR_INIT(false);

typedef enum {
  BT_WAIT_HOST,
  BT_IDLE,
  BT_SCANNING,
  BT_CONNECTING,
  BT_CONNECTED
} link_state_t;
static atomic_int s_link = ATOMIC_VAR_INIT(BT_WAIT_HOST);
static atomic_bool s_scan_done = ATOMIC_VAR_INIT(false);
static atomic_bool s_mac_visible = ATOMIC_VAR_INIT(false);
static atomic_int_fast64_t s_connect_start_us = ATOMIC_VAR_INIT(0);

// Last fully decoded packet, safe to read from another task.
static portMUX_TYPE s_gamepad_mux = portMUX_INITIALIZER_UNLOCKED;
static cyclone2_gamepad_t s_last_gamepad;
static int64_t s_last_input_us;
static bool s_has_gamepad;

static void clear_gamepad(void) {
  portENTER_CRITICAL(&s_gamepad_mux);
  memset(&s_last_gamepad, 0, sizeof(s_last_gamepad));
  s_last_input_us = 0;
  s_has_gamepad = false;
  portEXIT_CRITICAL(&s_gamepad_mux);
}

static bool parse_mac(const char *str, uint8_t dst[6]) {
  if (!str || strlen(str) != 17)
    return false;
  unsigned v[6];
  char tail = 0;
  int fields = sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x%c", &v[0], &v[1], &v[2],
                      &v[3], &v[4], &v[5], &tail);
  if (fields != 6)
    return false;
  for (int i = 0; i < 6; ++i)
    dst[i] = (uint8_t)v[i];
  return true;
}

static bool wanted_addr(const uint8_t *addr) {
  return addr != NULL && memcmp(addr, s_mac, sizeof(s_mac)) == 0;
}

static void send_app_event(cyclone2_event_type_t type, uint16_t id,
                           uint8_t map_index, const uint8_t *data, size_t len) {
  cyclone2_event_t ev = {
      .type = type,
      .report_id = id,
      .map_index = map_index,
      .data = data,
      .length = len,
  };
  if (type == CYCLONE2_INPUT) {
    if (cyclone2_parse(id, data, len, &ev.gamepad)) {
      portENTER_CRITICAL(&s_gamepad_mux);
      s_last_gamepad = ev.gamepad;
      s_last_input_us = esp_timer_get_time();
      s_has_gamepad = true;
      portEXIT_CRITICAL(&s_gamepad_mux);
    }
  }
  if (s_callback)
    s_callback(&ev, s_callback_arg);
}

static void gap_callback(esp_bt_gap_cb_event_t event,
                         esp_bt_gap_cb_param_t *param) {
  switch (event) {
  case ESP_BT_GAP_DISC_RES_EVT: {
    const uint8_t *bda = param->disc_res.bda;
    char name[64] = "";
    for (int i = 0; i < param->disc_res.num_prop; i++) {
      esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
      const uint8_t *p = NULL;
      size_t n = 0;
      if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME && prop->val &&
          prop->len > 0) {
        p = (const uint8_t *)prop->val;
        n = prop->len;
      } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR && prop->val) {
        uint8_t eir_len = 0;
        p = esp_bt_gap_resolve_eir_data(
            (uint8_t *)prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &eir_len);
        if (p == NULL) {
          p = esp_bt_gap_resolve_eir_data(
              (uint8_t *)prop->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &eir_len);
        }
        n = eir_len;
      }
      if (p && n) {
        if (n >= sizeof(name))
          n = sizeof(name) - 1;
        memcpy(name, p, n);
        name[n] = '\0';
      }
    }
    const bool alvo = wanted_addr(bda);
    ESP_LOGI(TAG, "SCAN BT %02X:%02X:%02X:%02X:%02X:%02X nome='%s'%s", bda[0],
             bda[1], bda[2], bda[3], bda[4], bda[5],
             name[0] ? name : "(nao informado)", alvo ? " [MAC ALVO]" : "");
    if (alvo)
      atomic_store(&s_mac_visible, true);
    break;
  }
  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      ESP_LOGI(TAG, "Varredura Bluetooth Classic finalizada");
      atomic_store(&s_scan_done, true);
    }
    break;
  case ESP_BT_GAP_AUTH_CMPL_EVT:
    if (wanted_addr(param->auth_cmpl.bda)) {
      ESP_LOGI(TAG, "Autenticacao: %s (status=%d)",
               param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS ? "OK" : "FALHOU",
               param->auth_cmpl.stat);
    }
    break;
  case ESP_BT_GAP_CFM_REQ_EVT:
    // No keyboard/screen on the ESP32. Just Works is appropriate only
    // for pairing with the intended local physical controller.
    if (wanted_addr(param->cfm_req.bda)) {
      ESP_LOGI(TAG, "Confirmacao SSP do controle alvo");
      esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    }
    break;
  case ESP_BT_GAP_PIN_REQ_EVT:
    if (wanted_addr(param->pin_req.bda)) {
      ESP_LOGI(TAG, "Controle solicitou PIN Bluetooth Classic");
      esp_bt_pin_code_t pin = {0};
      uint8_t len = 4;
      // Many legacy HID devices use 0000. For 16-digit PIN we send all zeros.
      if (param->pin_req.min_16_digit)
        len = 16;
      memset(pin, '0', len);
      esp_bt_gap_pin_reply(param->pin_req.bda, true, len, pin);
    }
    break;
  default:
    break;
  }
}

static void hid_callback(void *arg, esp_event_base_t base, int32_t id,
                         void *event_data) {
  (void)arg;
  (void)base;
  esp_hidh_event_data_t *p = (esp_hidh_event_data_t *)event_data;
  switch ((esp_hidh_event_t)id) {
  case ESP_HIDH_START_EVENT:
    ESP_LOGI(TAG, "HID Host iniciado: %s", esp_err_to_name(p->start.status));
    if (p->start.status == ESP_OK) {
      // O evento START pode chegar enquanto a task esta sendo criada.
      // Nao sobrescrever estados posteriores (CONNECTING/CONNECTED).
      int expected = BT_WAIT_HOST;
      atomic_compare_exchange_strong(&s_link, &expected, BT_IDLE);
      ESP_LOGI(TAG, "DIAG-R3: START recebido, estado=%d", atomic_load(&s_link));
    } else {
      ESP_LOGE(TAG, "DIAG-R3: falha ao iniciar HID Host; conexao bloqueada");
    }
    break;
  case ESP_HIDH_OPEN_EVENT: {
    const uint8_t *addr =
        p->open.dev ? esp_hidh_dev_bda_get(p->open.dev) : NULL;
    if (addr != NULL && !wanted_addr(addr)) {
      ESP_LOGW(TAG, "Dispositivo HID nao esperado: recusando");
      if (p->open.status == ESP_OK)
        esp_hidh_dev_close(p->open.dev);
      break;
    }
    if (p->open.status == ESP_OK) {
      clear_gamepad();
      atomic_store(&s_link, BT_CONNECTED);
      ESP_LOGI(TAG,
               "CONECTADO a %02X:%02X:%02X:%02X:%02X:%02X; nome=%s; VID=%04x "
               "PID=%04x",
               s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
               esp_hidh_dev_name_get(p->open.dev),
               esp_hidh_dev_vendor_id_get(p->open.dev),
               esp_hidh_dev_product_id_get(p->open.dev));
      // Prints descriptor/report information once, helpful for determining
      // layout.
      esp_hidh_dev_dump(p->open.dev, stdout);
      send_app_event(CYCLONE2_CONNECTED, 0, 0, NULL, 0);
    } else {
      ESP_LOGW(TAG, "Falha ao abrir HID: %s; nova tentativa",
               esp_err_to_name(p->open.status));
      atomic_store(&s_link, BT_IDLE);
    }
    break;
  }
  case ESP_HIDH_INPUT_EVENT: {
    if (atomic_load(&s_link) != BT_CONNECTED)
      break;
    const uint8_t *addr = esp_hidh_dev_bda_get(p->input.dev);
    if (!wanted_addr(addr))
      break;
    // ESP-IDF has removed the Report ID from p->input.data.
    send_app_event(CYCLONE2_INPUT, p->input.report_id, p->input.map_index,
                   p->input.data, p->input.length);
    break;
  }
  case ESP_HIDH_CLOSE_EVENT: {
    const uint8_t *addr =
        p->close.dev ? esp_hidh_dev_bda_get(p->close.dev) : NULL;
    if (addr == NULL || wanted_addr(addr)) {
      ESP_LOGW(TAG, "DESCONECTADO; tentando novamente");
      atomic_store(&s_link, BT_IDLE);
      clear_gamepad();
      send_app_event(CYCLONE2_DISCONNECTED, 0, 0, NULL, 0);
    }
    // ESP-IDF 6.x frees the device after this event callback.
    break;
  }
  default:
    break;
  }
}

static void reconnect_task(void *arg) {
  (void)arg;
  bool warned_stall = false;
  unsigned wait_host_count = 0;

  ESP_LOGI(TAG, "DIAG-R3: task cyclone2_link EXECUTANDO; estado=%d",
           atomic_load(&s_link));

  for (;;) {
    int state = atomic_load(&s_link);
    if (state == BT_WAIT_HOST) {
      // Nunca silenciar um driver parado antes da primeira tentativa.
      if (++wait_host_count % 10 == 0) {
        ESP_LOGW(TAG, "DIAG-R3: aguardando START do HID; estado=%d", state);
      }
    } else if (state == BT_IDLE) {
      warned_stall = false;
      wait_host_count = 0;
      ESP_LOGI(TAG,
               "DIAG-R3: abrindo HID Classic DIRETO para "
               "%02X:%02X:%02X:%02X:%02X:%02X",
               s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5]);
      // O MAC ja foi confirmado por bluetoothctl; scan nao e necessario.
      atomic_store(&s_link, BT_CONNECTING);
      atomic_store(&s_connect_start_us, esp_timer_get_time());
      esp_hidh_dev_t *dev = esp_hidh_dev_open(s_mac, ESP_HID_TRANSPORT_BT, 0);
      if (dev == NULL) {
        ESP_LOGW(TAG, "DIAG-R3: esp_hidh_dev_open retornou NULL");
        atomic_store(&s_link, BT_IDLE);
        vTaskDelay(pdMS_TO_TICKS(3000));
        continue;
      }
      ESP_LOGI(TAG, "DIAG-R3: abertura solicitada; aguardando OPEN/AUTH");
    } else if (state == BT_CONNECTING && !warned_stall) {
      int64_t elapsed = esp_timer_get_time() - atomic_load(&s_connect_start_us);
      if (elapsed > 20000000) {
        warned_stall = true;
        ESP_LOGE(TAG, "DIAG-R3: sem resposta HID apos 20s. "
                      "Nao duplicar esp_hidh_dev_open; reinicie o ESP32 "
                      "para uma nova tentativa.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Executa a inicializacao HID fora de app_main. Se a API do ESP-IDF
// permanecer bloqueada, ainda veremos a task de conexao e o loop principal.
// Versao apenas para diagnostico da inicializacao, nao um fix definitivo.
static void hid_init_task(void *arg) {
  (void)arg;
  esp_hidh_config_t cfg = {
      .callback = hid_callback,
      .event_stack_size = 6144,
      .callback_arg = NULL,
  };
  ESP_LOGI(TAG, "DIAG-R3: entrando em esp_hidh_init (task independente)");
  esp_err_t ret = esp_hidh_init(&cfg);
  ESP_LOGI(TAG, "DIAG-R3: esp_hidh_init RETORNOU: %s", esp_err_to_name(ret));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DIAG-R3: HID Host falhou: %s", esp_err_to_name(ret));
    atomic_store(&s_link, BT_WAIT_HOST);
  }
  vTaskDelete(NULL);
}

esp_err_t cyclone2_start(const char *mac, cyclone2_callback_t cb, void *arg) {
  uint8_t parsed[6];
  if (!parse_mac(mac, parsed))
    return ESP_ERR_INVALID_ARG;
  if (atomic_exchange(&s_started, true))
    return ESP_ERR_INVALID_STATE;
  memcpy(s_mac, parsed, 6);
  s_callback = cb;
  s_callback_arg = arg;

  esp_err_t ret;
  esp_bt_controller_config_t ctl_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  // Explicitly use BR/EDR only, not BLE.
  ctl_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
  ret = esp_bt_controller_init(&ctl_cfg);
  if (ret != ESP_OK)
    goto failed;
  ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  if (ret != ESP_OK)
    goto failed;
  esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ret = esp_bluedroid_init_with_cfg(&bd_cfg);
  if (ret != ESP_OK)
    goto failed;
  ret = esp_bluedroid_enable();
  if (ret != ESP_OK)
    goto failed;

  ret = esp_bt_gap_register_callback(gap_callback);
  if (ret != ESP_OK)
    goto failed;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  ret = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap,
                                      sizeof(iocap));
  if (ret != ESP_OK)
    goto failed;
  esp_bt_pin_code_t unused_pin = {0};
  ret = esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, unused_pin);
  if (ret != ESP_OK)
    goto failed;
  ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  if (ret != ESP_OK)
    goto failed;

  // Primeiro cria a task de conexao. Ela espera o evento START do HID.
  ESP_LOGI(TAG, "DIAG-R3: criando task de conexao ANTES do HID init");
  BaseType_t link_task_result =
      xTaskCreate(reconnect_task, "cyclone2_link", 4096, NULL, 4, NULL);
  if (link_task_result != pdPASS) {
    ret = ESP_ERR_NO_MEM;
    goto failed;
  }

  // A versao R2 nunca chegou ao log apos esp_hidh_init().
  // Executar essa chamada em outra task evita bloquear app_main().
  ESP_LOGI(TAG, "DIAG-R3: criando task de inicializacao HID");
  BaseType_t init_task_result =
      xTaskCreate(hid_init_task, "cyclone2_init", 6144, NULL, 4, NULL);
  if (init_task_result != pdPASS) {
    ESP_LOGE(TAG, "DIAG-R3: nao foi possivel criar cyclone2_init");
    ret = ESP_ERR_NO_MEM;
    goto failed;
  }
  ESP_LOGI(TAG, "DIAG-R3: ambas tasks criadas; retornando ao app_main");
  return ESP_OK;

failed:
  ESP_LOGE(TAG, "Inicializacao incompleta: %s", esp_err_to_name(ret));
  // On startup failure, require a reboot before retrying start(); Bluetooth
  // teardown is intentionally not attempted from this error path.
  return ret;
}

bool cyclone2_is_connected(void) {
  return atomic_load(&s_link) == BT_CONNECTED;
}

// Return false if disconnected, no successful parser, or more than 500 ms
// without fresh input. This prevents controls remaining stuck after RF loss.
bool cyclone2_get_gamepad(cyclone2_gamepad_t *out) {
  if (!out)
    return false;
  cyclone2_gamepad_t snapshot;
  int64_t received_us;
  bool valid;
  portENTER_CRITICAL(&s_gamepad_mux);
  snapshot = s_last_gamepad;
  received_us = s_last_input_us;
  valid = s_has_gamepad;
  portEXIT_CRITICAL(&s_gamepad_mux);
  if (!valid || !cyclone2_is_connected() ||
      esp_timer_get_time() - received_us > 500000) {
    memset(out, 0, sizeof(*out));
    return false;
  }
  *out = snapshot;
  return true;
}
