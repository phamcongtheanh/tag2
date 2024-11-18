#include "openhaystack.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <freertos/task.h>
#include <esp_gap_ble_api.h>
#include <cstring>
#include "esphome/core/hal.h"

#ifdef USE_ARDUINO
#include <esp32-hal-bt.h>
#endif

namespace esphome {
namespace openhaystack {

static const char *const TAG = "openhaystack";

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x0640, // 1s
    .adv_int_max = 0x0C80, // 2s
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


void OpenHaystack::setup() {
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
  global_openhaystack = this;

  xTaskCreatePinnedToCore(OpenHaystack::ble_core_task,
                          "ble_task",  // name
                          10000,       // stack size (in words)
                          nullptr,     // input params
                          1,           // priority
                          nullptr,     // Handle, not needed
                          0            // core
  );
}

float OpenHaystack::get_setup_priority() const { return setup_priority::BLUETOOTH; }

void OpenHaystack::ble_core_task(void *params) {
  ble_setup();

  while (true) {
    // Quảng bá trong 5 giây
    esp_err_t err = esp_ble_gap_start_advertising(&ble_adv_params);
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Quảng bá trong 10 giây

    // Dừng quảng bá sau 5 giây
    err = esp_ble_gap_stop_advertising();

    // Đưa ESP32 vào chế độ deep sleep trong 120 giây
    esp_sleep_enable_timer_wakeup(120000000);  // Thiết lập thời gian wakeup 60 giây
    esp_deep_sleep_start();  // Bắt đầu deep sleep
  }
}

void OpenHaystack::set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key) {
  addr[0] = public_key[0] | 0b11000000;
  addr[1] = public_key[1];
  addr[2] = public_key[2];
  addr[3] = public_key[3];
  addr[4] = public_key[4];
  addr[5] = public_key[5];
}

void OpenHaystack::set_payload_from_key(uint8_t *payload, uint8_t *public_key) {
  /* copy last 22 bytes */
  memcpy(&payload[7], &public_key[6], 22);
  /* append two bits of public key */
  payload[29] = public_key[0] >> 6;
}

void OpenHaystack::ble_setup() {
  // Initialize non-volatile storage for the bluetooth controller
  esp_err_t err = nvs_flash_init();

#ifdef USE_ARDUINO

#else
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    // start bt controller
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
      esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
      err = esp_bt_controller_init(&cfg);
      while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
        ;
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
      err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    }
  }
#endif

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  err = esp_bluedroid_init();
  err = esp_bluedroid_enable();

  set_addr_from_key(global_openhaystack->random_address_, global_openhaystack->advertising_key_.data());
  set_payload_from_key(global_openhaystack->adv_data_, global_openhaystack->advertising_key_.data());

  err = esp_ble_gap_register_callback(OpenHaystack::gap_event_handler);
  err = esp_ble_gap_set_rand_addr(global_openhaystack->random_address_);

  esp_ble_gap_config_adv_data_raw((uint8_t *) &global_openhaystack->adv_data_, sizeof(global_openhaystack->adv_data_));
}

void OpenHaystack::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  esp_err_t err;
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
      err = esp_ble_gap_start_advertising(&ble_adv_params);
      break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      break;
    }
    default:
      break;
  }
}

OpenHaystack *global_openhaystack = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace openhaystack
}  // namespace esphome

#endif
