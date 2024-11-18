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

// Tắt các log không cần thiết để tiết kiệm năng lượng
#define DISABLE_LOGS

#ifndef DISABLE_LOGS
static const char *const TAG = "openhaystack";
#endif

// Thời gian quảng bá BLE và deep sleep
static const int ADVERTISING_DURATION_MS = 3000;  // 3 giây
static const int DEEP_SLEEP_DURATION_US = 120000000;  // 2 phút (micro giây)

// Thông số BLE quảng bá
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x0640, // 1s
    .adv_int_max = 0x0640, // 1s (giảm thiểu để tiết kiệm năng lượng)
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void OpenHaystack::dump_config() {
#ifndef DISABLE_LOGS
  ESP_LOGCONFIG(TAG, "OpenHaystack:");
  ESP_LOGCONFIG(TAG,
                "  Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                this->random_address_[0],
                this->random_address_[1],
                this->random_address_[2],
                this->random_address_[3],
                this->random_address_[4],
                this->random_address_[5]
  );
  ESP_LOGCONFIG(TAG,
                "  Advertising Key (first six digits): %02X %02X %02X %02X %02X %02X",
                this->advertising_key_[0],
                this->advertising_key_[1],
                this->advertising_key_[2],
                this->advertising_key_[3],
                this->advertising_key_[4],
                this->advertising_key_[5]
  );
#endif
}

void OpenHaystack::setup() {
#ifndef DISABLE_LOGS
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
#endif
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
    // Bắt đầu quảng bá
    esp_err_t err = esp_ble_gap_start_advertising(&ble_adv_params);
    if (err != ESP_OK) {
#ifndef DISABLE_LOGS
      ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %d", err);
#endif
    }
#ifndef DISABLE_LOGS
    ESP_LOGD(TAG, "Started advertising for 3 seconds...");
#endif
    
    // Quảng bá trong 3 giây
    vTaskDelay(ADVERTISING_DURATION_MS / portTICK_PERIOD_MS);

    // Dừng quảng bá
    err = esp_ble_gap_stop_advertising();
    if (err != ESP_OK) {
#ifndef DISABLE_LOGS
      ESP_LOGE(TAG, "esp_ble_gap_stop_advertising failed: %d", err);
#endif
    }
#ifndef DISABLE_LOGS
    ESP_LOGD(TAG, "Stopped advertising.");
#endif

    // Deep sleep trong 2 phút
#ifndef DISABLE_LOGS
    ESP_LOGD(TAG, "Entering deep sleep for 2 minutes...");
#endif
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);  // Thiết lập thời gian wakeup
    esp_deep_sleep_start();  // Bắt đầu deep sleep
  }
}

// Các hàm còn lại giữ nguyên...

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
  if (err != ESP_OK) {
#ifndef DISABLE_LOGS
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", err);
#endif
    return;
  }

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  err = esp_bluedroid_init();
  if (err != ESP_OK) {
#ifndef DISABLE_LOGS
    ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", err);
#endif
    return;
  }
  err = esp_bluedroid_enable();
  if (err != ESP_OK) {
#ifndef DISABLE_LOGS
    ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", err);
#endif
    return;
  }

  set_addr_from_key(global_openhaystack->random_address_, global_openhaystack->advertising_key_.data());
  set_payload_from_key(global_openhaystack->adv_data_, global_openhaystack->advertising_key_.data());

  err = esp_ble_gap_register_callback(OpenHaystack::gap_event_handler);
  if (err != ESP_OK) {
#ifndef DISABLE_LOGS
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", err);
#endif
    return;
  }
  err = esp_ble_gap_set_rand_addr(global_openhaystack->random_address_);
  if (err != ESP_OK) {
#ifndef DISABLE_LOGS
    ESP_LOGE(TAG, "esp_ble_gap_set_rand_addr failed: %s", esp_err_to_name(err));
#endif
    return;
  }

  esp_ble_gap_config_adv_data_raw((uint8_t *) &global_openhaystack->adv_data_, sizeof(global_openhaystack->adv_data_));
}

OpenHaystack *global_openhaystack = nullptr;

}  // namespace openhaystack
}  // namespace esphome

#endif
