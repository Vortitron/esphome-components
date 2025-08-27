#include "ethernet_spi.h"

#include <esp_system.h>
#include <esp_err.h>
#include <esp_eth.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "lwip/err.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ethernet_spi {

static const char *const TAG = "ethernet_spi";

static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *) event_data;

  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
      ESP_LOGI(TAG, "Ethernet Link Up");
      ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "Ethernet Link Down");
      break;
    case ETHERNET_EVENT_START:
      ESP_LOGI(TAG, "Ethernet Started");
      break;
    case ETHERNET_EVENT_STOP:
      ESP_LOGI(TAG, "Ethernet Stopped");
      break;
    default:
      break;
  }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;

  ESP_LOGI(TAG, "Ethernet Got IP Address");
  ESP_LOGI(TAG, "~~~~~~~~~~~");
  ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
  ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
  ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
  ESP_LOGI(TAG, "~~~~~~~~~~~");
}

float EthernetComponent::get_setup_priority() const { return setup_priority::ETHERNET; }

void EthernetComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Ethernet...");

  // Configure SPI device
  spi_device_interface_config_t devcfg = {
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0,
      .clock_speed_hz = 1000000,  // 1 MHz, adjust as needed
      .spics_io_num = this->cs_pin_,
      .queue_size = 1,
      .pre_cb = nullptr,
      .post_cb = nullptr,
  };
  spi_device_handle_t spi_handle;
  esp_err_t err = spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  // Configure W5500
  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(VSPI_HOST, &devcfg);
  w5500_config.int_gpio_num = this->interrupt_pin_;
  if (this->reset_pin_ >= 0) {
    w5500_config.reset_gpio_num = this->reset_pin_;
  }

  // Initialize Ethernet MAC
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.swap = true;  // Adjust based on your module
  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  if (mac == nullptr) {
    ESP_LOGE(TAG, "Failed to create W5500 MAC");
    this->mark_failed();
    return;
  }

  // Configure PHY (W5500 has an internal PHY)
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
  if (phy == nullptr) {
    ESP_LOGE(TAG, "Failed to create W5500 PHY");
    this->mark_failed();
    return;
  }

  // Create Ethernet driver
  eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  err = esp_eth_driver_install(&eth_config, &this->eth_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install Ethernet driver: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  // Set MAC address (custom or default)
  uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};  // Example MAC, customize as needed
  esp_eth_ioctl(this->eth_handle_, ETH_CMD_S_MAC_ADDR, mac_addr);

  // Start Ethernet
  err = esp_eth_start(this->eth_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start Ethernet: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Ethernet setup complete");
}

void EthernetComponent::loop() {
  /*static bool init = false;
  if (!init) {
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  */
}

void EthernetComponent::dump_config() {
  std::string eth_type;
  switch (this->type_) {
    case ETHERNET_TYPE_W5500:
      eth_type = "W5500";
      break;
    default:
      eth_type = "Unknown";
      break;
  }

  ESP_LOGCONFIG(TAG, "Ethernet:");
  ESP_LOGCONFIG(TAG, "  CLK Pin: %u", this->clk_pin_);
  ESP_LOGCONFIG(TAG, "  MISO Pin: %u", this->miso_pin_);
  ESP_LOGCONFIG(TAG, "  MOSI Pin: %u", this->mosi_pin_);
  ESP_LOGCONFIG(TAG, "  CS Pin: %u", this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  IRQ Pin: %u", this->interrupt_pin_);
  ESP_LOGCONFIG(TAG, "  Reset Pin: %d", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  Clock Speed: %d MHz", this->clock_speed_ / 1000000);
  ESP_LOGCONFIG(TAG, "  Type: %s", eth_type.c_str());
}

}  // namespace ethernet_spi
}  // namespace esphome
