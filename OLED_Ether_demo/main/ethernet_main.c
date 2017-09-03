/* ethernet Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"

#include "rom/ets_sys.h"
#include "rom/gpio.h"

#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
//#include "tlk110_phy.h"
#include "driver/rtc_io.h"

///////////
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "xi2c.h"
#include "fonts.h"
#include "ssd1306.h"
////////////

static const char *TAG = "eth_example";

#define DEFAULT_PHY_CONFIG (AUTO_MDIX_ENABLE|AUTO_NEGOTIATION_ENABLE|AN_1|AN_0|LED_CFG)
#define PIN_PHY_POWER 17
#define PIN_SMI_MDC   23
#define PIN_SMI_MDIO  2
#define PIN_CRS_DV    27
#define PHY_RESET_CONTROL_REG (0x0)
#define SOFTWARE_RESET BIT(15)
#define SOFTWARE_AUTO_NEGTIATION BIT(12)

#define BASIC_MODE_STATUS_REG (0x1)
#define AUTO_NEGOTIATION_COMPLETE BIT(5)
#define LINK_STATUS BIT(2)

#define AUTO_NEG_ADVERTISEMENT_REG (0x4)
#define ASM_DIR BIT(11)
#define PAUSE BIT(10)

#define PHY_LINK_PARTNER_ABILITY_REG (0x5)
#define PARTNER_ASM_DIR BIT(11)
#define PARTNER_PAUSE BIT(10)

#define PHY_STATUS_REG (0x1f)
#define AUTO_NEGTIATION_STATUS BIT(12)
#define DUPLEX_STATUS BIT(4)
#define SPEED_STATUS BIT(2)

//////////////OLED definition////////////////////
#define I2C_EXAMPLE_MASTER_SCL_IO    14    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO    13    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

xSemaphoreHandle print_mux;
/////////////////////////////////////////////////


void display_address(uint16_t ip_param);


void phy_tlk110_check_phy_init(void)
{
    while((esp_eth_smi_read(BASIC_MODE_STATUS_REG) & AUTO_NEGOTIATION_COMPLETE ) != AUTO_NEGOTIATION_COMPLETE)
    {};
    while((esp_eth_smi_read(PHY_STATUS_REG) & AUTO_NEGTIATION_STATUS ) != AUTO_NEGTIATION_STATUS)
    {};
}

eth_speed_mode_t phy_tlk110_get_speed_mode(void)
{
    if((esp_eth_smi_read(PHY_STATUS_REG) & SPEED_STATUS ) != SPEED_STATUS) {
        return ETH_SPEED_MODE_100M;
    } else {
        return ETH_SPEED_MODE_10M;
    }
}

eth_duplex_mode_t phy_tlk110_get_duplex_mode(void)
{
    if((esp_eth_smi_read(PHY_STATUS_REG) & DUPLEX_STATUS ) == DUPLEX_STATUS) {
        return ETH_MODE_FULLDUPLEX;
    } else {
        return ETH_MODE_HALFDUPLEX;
    }
}

bool phy_tlk110_check_phy_link_status(void)
{
    if((esp_eth_smi_read(BASIC_MODE_STATUS_REG) & LINK_STATUS) == LINK_STATUS ) {
        return true;
    } else {
        return false;
    }
}

bool phy_tlk110_get_partner_pause_enable(void)
{
    if((esp_eth_smi_read(PHY_LINK_PARTNER_ABILITY_REG) & PARTNER_PAUSE) == PARTNER_PAUSE) {
        return true;
    } else {
        return false;
    }
}

void phy_enable_flow_ctrl(void)
{
    uint32_t data = 0;
    data = esp_eth_smi_read(AUTO_NEG_ADVERTISEMENT_REG);
    esp_eth_smi_write(AUTO_NEG_ADVERTISEMENT_REG,data|ASM_DIR|PAUSE);
}

void phy_tlk110_power_enable(bool enable)
{
}

void phy_tlk110_init(void)
{
    gpio_set_level(GPIO_NUM_18, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_18, 1);
    
    esp_eth_smi_write(PHY_RESET_CONTROL_REG, SOFTWARE_RESET);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    esp_eth_smi_write(PHY_RESET_CONTROL_REG, SOFTWARE_AUTO_NEGTIATION);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    //if config.flow_ctrl_enable == true ,enable this
    phy_enable_flow_ctrl();
}

void eth_gpio_config_rmii(void)
{
    //txd0 to gpio19 ,can not change
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO19_U, FUNC_GPIO19_EMAC_TXD0);
    //tx_en to gpio21 ,can not change
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO21_U, FUNC_GPIO21_EMAC_TX_EN);
    //txd1 to gpio22 , can not change
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO22_U, FUNC_GPIO22_EMAC_TXD1);
    //rxd0 to gpio25 , can not change
    gpio_set_direction(25, GPIO_MODE_INPUT);
    //rxd1 to gpio26 ,can not change
    gpio_set_direction(26, GPIO_MODE_INPUT);
    //rmii clk ,can not change
    gpio_set_direction(0, GPIO_MODE_INPUT);
    
    //if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_23)) rtc_gpio_deinit(GPIO_NUM_23);
    //if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_2)) rtc_gpio_deinit(GPIO_NUM_2);
    if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_23)) rtc_gpio_deinit(GPIO_NUM_23);
    if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_2)) rtc_gpio_deinit(GPIO_NUM_2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_23], PIN_FUNC_GPIO);
    gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLUP_ONLY);
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_matrix_out(GPIO_NUM_23, EMAC_MDC_O_IDX, 0, 0);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_2], PIN_FUNC_GPIO);
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_in(GPIO_NUM_2, EMAC_MDI_I_IDX, 0);
    gpio_matrix_out(GPIO_NUM_2, EMAC_MDO_O_IDX, 0, 0);
    
    if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_18)) rtc_gpio_deinit(GPIO_NUM_18);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_18], PIN_FUNC_GPIO);
    gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);
    gpio_set_level(GPIO_NUM_18, 1);
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
    gpio_matrix_out(GPIO_NUM_18, SIG_GPIO_OUT_IDX, 0, 0);
}
void eth_task(void *pvParameter)
{
    tcpip_adapter_ip_info_t ip;
    memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    char ip_str[4];
    
    while (1) {

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        SSD1306_GotoXY(2,4);
        SSD1306_Puts("ESP32 Monster brd",&Font_7x10, SSD1306_COLOR_WHITE);

        if (tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip) == 0) {
            ESP_LOGI(TAG, "\n~~~~~~~~~~~\n");
            ESP_LOGI(TAG, "ETHIP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "ETHPMASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "ETHPGW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "\n~~~~~~~~~~~\n");
///////////
  
            SSD1306_GotoXY(4, 16);
            SSD1306_Puts("IP:",&Font_7x10, SSD1306_COLOR_WHITE);
            display_address(ip4_addr1_16(&ip.ip));
            display_address(ip4_addr2_16(&ip.ip));
            display_address(ip4_addr3_16(&ip.ip));
            display_address(ip4_addr4_16(&ip.ip));
            
            SSD1306_GotoXY(4, 25);
            SSD1306_Puts("MK:",&Font_7x10, SSD1306_COLOR_WHITE);
            display_address(ip4_addr1_16(&ip.netmask));
            display_address(ip4_addr2_16(&ip.netmask));
            display_address(ip4_addr3_16(&ip.netmask));
            display_address(ip4_addr4_16(&ip.netmask));
            
            SSD1306_GotoXY(4, 34);
            SSD1306_Puts("GW:",&Font_7x10, SSD1306_COLOR_WHITE);
            display_address(ip4_addr1_16(&ip.gw));
            display_address(ip4_addr2_16(&ip.gw));
            display_address(ip4_addr3_16(&ip.gw));
            display_address(ip4_addr4_16(&ip.gw));
            
            SSD1306_GotoXY(4, 45);
            SSD1306_Puts("Ether connected!",&Font_7x10, SSD1306_COLOR_WHITE);

            
            /* Update screen, send changes to LCD */
            SSD1306_UpdateScreen();
            
            
        }
    }
}

void display_address(uint16_t ip_param){
    char ip_str2[4];
    sprintf(ip_str2,"%d",ip_param);
    SSD1306_Puts(ip_str2, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_Puts(".",&Font_7x10, SSD1306_COLOR_WHITE);
}


void i2c_test(void)
{
    SSD1306_GotoXY(40, 4);
    SSD1306_Puts("ESP32", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Go to location X = 20, Y = 25 */
    SSD1306_GotoXY(8, 25);
    SSD1306_Puts("I2C SH1106 OLED", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Go to location X = 15, Y = 45 */
    SSD1306_GotoXY(20, 45);
    SSD1306_Puts("Akbar Hashim", &Font_7x10, SSD1306_COLOR_WHITE);
    
    /* Update screen, send changes to LCD */
    SSD1306_UpdateScreen();
    
    //    while (1) {
    //      /* Invert pixels */
    //    SSD1306_ToggleInvert();
    
    /* Update screen */
    //  SSD1306_UpdateScreen();
    
    /* Make a little delay */
    //   vTaskDelay(50);
    //  }
    
}

/**
 * @brief i2c master initialization
 */
void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}


void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    i2c_example_master_init();
    SSD1306_Init();
    esp_err_t ret = ESP_OK;
    tcpip_adapter_init();
    esp_event_loop_init(NULL, NULL);

    eth_config_t config;
    config.phy_addr = PHY1;
    config.mac_mode = ETH_MODE_RMII;
    config.phy_init = phy_tlk110_init;
    config.gpio_config = eth_gpio_config_rmii;
    config.tcpip_input = tcpip_adapter_eth_input;
    config.phy_check_init = phy_tlk110_check_phy_init;
    config.phy_check_link = phy_tlk110_check_phy_link_status;
    config.phy_get_speed_mode = phy_tlk110_get_speed_mode;
    config.phy_get_duplex_mode = phy_tlk110_get_duplex_mode;
    //Only FULLDUPLEX mode support flow ctrl now!
    config.flow_ctrl_enable = true;
    config.phy_get_partner_pause_enable = phy_tlk110_get_partner_pause_enable;    
    config.phy_power_enable = phy_tlk110_power_enable;

    ret = esp_eth_init(&config);

    if(ret == ESP_OK) {
        esp_eth_enable();
        xTaskCreate(eth_task, "eth_task", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL);
    }

}



//void app_main()
//{
//    print_mux = xSemaphoreCreateMutex();
//    i2c_example_master_init();
//    SSD1306_Init();
    
//    xTaskCreate(i2c_test, "i2c_test", 1024, NULL, 10, NULL);
//}

