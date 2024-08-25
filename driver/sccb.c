/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))


// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define SCCB_FREQ               CONFIG_SCCB_CLK_FREQ  /*!< I2C master frequency*/
#define WRITE_BIT               I2C_MASTER_WRITE      /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ       /*!< I2C master read */
#define ACK_CHECK_EN            0x1                   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0                   /*!< I2C ack value */
#define NACK_VAL                0x1                   /*!< I2C nack value */
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

static int sccb_i2c_port;
static bool sccb_owns_i2c_port;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle = NULL;
static uint8_t curr_slave_addr = 0;

static int UpdateDevice(uint8_t slv_addr) {
    if (slv_addr != curr_slave_addr) {
        i2c_master_bus_rm_device(dev_handle);
        curr_slave_addr = 0;
    }
    if (!dev_handle) {
        // Not initialized
        i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slv_addr,
        .scl_speed_hz = SCCB_FREQ,
        };
      ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
      curr_slave_addr = slv_addr;
    }
    return 0;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = SCCB_I2C_PORT_DEFAULT,
    .scl_io_num = pin_scl,
    .sda_io_num =pin_sda,
    .glitch_ignore_cnt = 7,
};
    return i2c_new_master_bus(&i2c_mst_config, &bus_handle);


    // memset(&conf, 0, sizeof(i2c_config_t));

    // sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    // sccb_owns_i2c_port = true;
    // ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    // conf.mode = I2C_MODE_MASTER;
    // conf.sda_io_num = pin_sda;
    // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.scl_io_num = pin_scl;
    // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.master.clk_speed = SCCB_FREQ;

    // if ((ret =  i2c_param_config(sccb_i2c_port, &conf)) != ESP_OK) {
    //     return ret;
    // }

    // return i2c_driver_install(sccb_i2c_port, conf.mode, 0, 0, 0);
}

int SCCB_Use_Port(int i2c_num) { // sccb use an already initialized I2C port
    if (sccb_owns_i2c_port) {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    sccb_i2c_port = i2c_num;
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    if (!sccb_owns_i2c_port) {
        return ESP_OK;
    }
    sccb_owns_i2c_port = false;
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }
    return i2c_del_master_bus(bus_handle);
}

uint8_t SCCB_Probe(void)
{
    uint8_t slave_addr = 0x0;

    for (size_t i = 0; i < CAMERA_MODEL_MAX; i++) {
        if (slave_addr == camera_sensor[i].sccb_addr) {
            continue;
        }
        slave_addr = camera_sensor[i].sccb_addr;
        esp_err_t ret = i2c_master_probe(bus_handle, slave_addr, 1000);
        if( ret == ESP_OK) {
            return slave_addr;
        }
    }
    return 0;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    ESP_ERROR_CHECK(UpdateDevice(slv_addr));
    uint8_t data=0;
    esp_err_t ret =  i2c_master_transmit_receive(dev_handle, &reg, 1, &data, 1, 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    ESP_ERROR_CHECK(UpdateDevice(slv_addr));
    const uint8_t all_data[2] = {reg, data};
    esp_err_t ret =  i2c_master_transmit(dev_handle, all_data, sizeof(all_data), 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    return -1;
    // uint8_t data=0;
    // esp_err_t ret = ESP_FAIL;
    // uint16_t reg_htons = LITTLETOBIG(reg);
    // uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) return -1;
    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, &data, NACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    // }
    // return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    return -1;
    // static uint16_t i = 0;
    // esp_err_t ret = ESP_FAIL;
    // uint16_t reg_htons = LITTLETOBIG(reg);
    // uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "W [%04x]=%02x %d fail\n", reg, data, i++);
    // }
    // return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    return ESP_FAIL;
    // uint16_t data = 0;
    // uint8_t *data_u8 = (uint8_t *)&data;
    // esp_err_t ret = ESP_FAIL;
    // uint16_t reg_htons = LITTLETOBIG(reg);
    // uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) return -1;

    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, &data_u8[1], ACK_VAL);
    // i2c_master_read_byte(cmd, &data_u8[0], NACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    // }
    // return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    return ESP_FAIL;
    // esp_err_t ret = ESP_FAIL;
    // uint16_t reg_htons = LITTLETOBIG(reg);
    // uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    // uint16_t data_htons = LITTLETOBIG(data);
    // uint8_t *data_u8 = (uint8_t *)&data_htons;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, data_u8[0], ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, data_u8[1], ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    // }
    // return ret == ESP_OK ? 0 : -1;
}
