// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "gc032a.h"
#include "gc032a_regs.h"
#include "gc032a_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "gc032a";
#endif

#define H8(v) ((v)>>8)
#define L8(v) ((v)&0xff)

//#define REG_DEBUG_ON

static int read_reg(uint8_t slv_addr, const uint16_t reg)
{
    int ret = SCCB_Read(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value)
{
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }
    if ((uint8_t)old_value != value) {
        ESP_LOGI(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write(slv_addr, reg, value);//maybe not?
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "WRITE REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask)
{
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static void print_regs(uint8_t slv_addr)
{
//#ifdef DEBUG_PRINT_REG
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "REG list look ======================");
    for (size_t i = 0xf0; i <= 0xfe; i++) {
        ESP_LOGI(TAG, "reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 0 ===");
    write_reg(slv_addr, 0xfe, 0x00); // page 0
    for (size_t i = 0x00; i < 0xf0; i++) {
        ESP_LOGI(TAG, "p0 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 1 ===");
    write_reg(slv_addr, 0xfe, 0x01); // page 1
    for (size_t i = 0x00; i < 0xf0; i++) {
        ESP_LOGI(TAG, "p1 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 2 ===");
    write_reg(slv_addr, 0xfe, 0x02); // page 2
    for (size_t i = 0x00; i < 0xf0; i++) {
        ESP_LOGI(TAG, "p2 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
    ESP_LOGI(TAG, "\npage 3 ===");
    write_reg(slv_addr, 0xfe, 0x03); // page 3
    for (size_t i = 0x00; i < 0xf0; i++) {
        ESP_LOGI(TAG, "p3 reg[0x%02x] = 0x%02x", i, read_reg(slv_addr, i));
    }
//#endif
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if (ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint16_t (*regs)[2])
{
    int i = 0, ret = 0;
    while (!ret && regs[i][0] != REGLIST_TAIL) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}

static int reset(sensor_t *sensor)
{
    int ret;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, RESET_RELATED, 0xf0);
    if (ret) {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ret = write_regs(sensor->slv_addr, gc032a_default_regs);
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        set_reg_bits(sensor->slv_addr, 0xf7, 1, 0x01, 1); // PLL_mode1:div2en
        set_reg_bits(sensor->slv_addr, 0xf7, 7, 0x01, 1); // PLL_mode1:dvp mode
        set_reg_bits(sensor->slv_addr, 0xf8, 0, 0x3f, 8); //PLL_mode2 :divx4
        set_reg_bits(sensor->slv_addr, 0xfa, 4, 0x0f, 2); //vlk div mode :divide_by
    }
    // Insert a delay so any subsequent programming happens after this delay.
    vTaskDelay(100 / portTICK_PERIOD_MS);

    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    switch (pixformat) {
    case PIXFORMAT_RGB565:
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        ret = set_reg_bits(sensor->slv_addr, 0x44, 0, 0x1f, 6);  //RGB565
        break;

    case PIXFORMAT_YUV422:
        write_reg(sensor->slv_addr, 0xfe, 0x00);
        ret = set_reg_bits(sensor->slv_addr, 0x44, 0, 0x1f, 3);
        break;
    default:
        ESP_LOGW(TAG, "unsupport format");
        ret = -1;
        break;
    }
    if (ret == 0) {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat to: %u", pixformat);
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    ESP_LOGI(TAG, "set_framesize");
    int ret = 0;
    if (framesize > FRAMESIZE_VGA) {
        ESP_LOGW(TAG, "Invalid framesize: %u", framesize);
        framesize = FRAMESIZE_VGA;
    }
    sensor->status.framesize = framesize;
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    uint16_t row_s = (resolution[FRAMESIZE_VGA].height - h) / 2;
    uint16_t col_s = (resolution[FRAMESIZE_VGA].width - w) / 2;

    write_reg(sensor->slv_addr, 0xfe, 0x00);
    write_reg(sensor->slv_addr, P0_ROW_START_HIGH, H8(row_s)); // Row_start[8]
    write_reg(sensor->slv_addr, P0_ROW_START_LOW, L8(row_s)); // Row_start[7:0]
    write_reg(sensor->slv_addr, P0_COLUMN_START_HIGH, H8(col_s)); // Column_start[9:8]
    write_reg(sensor->slv_addr, P0_COLUMN_START_LOW, L8(col_s)); // Column_start[7:0]
    write_reg(sensor->slv_addr, P0_WINDOW_HEIGHT_HIGH, H8(h + 8)); //window_height [8]
    write_reg(sensor->slv_addr, P0_WINDOW_HEIGHT_LOW, L8(h + 8)); //window_height [7:0]
    write_reg(sensor->slv_addr, P0_WINDOW_WIDTH_HIGH, H8(w + 8)); //window_width [9:8]
    write_reg(sensor->slv_addr, P0_WINDOW_WIDTH_LOW, L8(w + 8)); //window_width [7:0]

    write_reg(sensor->slv_addr, P0_WIN_MODE, 0x01);
    write_reg(sensor->slv_addr, P0_OUT_WIN_HEIGHT_HIGH, H8(h));
    write_reg(sensor->slv_addr, P0_OUT_WIN_HEIGHT_LOW, L8(h));
    write_reg(sensor->slv_addr, P0_OUT_WIN_WIDTH_HIGH, H8(w));
    write_reg(sensor->slv_addr, P0_OUT_WIN_WIDTH_LOW, L8(w));

    if (ret == 0) {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    //print_regs(sensor->slv_addr);
    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, P0_CISCTL_MODE1, 0, 0x01, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, P0_CISCTL_MODE1, 1, 0x01, enable);
    if (ret == 0) {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = write_reg(sensor->slv_addr, 0xfe, 0x00);
    ret |= set_reg_bits(sensor->slv_addr, P0_DEBUG_MODE2, 3, 0x01, enable);
    if (ret == 0) {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
    }
    return ret;
}

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret > 0) {
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0;
    if (mask > 0xFF) {
        ESP_LOGE(TAG, "mask should not more than 0xff");
    } else {
        ret = read_reg(sensor->slv_addr, reg);
    }
    if (ret < 0) {
        return ret;
    }
    value = (ret & ~mask) | (value & mask);

    if (mask > 0xFF) {

    } else {
        ret = write_reg(sensor->slv_addr, reg, value);
    }
    return ret;
}

static int init_status(sensor_t *sensor)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.sharpness = 0;
    sensor->status.denoise = 0;
    sensor->status.ae_level = 0;
    sensor->status.gainceiling = 0;
    sensor->status.awb = 0;
    sensor->status.dcw = 0;
    sensor->status.agc = 0;
    sensor->status.aec = 0;
    sensor->status.hmirror = check_reg_mask(sensor->slv_addr, P0_CISCTL_MODE1, 0x01);
    sensor->status.vflip = check_reg_mask(sensor->slv_addr, P0_CISCTL_MODE1, 0x02);
    sensor->status.colorbar = 0;
    sensor->status.bpc = 0;
    sensor->status.wpc = 0;
    sensor->status.raw_gma = 0;
    sensor->status.lenc = 0;
    sensor->status.quality = 0;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;
    sensor->status.awb_gain = 0;
    sensor->status.agc_gain = 0;
    sensor->status.aec_value = 0;
    sensor->status.aec2 = 0;
    return 0;
}

static int set_dummy(sensor_t *sensor, int val)
{
    print_regs(sensor->slv_addr);
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val)
{
    ESP_LOGW(TAG, "Unsupported");
    return -1;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    set_reg_bits(sensor->slv_addr, 0x42, 1, 0x01, enable); // AWB enable
    if (!enable) {
        // Restore original settings.
        set_reg_bits(sensor->slv_addr, 0x77, 0, 0xff, 0x64);
        set_reg_bits(sensor->slv_addr, 0x78, 0, 0xff, 0x40);
        set_reg_bits(sensor->slv_addr, 0x79, 0, 0xff, 0x60);
        set_reg_bits(sensor->slv_addr, 0x4e, 0, 0x7, 0x0);
    }
    // Also turn off color correction.
    return set_reg_bits(sensor->slv_addr, 0x40, 5, 0x01, enable); // CC_en.
}

static int set_sharpness(sensor_t *sensor, int value)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    const bool enable = value > 0;
    // Turn off edge enhancement.
    return set_reg_bits(sensor->slv_addr, 0x40, 4, 0x1, enable);
}

static int set_denoise(sensor_t *sensor, int value)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    const bool enable = value > 0;
    // Turn off edge enhancement.
    return set_reg_bits(sensor->slv_addr, 0x40, 2, 0x1, enable);
}

static int set_dcw(sensor_t *sensor, int enable)
{
    write_reg(sensor->slv_addr, 0xfe, 0x02);
    uint8_t val = (enable << 1) | enable; 
    return set_reg_bits(sensor->slv_addr, 0x40, 0, 0x03, val); // Dark current correction.
}

static int set_raw_gma(sensor_t *sensor, int enable)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    if (enable) {
        const uint8_t gamma[] = {0x9, 0x14, 0x19, 0x1f, 0x26, 0x32, 0x45, 0x53, 0x69, 0x7d, 0x8f, 0x9d, 
                                0xa9, 0xbd, 0xcd, 0xd9, 0xe3, 0xea, 0xef, 0xf5, 0xf9, 0xff};
        const uint8_t y_gamma[] = {0x9, 0x12, 0x16, 0x1b, 0x29, 0x37, 0x46, 0x54, 0x6b, 0x79, 0x85, 0x90, 
                                0x9a, 0xaa, 0xb8, 0xc3, 0xcc, 0xd3, 0xdb, 0xe9, 0xf6, 0xff};
        for (int i=0; i< sizeof(gamma); i++) {
            set_reg_bits(sensor->slv_addr, 0x5a + i, 0, 0xff, gamma[i]);
            set_reg_bits(sensor->slv_addr, 0xba + i, 0, 0xff, y_gamma[i]);
        }
    } else {
        // Apply a linear mapping.
        const uint8_t gamma[] = {1, 3, 5, 7, 11, 15, 19, 23, 31, 38, 47, 55, 
                                63, 79, 95, 111, 127, 143, 159, 191, 223, 255};
        for (int i=0; i< sizeof(gamma); i++) {
            set_reg_bits(sensor->slv_addr, 0x5a + i, 0, 0xff, gamma[i]);
            set_reg_bits(sensor->slv_addr, 0xba + i, 0, 0xff, gamma[i]);
        }
    }
    return 0;
}


static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    return set_reg_bits(sensor->slv_addr, 0x4f, 0, 0x01, enable); // AEC enable
}

static int set_aec_value(sensor_t *sensor, int value)
{
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    set_reg_bits(sensor->slv_addr, 0x04, 0, 0xff, value & 0xff); // Exposure settings.
    return set_reg_bits(sensor->slv_addr, 0x03, 0, 0xf, (value >> 8) & 0xf);
}

static int set_gain_ctrl(sensor_t *sensor, int value)
{
    // Generally stratedgy is:
    // 1. Increase analog gain.
    // 2. Increase pregain if needed, starting from unity (0x20=1x, 0x40=2x, etc...).
    // 3. Increase postgain only if pregain maxed out, starting from unity (0x40=1x, 0x80=2x, etc...).
    // 4. Increase global gain until brightest pixel maps to 255 without clipping.
    write_reg(sensor->slv_addr, 0xfe, 0x00);
    // Value encodes the following:
    // Digital gains:
    const uint8_t global_gain = value & 0xff;
    const uint8_t pregain = (value >> 8) & 0xff;
    // Note this only control quantization mapping from 0 - 255.
    const uint8_t postgain = (value >> 16) & 0xff;
    // Analog gain:
    const uint8_t gain_code = (value >> 24) & 0xf;
    set_reg_bits(sensor->slv_addr, 0x48, 0, 0xf, gain_code);
    set_reg_bits(sensor->slv_addr, 0x71, 0, 0xff, pregain);
    set_reg_bits(sensor->slv_addr, 0x72, 0, 0xff, postgain);
    return set_reg_bits(sensor->slv_addr, 0x70, 0, 0xff, global_gain);
}

int gc032a_detect(int slv_addr, sensor_id_t *id)
{
    if (GC032A_SCCB_ADDR == slv_addr) {
        uint8_t MIDL = SCCB_Read(slv_addr, SENSOR_ID_LOW);
        uint8_t MIDH = SCCB_Read(slv_addr, SENSOR_ID_HIGH);
        uint16_t PID = MIDH << 8 | MIDL;
        if (GC032A_PID == PID) {
            id->PID = PID;
            return PID;
        } else {
            ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int gc032a_init(sensor_t *sensor)
{
    sensor->init_status = init_status;
    sensor->reset = reset;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast = set_dummy;
    sensor->set_brightness = set_dummy;
    sensor->set_saturation = set_dummy;
    sensor->set_sharpness = set_sharpness;
    sensor->set_denoise = set_denoise;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_whitebal;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_aec2 = set_dummy;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_aec_value = set_aec_value;

    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;

    sensor->set_dcw = set_dcw;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;

    sensor->set_raw_gma = set_raw_gma;
    sensor->set_lenc = set_dummy;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = NULL;
    sensor->set_pll = NULL;
    sensor->set_xclk = NULL;

    ESP_LOGD(TAG, "GC032A Attached");
    return 0;
}
