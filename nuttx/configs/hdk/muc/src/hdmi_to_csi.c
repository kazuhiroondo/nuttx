/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/camera/camera_ext.h>
#include <nuttx/camera/camera_ext_meta.h>
#include <nuttx/camera/camera_ext_defs.h>
#include <nuttx/device.h>
#include <nuttx/device_cam_ext.h>
#include <nuttx/device_mhb_cam.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/math.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/mhb/mhb_csi_camera.h>

struct dev_private_s
{
    uint8_t rst_n;
    uint8_t led_en;
    uint8_t spi_sel;
    struct i2c_dev_s* i2c;
};

struct dev_private_s s_data;

#define CAMERA_POWER_DELAY_US (500000)
#define BRIDGE_RESET_DELAY     50000 /* us */
#define BRIDGE_SETUP_DELAY     10000 /* us */
#define DEV_I2C_ADDR 0x0F

/*
 * This is the MHB(Mods Hi-Speed Bus) camera driver for
 * Sony IMS220 raw sensor
 *
 */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct cam_i2c_reg_array {
    const uint16_t reg_addr;
    const uint8_t data;
};

struct cam_i2c_reg_setting {
    const uint16_t size;
    struct cam_i2c_reg_array const *regs;
};

static int _i2c_write(uint16_t i2c_addr, uint8_t *addr, int addr_len)
{
    struct i2c_msg_s msg;
    int ret = 0, retries = CONFIG_MHB_CAMERA_I2C_RETRY;

    msg.addr   = i2c_addr;
    msg.flags  = I2C_M_NORESTART;
    msg.buffer = addr;
    msg.length = addr_len;

    do {
        ret = I2C_TRANSFER(s_data.i2c, &msg, 1);
        if (ret) {
            usleep(CONFIG_MHB_CAMERA_I2C_RETRY_DELAY_US);
            CAM_DBG("i2c err %d\n", ret);
        }
    } while (ret && --retries);

    if (ret || (CONFIG_MHB_CAMERA_I2C_RETRY - retries))
        CAM_ERR("%s I2C write retried %d of %d : ret %d\n",
                ret ? "FAIL":"INFO",
                CONFIG_MHB_CAMERA_I2C_RETRY - retries,
                CONFIG_MHB_CAMERA_I2C_RETRY, ret);

    return ret;
}

static int bridge_i2c_write_int(uint16_t i2c_addr, uint16_t regaddr,
                                uint32_t data, uint8_t size)
{
    uint8_t addr[10]; /* 2 bytes addr + 8 bytes data max */

    if (size > 8) {
        CAM_ERR("Too many I2C data to send (%d).", size);
        return -1;
    }

    /* Address - MSB first */
    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;

    if (size <= 4) {
        /* LSB first for 2/4 bytes bounded registers */
        switch (size) {
        case 4:
            addr[5] = (data >> 24) & 0xFF;
            addr[4] = (data >> 16) & 0xFF;
        case 2:
            addr[3] = (data >> 8) & 0xFF;
        case 1:
            addr[2] = data & 0xFF;;
            break;
        default:
            CAM_ERR("Invalid data length %d\n", size);
            return -1;
        }
    }

    int ret = _i2c_write(i2c_addr, addr, size + 2);
    if (ret != 0) {
        CAM_ERR("Failed i2c write 0x%08x to %02x  addr 0x%04x err %d\n",
                data, i2c_addr, regaddr, ret);
    }

    return ret;
}

static int bridge_i2c_write_raw(uint16_t i2c_addr, uint16_t regaddr,
                                const uint8_t *data, uint8_t size)
{
    uint8_t addr[10]; /* 2 bytes addr + 8 bytes data max */

    if (size > 8) {
        CAM_ERR("Too many I2C data to send (%d).", size);
        return -1;
    }

    /* Address - MSB first */
    addr[0] = (regaddr >> 8) & 0xFF;
    addr[1] = regaddr & 0xFF;
    memcpy(&addr[2], data, size);

    int ret = _i2c_write(i2c_addr, addr, size + 2);
    if (ret != 0) {
        CAM_ERR("Failed i2c write 0x%08x to %02x  addr 0x%04x err %d\n",
                data, i2c_addr, regaddr, ret);
    }

    return ret;
}

static const struct camera_ext_frmival_node frmival_res0[] = {
    {
        /* fps is set to 30 always - otherwise video record fails on phone */
        .numerator = 1,
        .denominator = 30,
        .user_data = (void *)0,
    },
};

static const struct camera_ext_frmsize_node _cam_frmsizes[] = {
    {
#ifdef CONFIG_MODS_HDMI_TO_CSI_1080P30
        .width = 1920,
        .height = 1080,
#else
        .width = 1280,
        .height = 720,
#endif
        .num_frmivals = ARRAY_SIZE(frmival_res0),
        .frmival_nodes = frmival_res0,
    },
};

// format for camera input
static const struct camera_ext_format_node _cam_formats[] = {
    {
        .name = "UYVY",
        .fourcc = V4L2_PIX_FMT_UYVY,
        .depth = 16,
        .num_frmsizes = ARRAY_SIZE(_cam_frmsizes),
        .frmsize_nodes = _cam_frmsizes,
    },
};

// ov7251 input
static const struct camera_ext_input_node _cam_inputs[] = {
    {
        .name = "TC358743",
        .type = CAM_EXT_INPUT_TYPE_CAMERA,
        .status = 0,
        .capabilities = CAMERA_EXT_STREAM_CAP_PREVIEW |
                        CAMERA_EXT_STREAM_CAP_VIDEO |
                        CAMERA_EXT_STREAM_CAP_SNAPSHOT,
        .num_formats = ARRAY_SIZE(_cam_formats),
        .format_nodes = _cam_formats,
    },
};

const struct camera_ext_format_db mhb_camera_format_db = {
    .num_inputs = ARRAY_SIZE(_cam_inputs),
    .input_nodes = _cam_inputs,
};

extern struct camera_ext_ctrl_db mhb_camera_ctrl_db;

struct mhb_cdsi_config mhb_camera_csi_config =
{
    .direction = 0,
    .mode = 0x01,            /* TSB_CDSI_MODE_CSI */

    .tx_num_lanes = 4,
    .rx_num_lanes = 0,       /* variable */
    .tx_bits_per_lane = 0,  /* variable */
    .rx_bits_per_lane = 0,  /* variable */

    .hs_rx_timeout = 0xffffffff,

    .framerate = 0, /* variable */

    .pll_frs = 0,
    .pll_prd = 0,
    .pll_fbd = 0,

    .width = 0,  /* variable */
    .height = 0, /* variable */
    .bpp = 0,    /* variable */

    .bta_enabled = 0,
    .continuous_clock = 0,
    .blank_packet_enabled = 0,
    .video_mode = 0,
    .color_bar_enabled = 0,
};

/* Device Ops */
static int _mhb_camera_get_csi_config(struct device *dev,
                          void *config)
{
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    const struct camera_ext_frmival_node *ival;

    ival = get_current_frmival_node(&mhb_camera_format_db, cfg);
    if (ival == NULL) {
        CAM_ERR("Failed to get current frame interval\n");
        return -1;
    }

    mhb_camera_csi_config.rx_num_lanes = 2;
    mhb_camera_csi_config.framerate = roundf((float)(ival->denominator) /
                                             (float)(ival->numerator));
    mhb_camera_csi_config.tx_bits_per_lane = 600000000;
#ifdef CONFIG_MODS_HDMI_TO_CSI_1080P30
    mhb_camera_csi_config.rx_bits_per_lane = 750000000;
#else
    mhb_camera_csi_config.rx_bits_per_lane = 600000000;
#endif

    *(struct mhb_cdsi_config **)config = &mhb_camera_csi_config;

    return 0;
}

static int reset_bridge(void)
{
    return 0;
}

static int _mhb_camera_soc_enable(struct device *dev, uint8_t bootmode)
{
    return 0;
}

static int _mhb_camera_soc_disable(struct device *dev)
{
    reset_bridge();

    usleep(CAMERA_POWER_DELAY_US);

    return 0;
}

static int _mhb_camera_stream_configure(struct device *dev)
{
    CAM_ERR("ONDO: started\n");

    return 0;
}


static int _mhb_camera_stream_enable(struct device *dev)
{
    start_metadata_task();

    int rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0004, 0x0CD7, 2);

    CAM_ERR("ONDO: stream enable \n");
    return rc;
}

static int _mhb_camera_stream_disable(struct device *dev)
{
    stop_metadata_task();

    return bridge_i2c_write_int(DEV_I2C_ADDR, 0x0004, 0x0004, 2);
}

#ifdef CONFIG_MODS_HDMI_TO_CSI_1080P30
/* EDID 1080p30 - no CEA extension block */
static const uint8_t edid[] = { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
                                0x52, 0x62, 0x09, 0x02, 0x01, 0x01, 0x01, 0x01,
                                0xff, 0x14, 0x01, 0x03, 0x80, 0xa0, 0x5a, 0x78,
                                0x0a, 0x0d, 0xc9, 0xa0, 0x57, 0x47, 0x98, 0x27,
                                0x12, 0x48, 0x4c, 0x2f, 0xcf, 0x00, 0x01, 0x00,
                                0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D,
                                0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
                                0x45, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1e,
                                0x66, 0x21, 0x50, 0xb0, 0x51, 0x00, 0x1b, 0x30,
                                0x40, 0x70, 0x36, 0x00, 0x3a, 0x84, 0x63, 0x00,
                                0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
                                0x4f, 0x53, 0x48, 0x49, 0x42, 0x41, 0x2d, 0x54,
                                0x56, 0x0a, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
                                0x00, 0x17, 0x4c, 0x0f, 0x51, 0x0f, 0x00, 0x0a,
                                0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0xc8 };
#else
/* EDID 720p60 */
static const uint8_t edid[] = { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
                                0x52, 0x62, 0x88, 0x88, 0x00, 0x88, 0x88, 0x88,
                                0x1C, 0x15, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
                                0x0A, 0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26,
                                0x0F, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,
                                0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D,
                                0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28,
                                0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x1E,
                                0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
                                0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00,
                                0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
                                0x6F, 0x73, 0x68, 0x69, 0x62, 0x61, 0x2D, 0x48,
                                0x32, 0x43, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD,
                                0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x0F, 0x1E, 0x0A,
                                0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x4F,
                                0x02, 0x03, 0x1A, 0x42, 0x47, 0x84, 0x13, 0x03,
                                0x02, 0x07, 0x06, 0x01, 0x23, 0x09, 0x07, 0x07,
                                0x66, 0x03, 0x0C, 0x00, 0x30, 0x00, 0x80, 0xE3,
                                0x00, 0x7F, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0,
                                0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xC4, 0x8E,
                                0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A,
                                0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00,
                                0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A,
                                0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7C,
                                0x43, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x98,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86,
                                0x00, 0x00, 0x00, 0x00, 0x75, 0x19, 0xF5, 0x3D,
                                0x88, 0x19, 0xF5, 0x3D, 0x9E, 0x19, 0xF5, 0x3D };
#endif

static void *bridge_init_thread(void *arg)
{
    usleep(5000000);

    gpio_direction_out(s_data.led_en, 1);
    usleep(CAMERA_POWER_DELAY_US);

    gpio_direction_out(s_data.rst_n, 1);
    usleep(CAMERA_POWER_DELAY_US);

    int rc  = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0004, 0x0004, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0002, 0x0F00, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0002, 0x0000, 2);

    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0006, 0x0018, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0014, 0x0000, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0016, 0x07FF, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0020, 0x80C8, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0022, 0x0213, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0140, 0x00000000, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0144, 0x00000000, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0148, 0x00000000, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x014C, 0x00000001, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0150, 0x00000001, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0210, 0x00002C00, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0214, 0x00000005, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0218, 0x00001F04, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x021C, 0x00000003, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0220, 0x00000104, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0224, 0x00004988, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0228, 0x0000000A, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x022C, 0x00000004, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0234, 0x00000007, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0238, 0x00000000, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0204, 0x00000001, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0518, 0x00000001, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x0500, 0xA3000083, 4);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8502, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8512, 0xFE, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8531, 0x00, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8534, 0x3E, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8533, 0x07, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8540, 0x0A8C, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8552, 0xD1, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8630, 0xB0, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8631, 0x041E, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8670, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8532, 0x80, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8536, 0x40, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x853F, 0x0A, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8543, 0x32, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8544, 0x10, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8545, 0x31, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8546, 0x2D, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x85AA, 0x0050, 2);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x85AF, 0xF6, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x85C7, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x85CB, 0x01, 1);

    int addrstart = 0x8C00;
    int edidlen = sizeof(edid);
    int offset = 0;
    while (offset < edidlen) {
        rc = bridge_i2c_write_raw(DEV_I2C_ADDR, addrstart + offset,
                                  &edid[offset], 8);
        if (rc != 0) {
            break;
        }
        offset += 8;
    }
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x85D1, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8560, 0x24, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8563, 0x11, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8564, 0x0F, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8574, 0x08, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8573, 0xC1, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8576, 0xA0, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8600, 0x00, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8602, 0xF3, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8603, 0x02, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8604, 0x0C, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8606, 0x05, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8607, 0x00, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8620, 0x22, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8640, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8641, 0x65, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8642, 0x07, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8652, 0x02, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8665, 0x10, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x870B, 0x2C, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x870C, 0x53, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x870D, 0xFF, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x870E, 0x30, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x9007, 0x10, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8531, 0x01, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x8534, 0x3F, 1);
    if (!rc) rc = bridge_i2c_write_int(DEV_I2C_ADDR, 0x854A, 0x01, 1);

    CAM_DBG("Bridge configured\n");

    return NULL;
}

int _mhb_camera_init(struct device *dev)
{
    struct device_resource *res;

    memset(&s_data, 0, sizeof(s_data));

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "rst_n");
    if (!res) {
        CAM_ERR("failed to get rst_n gpio\n");
        return -ENODEV;
    }
    s_data.rst_n = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "led_en");
    if (!res) {
        CAM_ERR("failed to get led_en gpio\n");
        return -ENODEV;
    }
    s_data.led_en = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "spi_sel");
    if (!res) {
        CAM_ERR("failed to get spi_sel gpio\n");
        return -ENODEV;
    }
    s_data.spi_sel = res->start;

    gpio_direction_out(s_data.spi_sel, 0);
    gpio_direction_out(s_data.rst_n, 0);
    gpio_direction_out(s_data.led_en, 0);

    camera_ext_register_format_db(&mhb_camera_format_db);
    camera_ext_register_control_db(&mhb_camera_ctrl_db);

    init_metadata_task();

    s_data.i2c = up_i2cinitialize(CONFIG_MHB_CAMERA_I2C_BUS_ID);

    if (s_data.i2c == NULL) {
        CAM_ERR("Failed to init I2C device\n");
        return -1;
    }
    I2C_SETFREQUENCY(s_data.i2c, 400000);

    pthread_t tid;
    if (pthread_create(&tid, NULL, &bridge_init_thread, NULL) != 0) {
        CAM_ERR("Failed to start bride init thread\n");
        return -1;
    }
    pthread_detach(tid);

    return 0;
}

static struct device_mhb_camera_dev_type_ops _mhb_camera_type_ops = {
    .soc_enable = _mhb_camera_soc_enable,
    .soc_disable = _mhb_camera_soc_disable,
    .stream_configure = _mhb_camera_stream_configure,
    .stream_enable = _mhb_camera_stream_enable,
    .stream_disable = _mhb_camera_stream_disable,
    .get_csi_config = _mhb_camera_get_csi_config,
};

static struct device_driver_ops _mhb_camera_driver_ops = {
    .probe    = &_mhb_camera_init,
    .type_ops = &_mhb_camera_type_ops,
};

struct device_driver hdmi_to_csi_camera_driver = {
    .type = DEVICE_TYPE_MHB_CAMERA_HW,
    .name = "HDMI_TO_CSI",
    .desc = "HDMI to CSI driver",
    .ops  = &_mhb_camera_driver_ops,
};
