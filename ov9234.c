// SPDX-License-Identifier: GPL-2.0-only
/*
 * OmniVision OV9234 Monochrome / IR CMOS Image Sensor Driver
 *
 * Target Kernel: Linux 7.x
 * Architecture: V4L2 Subdevice + Media Controller API (active subdev state)
 * Datasheet Reference: OV9234 Rev 2.01 (May 2017)
 */
#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV9234_DRIVER_NAME			"ov9234"

/* External Master Clock (XVCLK: 6 MHz to 27 MHz, 24 MHz typical) */
#define OV9234_XVCLK_MIN_HZ			6000000UL
#define OV9234_XVCLK_TYP_HZ			24000000UL
#define OV9234_XVCLK_MAX_HZ			27000000UL

/* Internal Clock & MIPI Link Rates */
#define OV9234_LINK_FREQ_360MBPS		180000000ULL /* 360 Mbps DDR */
#define OV9234_SYS_PIXEL_RATE_HZ		36000000ULL  /* 720 MHz VCO / 20 */

/* Power-up / Power-down XVCLK cycle constraints */
#define OV9234_PWRUP_MIN_XVCLK_CYCLES		8192U
#define OV9234_PWRUP_MIN_DELAY_US		1000U
#define OV9234_PWRDN_MIN_XVCLK_CYCLES		512U
#define OV9234_SW_RESET_DELAY_US		1000U

/* Physical Pixel Array Geometry (1296 x 736 total) */
#define OV9234_NATIVE_WIDTH			1296U
#define OV9234_NATIVE_HEIGHT			736U
#define OV9234_ACTIVE_START_LEFT		4U
#define OV9234_ACTIVE_START_TOP			4U
#define OV9234_ACTIVE_WIDTH			1280U
#define OV9234_ACTIVE_HEIGHT			720U

/* 6.1 General Status & System Control Registers[cite: 1] */
#define OV9234_REG_MODE_SELECT			0x0100
#define OV9234_MODE_STANDBY			0x00
#define OV9234_MODE_STREAMING			BIT(0)

#define OV9234_REG_SOFTWARE_RESET		0x0103
#define OV9234_SW_RESET_TRIG			BIT(0)

#define OV9234_REG_FAST_STANDBY_CTRL		0x0106
#define OV9234_REG_CCI_ADDRESS_PGM_ID		0x0107

/* 6.3 I/O & Chip Identification Registers[cite: 1] */
#define OV9234_REG_PCLK_HREF_VSYNC_OEN		0x3001
#define OV9234_REG_GPIO_OEN			0x3002
#define OV9234_REG_PAD_DRIVE_CAP		0x3009
#define OV9234_REG_CHIP_ID_HIGH			0x300a
#define OV9234_REG_CHIP_ID_LOW			0x300b
#define OV9234_CHIP_ID				0x9734 /* Shared with OV9734 */

#define OV9234_REG_PLL_PCLK_DIV			0x301e
#define OV9234_REG_CHIP_REVISION		0x302a
#define OV9234_CHIP_REV_A0			0xa0

#define OV9234_REG_PLL_DAC_SA1_DIV		0x3030
#define OV9234_REG_SCCB_CTRL			0x3031

/* 6.4 Clock & PLL Configuration Registers[cite: 1] */
#define OV9234_REG_PRE_PLL_CLK_DIV		0x3080
#define OV9234_REG_PLL_MULTIPLIER		0x3081
#define OV9234_REG_VT_SYS_CLK_DIV		0x3082
#define OV9234_REG_VT_PIXEL_CLK_DIV		0x3083
#define OV9234_REG_PLL_DAC_DIV			0x3084
#define OV9234_REG_PLL_MIPI_DIV			0x3089
#define OV9234_REG_PLL_PHY_DIV			0x308a
#define OV9234_REG_PLL_SLEEP_CTRL		0x308b
#define OV9234_PLL_SLEEP_GATE			BIT(0)
#define OV9234_REG_SYS_CLK_DIV			0x3103

/* 2.11 Group Write Atomic Update Control[cite: 1] */
#define OV9234_REG_GROUP_ACCESS			0x3208
#define OV9234_GROUP_HOLD_START_G0		0x00
#define OV9234_GROUP_HOLD_END_G0		0x10
#define OV9234_GROUP_LAUNCH_G0			0xa0

/* 6.10 AEC / AGC Control Registers[cite: 1] */
#define OV9234_REG_LONG_EXPO_HIGH		0x3500 /* Bits [3:0] = EXPO[19:16] */
#define OV9234_REG_LONG_EXPO_MID		0x3501 /* Bits [7:0] = EXPO[15:8] */
#define OV9234_REG_LONG_EXPO_LOW		0x3502 /* Bits [7:0] = EXPO[7:0] (Q16.4) */

#define OV9234_REG_AEC_MANUAL			0x3503
#define OV9234_AEC_MANUAL_EN			BIT(0)
#define OV9234_AGC_MANUAL_EN			BIT(1)
#define OV9234_VTS_MANUAL_EN			BIT(3)
#define OV9234_GAIN_DELAY_EN			BIT(4)
#define OV9234_GAIN_DELAY_1FRAME		BIT(5)

#define OV9234_REG_AEC_CTRL_9			0x3509
#define OV9234_REG_LONG_GAIN_HIGH		0x350a /* Bits [1:0] = GAIN[9:8] */
#define OV9234_REG_LONG_GAIN_LOW		0x350b /* Bits [7:0] = GAIN[7:0] (Q6.4) */

/* Exposure & Gain Constraints[cite: 1] */
#define OV9234_EXPO_MIN_LINES			4U
#define OV9234_EXPO_MARGIN_LINES		4U
#define OV9234_EXPO_DEFAULT_LINES		512U
#define OV9234_EXPO_Q16_4_SHIFT			4U

#define OV9234_ANA_GAIN_MIN			0x010 /* 1.0x gain in Q6.4 (16) */
#define OV9234_ANA_GAIN_MAX			0x0f8 /* 15.5x gain in Q6.4 (248) */
#define OV9234_ANA_GAIN_STEP			1U
#define OV9234_ANA_GAIN_DEFAULT			0x010

/* 6.6 Output Format & Standby Power Control[cite: 1] */
#define OV9234_REG_FMT_CTRL			0x3654
#define OV9234_FMT_CTRL_RAW8_SEL		BIT(5) /* 0: RAW10, 1: RAW8 */
#define OV9234_FMT_CTRL_DEFAULT			0x10

#define OV9234_REG_STANDBY_PWR_58		0x3658
#define OV9234_REG_STANDBY_PWR_59		0x3659
#define OV9234_REG_STANDBY_PWR_5A		0x365a

/* Binning Control[cite: 1] */
#define OV9234_REG_BIN_AVG			0x37c0
#define OV9234_BIN_AVG_ENABLE			BIT(2) /* 0: Sum, 1: Average */
#define OV9234_BIN_AVG_DEFAULT			0x07

/* 6.9 Frame Timing & Windowing Registers[cite: 1] */
#define OV9234_REG_X_ADDR_START_H		0x3800
#define OV9234_REG_X_ADDR_START_L		0x3801
#define OV9234_REG_Y_ADDR_START_H		0x3802
#define OV9234_REG_Y_ADDR_START_L		0x3803
#define OV9234_REG_X_ADDR_END_H			0x3804
#define OV9234_REG_X_ADDR_END_L			0x3805
#define OV9234_REG_Y_ADDR_END_H			0x3806
#define OV9234_REG_Y_ADDR_END_L			0x3807
#define OV9234_REG_X_OUTPUT_SIZE_H		0x3808
#define OV9234_REG_X_OUTPUT_SIZE_L		0x3809
#define OV9234_REG_Y_OUTPUT_SIZE_H		0x380a
#define OV9234_REG_Y_OUTPUT_SIZE_L		0x380b
#define OV9234_REG_HTS_H			0x380c
#define OV9234_REG_HTS_L			0x380d
#define OV9234_REG_VTS_H			0x380e
#define OV9234_REG_VTS_L			0x380f
#define OV9234_REG_ISP_X_WIN_H			0x3810
#define OV9234_REG_ISP_X_WIN_L			0x3811
#define OV9234_REG_ISP_Y_WIN_H			0x3812
#define OV9234_REG_ISP_Y_WIN_L			0x3813

#define OV9234_VTS_MAX				0x7fff

/* Orientation & Binning Registers[cite: 1] */
#define OV9234_REG_FORMAT0_ORIENTATION		0x3820
#define OV9234_ORIENT_VBIN_EN			BIT(0)
#define OV9234_ORIENT_HBIN_EN			BIT(1)
#define OV9234_ORIENT_BIN_MASK			(OV9234_ORIENT_HBIN_EN | OV9234_ORIENT_VBIN_EN)
#define OV9234_ORIENT_VFLIP_EN			BIT(2)
#define OV9234_ORIENT_HMIRROR_EN		BIT(3)
#define OV9234_ORIENT_BLC_FLIP_EN		BIT(4) /* Must match BIT(2) vflip */
#define OV9234_ORIENT_FLIP_MASK			(OV9234_ORIENT_BLC_FLIP_EN | \
						 OV9234_ORIENT_HMIRROR_EN | \
						 OV9234_ORIENT_VFLIP_EN)

#define OV9234_REG_FORMAT1_SUBSAMPLING		0x3821
#define OV9234_SUB_VSUB_MASK			GENMASK(1, 0)
#define OV9234_SUB_HSUB_MASK			GENMASK(3, 2)
#define OV9234_SUB_FULL				0x00
#define OV9234_SUB_HSUB2_VSUB2			0x05 /* hsub=01, vsub=01 */

/* 6.11 Black Level Calibration (BLC) & 6.13 MIPI Control[cite: 1] */
#define OV9234_REG_BLC_CTRL_00			0x4000
#define OV9234_REG_BLC_CTRL_01			0x4001
#define OV9234_REG_BLC_TARGET_H			0x4002
#define OV9234_REG_BLC_TARGET_L			0x4003

#define OV9234_REG_MIPI_CTRL00			0x4800
#define OV9234_MIPI_GATE_SC_EN			BIT(5)
#define OV9234_REG_MIPI_CTRL14			0x4814
#define OV9234_MIPI_DT_RAW8			0x2a
#define OV9234_MIPI_DT_RAW10			0x2b

/* 6.14 & 6.15 ISP, DPC & Test Pattern Control[cite: 1] */
#define OV9234_REG_ISP_CTRL00			0x5000
#define OV9234_ISP_BLC_EN			BIT(0)
#define OV9234_ISP_DCBLC_EN			BIT(1)
#define OV9234_ISP_WC_EN			BIT(2)
#define OV9234_ISP_BC_EN			BIT(3)
#define OV9234_ISP_CTRL00_DEFAULT		0x2f

#define OV9234_REG_PRE_ISP_CTRL0		0x5080
#define OV9234_TEST_PATTERN_EN			BIT(7)
#define OV9234_TEST_PATTERN_STYLE_MASK		GENMASK(3, 2)

#define OV9234_REG_DPC_CTRL00			0x5780

struct ov9234_reg {
	u16 address;
	u8 val;
};

struct ov9234_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u32 vts_min;
	u32 max_fps;
	struct v4l2_rect crop;
	bool binning_enabled;
	const struct ov9234_reg *reg_list;
	unsigned int num_regs;
};

/*
 * Common initialization sequence establishing default PLL (24 MHz XVCLK ->
 * 720 MHz VCO -> 360 Mbps MIPI / 36 MHz SYS_CLK), BLC, and DPC settings[cite: 1].
 */
static const struct ov9234_reg ov9234_common_init_regs[] = {
	{ OV9234_REG_PRE_PLL_CLK_DIV, 0x02 }, /* /2 -> 12 MHz PLL input */
	{ OV9234_REG_PLL_MULTIPLIER,  0x3c }, /* x60 -> 720 MHz VCO */
	{ OV9234_REG_VT_SYS_CLK_DIV,  0x04 }, /* /(1 + 4) = /5 */
	{ OV9234_REG_VT_PIXEL_CLK_DIV, 0x00 }, /* /2 -> PLL_SCLK */
	{ OV9234_REG_SYS_CLK_DIV,     0x01 }, /* /2 -> 36 MHz SYS_CLK */
	{ OV9234_REG_AEC_MANUAL,      OV9234_AEC_MANUAL_EN | OV9234_AGC_MANUAL_EN },
	{ OV9234_REG_ISP_CTRL00,      OV9234_ISP_CTRL00_DEFAULT },
	{ OV9234_REG_BLC_CTRL_00,     0xc1 },
	{ OV9234_REG_BLC_CTRL_01,     0xe0 },
	{ OV9234_REG_BLC_TARGET_H,    0x00 },
	{ OV9234_REG_BLC_TARGET_L,    0x10 }, /* Default black level = 16 */
	{ OV9234_REG_DPC_CTRL00,      0x14 }, /* 3x3 & cross-cluster DPC */
	{ OV9234_REG_MIPI_CTRL00,     0x04 | OV9234_MIPI_GATE_SC_EN },
};

/* Mode 0: 720p (1280x720) @ 30 fps full resolution readout (1288x728 array)[cite: 1] */
static const struct ov9234_reg ov9234_mode_720p_30fps_regs[] = {
	{ OV9234_REG_X_ADDR_START_H,  0x00 },
	{ OV9234_REG_X_ADDR_START_L,  0x04 }, /* x_addr_start = 4 */
	{ OV9234_REG_Y_ADDR_START_H,  0x00 },
	{ OV9234_REG_Y_ADDR_START_L,  0x04 }, /* y_addr_start = 4 */
	{ OV9234_REG_X_ADDR_END_H,    0x05 },
	{ OV9234_REG_X_ADDR_END_L,    0x0b }, /* x_addr_end = 1291 (1288 width) */
	{ OV9234_REG_Y_ADDR_END_H,    0x02 },
	{ OV9234_REG_Y_ADDR_END_L,    0xdb }, /* y_addr_end = 731 (728 height) */
	{ OV9234_REG_X_OUTPUT_SIZE_H, 0x05 },
	{ OV9234_REG_X_OUTPUT_SIZE_L, 0x00 }, /* 1280 */
	{ OV9234_REG_Y_OUTPUT_SIZE_H, 0x02 },
	{ OV9234_REG_Y_OUTPUT_SIZE_L, 0xd0 }, /* 720 */
	{ OV9234_REG_HTS_H,           0x05 },
	{ OV9234_REG_HTS_L,           0xc6 }, /* HTS = 1478 */
	{ OV9234_REG_VTS_H,           0x03 },
	{ OV9234_REG_VTS_L,           0x22 }, /* VTS = 802 */
	{ OV9234_REG_ISP_X_WIN_H,     0x00 },
	{ OV9234_REG_ISP_X_WIN_L,     0x04 }, /* isp_x_win = 4 */
	{ OV9234_REG_ISP_Y_WIN_H,     0x00 },
	{ OV9234_REG_ISP_Y_WIN_L,     0x04 }, /* isp_y_win = 4 */
	{ OV9234_REG_BIN_AVG,         OV9234_BIN_AVG_DEFAULT },
	{ OV9234_REG_FORMAT1_SUBSAMPLING, OV9234_SUB_FULL },
};

/* Mode 1: VGA (640x480) @ 45 fps center crop (648x488 array readout)[cite: 1] */
static const struct ov9234_reg ov9234_mode_vga_45fps_regs[] = {
	{ OV9234_REG_X_ADDR_START_H,  0x01 },
	{ OV9234_REG_X_ADDR_START_L,  0x44 }, /* x_addr_start = 324 */
	{ OV9234_REG_Y_ADDR_START_H,  0x00 },
	{ OV9234_REG_Y_ADDR_START_L,  0x7c }, /* y_addr_start = 124 */
	{ OV9234_REG_X_ADDR_END_H,    0x03 },
	{ OV9234_REG_X_ADDR_END_L,    0xcb }, /* x_addr_end = 971 (648 width) */
	{ OV9234_REG_Y_ADDR_END_H,    0x02 },
	{ OV9234_REG_Y_ADDR_END_L,    0x63 }, /* y_addr_end = 611 (488 height) */
	{ OV9234_REG_X_OUTPUT_SIZE_H, 0x02 },
	{ OV9234_REG_X_OUTPUT_SIZE_L, 0x80 }, /* 640 */
	{ OV9234_REG_Y_OUTPUT_SIZE_H, 0x01 },
	{ OV9234_REG_Y_OUTPUT_SIZE_L, 0xe0 }, /* 480 */
	{ OV9234_REG_HTS_H,           0x05 },
	{ OV9234_REG_HTS_L,           0xc6 }, /* HTS = 1478 */
	{ OV9234_REG_VTS_H,           0x02 },
	{ OV9234_REG_VTS_L,           0x1d }, /* VTS = 541 (~45 fps at 36 MHz) */
	{ OV9234_REG_ISP_X_WIN_H,     0x00 },
	{ OV9234_REG_ISP_X_WIN_L,     0x04 }, /* isp_x_win = 4 */
	{ OV9234_REG_ISP_Y_WIN_H,     0x00 },
	{ OV9234_REG_ISP_Y_WIN_L,     0x04 }, /* isp_y_win = 4 */
	{ OV9234_REG_BIN_AVG,         OV9234_BIN_AVG_DEFAULT },
	{ OV9234_REG_FORMAT1_SUBSAMPLING, OV9234_SUB_FULL },
};

/*
 * Mode 2: 360p (640x360) @ 60 fps 2x2 B&W binning (644x364 post-bin readout)
 * Horizontal neighbor pixel averaging + vertical neighbor subsampling[cite: 1].
 */
static const struct ov9234_reg ov9234_mode_360p_60fps_regs[] = {
	{ OV9234_REG_X_ADDR_START_H,  0x00 },
	{ OV9234_REG_X_ADDR_START_L,  0x04 }, /* x_addr_start = 4 */
	{ OV9234_REG_Y_ADDR_START_H,  0x00 },
	{ OV9234_REG_Y_ADDR_START_L,  0x04 }, /* y_addr_start = 4 */
	{ OV9234_REG_X_ADDR_END_H,    0x05 },
	{ OV9234_REG_X_ADDR_END_L,    0x0b }, /* x_addr_end = 1291 */
	{ OV9234_REG_Y_ADDR_END_H,    0x02 },
	{ OV9234_REG_Y_ADDR_END_L,    0xdb }, /* y_addr_end = 731 */
	{ OV9234_REG_X_OUTPUT_SIZE_H, 0x02 },
	{ OV9234_REG_X_OUTPUT_SIZE_L, 0x80 }, /* 640 */
	{ OV9234_REG_Y_OUTPUT_SIZE_H, 0x01 },
	{ OV9234_REG_Y_OUTPUT_SIZE_L, 0x68 }, /* 360 */
	{ OV9234_REG_HTS_H,           0x05 },
	{ OV9234_REG_HTS_L,           0xc6 }, /* HTS = 1478 */
	{ OV9234_REG_VTS_H,           0x01 },
	{ OV9234_REG_VTS_L,           0x96 }, /* VTS = 406 (~60 fps at 36 MHz) */
	{ OV9234_REG_ISP_X_WIN_H,     0x00 },
	{ OV9234_REG_ISP_X_WIN_L,     0x02 }, /* isp_x_win = 2 (644 -> 640) */
	{ OV9234_REG_ISP_Y_WIN_H,     0x00 },
	{ OV9234_REG_ISP_Y_WIN_L,     0x02 }, /* isp_y_win = 2 (364 -> 360) */
	{ OV9234_REG_BIN_AVG,         OV9234_BIN_AVG_DEFAULT | OV9234_BIN_AVG_ENABLE },
	{ OV9234_REG_FORMAT1_SUBSAMPLING, OV9234_SUB_HSUB2_VSUB2 },
};

/* Deep software sleep register sequences for minimum standby current (60 uA typ)[cite: 1] */
static const struct ov9234_reg ov9234_deep_sleep_on_regs[] = {
	{ OV9234_REG_MODE_SELECT,    OV9234_MODE_STANDBY },
	{ OV9234_REG_STANDBY_PWR_58, 0xff },
	{ OV9234_REG_STANDBY_PWR_59, 0xff },
	{ OV9234_REG_STANDBY_PWR_5A, 0xff },
	{ OV9234_REG_PLL_SLEEP_CTRL, OV9234_PLL_SLEEP_GATE },
};

static const struct ov9234_reg ov9234_deep_sleep_off_regs[] = {
	{ OV9234_REG_STANDBY_PWR_58, 0x00 },
	{ OV9234_REG_STANDBY_PWR_59, 0x00 },
	{ OV9234_REG_STANDBY_PWR_5A, 0x00 },
	{ OV9234_REG_PLL_SLEEP_CTRL, 0x00 },
	{ OV9234_REG_MODE_SELECT,    OV9234_MODE_STREAMING },
};

static const struct ov9234_mode ov9234_supported_modes[] = {
	{
		/* 720p 30fps Full Resolution Readout[cite: 1] */
		.width = 1280,
		.height = 720,
		.hts = 1478,
		.vts_def = 802,
		.vts_min = 720 + 20,
		.max_fps = 30,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 1280,
			.height = 720,
		},
		.binning_enabled = false,
		.reg_list = ov9234_mode_720p_30fps_regs,
		.num_regs = ARRAY_SIZE(ov9234_mode_720p_30fps_regs),
	},
	{
		/* VGA 45fps Center Crop[cite: 1] */
		.width = 640,
		.height = 480,
		.hts = 1478,
		.vts_def = 541,
		.vts_min = 480 + 20,
		.max_fps = 45,
		.crop = {
			.left = 328,
			.top = 128,
			.width = 640,
			.height = 480,
		},
		.binning_enabled = false,
		.reg_list = ov9234_mode_vga_45fps_regs,
		.num_regs = ARRAY_SIZE(ov9234_mode_vga_45fps_regs),
	},
	{
		/* 360p 60fps 2x2 B&W Binning[cite: 1] */
		.width = 640,
		.height = 360,
		.hts = 1478,
		.vts_def = 406,
		.vts_min = 360 + 20,
		.max_fps = 60,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 1280,
			.height = 720,
		},
		.binning_enabled = true,
		.reg_list = ov9234_mode_360p_60fps_regs,
		.num_regs = ARRAY_SIZE(ov9234_mode_360p_60fps_regs),
	},
};

static const u32 ov9234_mbus_codes[] = {
	MEDIA_BUS_FMT_Y10_1X10, /* 10-bit monochrome / IR (0x3654[5] = 0)[cite: 1] */
	MEDIA_BUS_FMT_Y8_1X8,   /* 8-bit monochrome / IR (0x3654[5] = 1)[cite: 1] */
};

static const s64 ov9234_link_freq_menu[] = {
	OV9234_LINK_FREQ_360MBPS,
};

static const char * const ov9234_test_pattern_menu[] = {
	"Disabled",
	"Standard Bar (Type 1)",
	"Top-Bottom Darker Bar (Type 2)",
	"Right-Left Darker Bar (Type 3)",
	"Bottom-Top Darker Bar (Type 4)",
};

struct ov9234 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xvclk;
	u32 xvclk_freq;

	/*
	 * Power supplies: AVDD (2.8V) and DOVDD (1.8V) may ramp in any order,
	 * whereas DVDD (1.2V) must ramp up AFTER AVDD/DOVDD and ramp down
	 * BEFORE AVDD/DOVDD[cite: 1].
	 */
	struct regulator_bulk_data io_analog_supplies[2];
	struct regulator *dvdd_supply;
	struct gpio_desc *xshutdn_gpio;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure_ctrl;
	struct v4l2_ctrl *again_ctrl;
	struct v4l2_ctrl *hflip_ctrl;
	struct v4l2_ctrl *vflip_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *pixel_rate_ctrl;
	struct v4l2_ctrl *link_freq_ctrl;

	const struct ov9234_mode *cur_mode;
	bool streaming;
};

static inline struct ov9234 *to_ov9234(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9234, sd);
}

static const struct regmap_config ov9234_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

/*
 * Register Access Helpers (Big-Endian 16-bit address, 8-bit data)[cite: 1]
 */
static int ov9234_read_reg(struct ov9234 *ov9234, u16 reg, u8 *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(ov9234->regmap, reg, &reg_val);
	if (ret) {
		dev_err(ov9234->dev, "Failed to read reg 0x%04x: %d\n", reg, ret);
		return ret;
	}

	*val = (u8)reg_val;
	return 0;
}

static int ov9234_write_reg(struct ov9234 *ov9234, u16 reg, u8 val)
{
	int ret;

	ret = regmap_write(ov9234->regmap, reg, val);
	if (ret)
		dev_err(ov9234->dev, "Failed to write reg 0x%04x=0x%02x: %d\n",
			reg, val, ret);

	return ret;
}

static int ov9234_write_reg16(struct ov9234 *ov9234, u16 reg_high, u16 val)
{
	u8 buf[2] = { (u8)(val >> 8), (u8)(val & 0xff) };
	int ret;

	ret = regmap_bulk_write(ov9234->regmap, reg_high, buf, sizeof(buf));
	if (ret)
		dev_err(ov9234->dev, "Failed to write 16-bit reg 0x%04x=0x%04x: %d\n",
			reg_high, val, ret);

	return ret;
}

static int ov9234_write_reg24(struct ov9234 *ov9234, u16 reg_high, u32 val)
{
	u8 buf[3] = {
		(u8)((val >> 16) & 0xff),
		(u8)((val >> 8) & 0xff),
		(u8)(val & 0xff),
	};
	int ret;

	ret = regmap_bulk_write(ov9234->regmap, reg_high, buf, sizeof(buf));
	if (ret)
		dev_err(ov9234->dev, "Failed to write 24-bit reg 0x%04x=0x%06x: %d\n",
			reg_high, val, ret);

	return ret;
}

static int ov9234_write_reg_array(struct ov9234 *ov9234,
				  const struct ov9234_reg *regs,
				  unsigned int num_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_regs; i++) {
		ret = ov9234_write_reg(ov9234, regs[i].address, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Group Write Atomic Update Helpers (Register 0x3208)[cite: 1]
 * Buffers up to 16 bytes in SRAM and latches them at the frame boundary.
 */
static int ov9234_group_hold_start(struct ov9234 *ov9234)
{
	if (!ov9234->streaming)
		return 0;

	return ov9234_write_reg(ov9234, OV9234_REG_GROUP_ACCESS,
				OV9234_GROUP_HOLD_START_G0);
}

static int ov9234_group_hold_launch(struct ov9234 *ov9234)
{
	int ret;

	if (!ov9234->streaming)
		return 0;

	ret = ov9234_write_reg(ov9234, OV9234_REG_GROUP_ACCESS,
			       OV9234_GROUP_HOLD_END_G0);
	if (ret)
		return ret;

	return ov9234_write_reg(ov9234, OV9234_REG_GROUP_ACCESS,
				OV9234_GROUP_LAUNCH_G0);
}

/*
 * Control Primitives[cite: 1]
 */
static int ov9234_set_exposure(struct ov9234 *ov9234, u32 exposure_lines)
{
	u32 reg_20bit;
	int ret;

	/*
	 * Exposure is encoded in Q16.4 fixed-point format across 0x3500..0x3502:
	 *   reg_20bit = exposure_lines << 4
	 *   0x3500 = (reg_20bit >> 16) & 0x0F
	 *   0x3501 = (reg_20bit >> 8) & 0xFF
	 *   0x3502 = reg_20bit & 0xFF[cite: 1]
	 */
	reg_20bit = (exposure_lines << OV9234_EXPO_Q16_4_SHIFT) & 0x0fffff;

	ret = ov9234_group_hold_start(ov9234);
	if (ret)
		return ret;

	ret = ov9234_write_reg24(ov9234, OV9234_REG_LONG_EXPO_HIGH, reg_20bit);
	if (ret)
		return ret;

	return ov9234_group_hold_launch(ov9234);
}

static int ov9234_set_analog_gain(struct ov9234 *ov9234, u32 gain_code)
{
	int ret;

	/*
	 * Analog gain is encoded in Q6.4 fixed-point format across 0x350A..0x350B:
	 *   0x010 (16) = 1.0x gain, 0x0F8 (248) = 15.5x gain
	 *   0x350A = (gain_code >> 8) & 0x03
	 *   0x350B = gain_code & 0xFF[cite: 1]
	 */
	gain_code = clamp_val(gain_code, OV9234_ANA_GAIN_MIN, OV9234_ANA_GAIN_MAX);

	ret = ov9234_group_hold_start(ov9234);
	if (ret)
		return ret;

	ret = ov9234_write_reg16(ov9234, OV9234_REG_LONG_GAIN_HIGH,
				 (u16)(gain_code & 0x03ff));
	if (ret)
		return ret;

	return ov9234_group_hold_launch(ov9234);
}

static int ov9234_set_orientation(struct ov9234 *ov9234, bool hflip, bool vflip)
{
	u8 val = 0;

	/*
	 * Register 0x3820 (FORMAT0_ORIENTATION):
	 *   Bit[4]: BLC_Flip (MUST match Bit[2] vflip: 0=OFF, 1=ON)
	 *   Bit[3]: Mirror (Horizontal mirror: 0=OFF, 1=ON)
	 *   Bit[2]: vflip (Vertical flip: 0=OFF, 1=ON)
	 *   Bit[1]: hbin (Preserve based on current mode)
	 *   Bit[0]: vbin (Preserve based on current mode)[cite: 1]
	 */
	if (ov9234->cur_mode->binning_enabled)
		val |= OV9234_ORIENT_HBIN_EN | OV9234_ORIENT_VBIN_EN;

	if (hflip)
		val |= OV9234_ORIENT_HMIRROR_EN;

	if (vflip)
		val |= OV9234_ORIENT_VFLIP_EN | OV9234_ORIENT_BLC_FLIP_EN;

	return ov9234_write_reg(ov9234, OV9234_REG_FORMAT0_ORIENTATION, val);
}

static int ov9234_set_vts(struct ov9234 *ov9234, u32 vts)
{
	int ret;

	vts = clamp_val(vts, ov9234->cur_mode->vts_min, OV9234_VTS_MAX);

	ret = ov9234_group_hold_start(ov9234);
	if (ret)
		return ret;

	ret = ov9234_write_reg16(ov9234, OV9234_REG_VTS_H, (u16)vts);
	if (ret)
		return ret;

	return ov9234_group_hold_launch(ov9234);
}

static int ov9234_set_test_pattern(struct ov9234 *ov9234, u32 pattern_idx)
{
	u8 val = 0;

	/*
	 * Register 0x5080 (PRE_ISP_CTRL0):
	 *   Bit[7]: Test pattern enable (0: Disable, 1: Enable)
	 *   Bits[3:2]: Style (00: Type 1, 01: Type 2, 10: Type 3, 11: Type 4)[cite: 1]
	 */
	if (pattern_idx > 0) {
		val = OV9234_TEST_PATTERN_EN |
		      FIELD_PREP(OV9234_TEST_PATTERN_STYLE_MASK, pattern_idx - 1);
	}

	return ov9234_write_reg(ov9234, OV9234_REG_PRE_ISP_CTRL0, val);
}

static int ov9234_configure_mbus_format(struct ov9234 *ov9234, u32 code)
{
	u8 fmt_ctrl = OV9234_FMT_CTRL_DEFAULT;
	u8 mipi_dt = OV9234_MIPI_DT_RAW10;
	int ret;

	/*
	 * Register 0x3654[5] selects RAW10 (0) vs RAW8 (1).
	 * Register 0x4814[5:0] configures the MIPI CSI-2 Data Type (0x2B for RAW10,
	 * 0x2A for RAW8)[cite: 1].
	 */
	if (code == MEDIA_BUS_FMT_Y8_1X8) {
		fmt_ctrl |= OV9234_FMT_CTRL_RAW8_SEL;
		mipi_dt = OV9234_MIPI_DT_RAW8;
	}

	ret = ov9234_write_reg(ov9234, OV9234_REG_FMT_CTRL, fmt_ctrl);
	if (ret)
		return ret;

	return ov9234_write_reg(ov9234, OV9234_REG_MIPI_CTRL14, mipi_dt);
}

static int ov9234_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9234 *ov9234 =
		container_of(ctrl->handler, struct ov9234, ctrl_handler);
	u32 vts, max_expo;
	int ret = 0;

	/* Update exposure upper bound dynamically whenever VBLANK changes[cite: 1] */
	if (ctrl->id == V4L2_CID_VBLANK) {
		vts = ov9234->cur_mode->height + ctrl->val;
		max_expo = vts - OV9234_EXPO_MARGIN_LINES;
		__v4l2_ctrl_modify_range(ov9234->exposure_ctrl,
					 OV9234_EXPO_MIN_LINES, max_expo, 1,
					 min_t(u32, OV9234_EXPO_DEFAULT_LINES,
					       max_expo));
	}

	/* Apply register writes only if the sensor is powered on */
	if (!pm_runtime_get_if_in_use(ov9234->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov9234_set_exposure(ov9234, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov9234_set_analog_gain(ov9234, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = ov9234_set_orientation(ov9234, ov9234->hflip_ctrl->val,
					     ov9234->vflip_ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ov9234_set_vts(ov9234,
				     ov9234->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov9234_set_test_pattern(ov9234, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ov9234->dev);
	return ret;
}

static const struct v4l2_ctrl_ops ov9234_ctrl_ops = {
	.s_ctrl = ov9234_s_ctrl,
};

/*
 * Power-Up & Power-Down Lifecycle Sequences (Section 2)[cite: 1]
 */
static int ov9234_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov9234 *ov9234 = to_ov9234(sd);
	u32 settle_us;
	int ret;

	/*
	 * Step 1: Assert XSHUTDN LOW (logical 1 when configured GPIO_ACTIVE_LOW)
	 * to hold the sensor in hardware standby[cite: 1].
	 */
	gpiod_set_value_cansleep(ov9234->xshutdn_gpio, 1);

	/*
	 * Step 2: Enable AVDD (2.8V) and DOVDD (1.8V) in any order (t0/t1 >= 0 ns)[cite: 1].
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(ov9234->io_analog_supplies),
				    ov9234->io_analog_supplies);
	if (ret) {
		dev_err(dev, "Failed to enable AVDD/DOVDD supplies: %d\n", ret);
		return ret;
	}

	/*
	 * Step 3: Enable DVDD (1.2V) AFTER AVDD and DOVDD have risen (t8 >= 0 ns)[cite: 1].
	 */
	ret = regulator_enable(ov9234->dvdd_supply);
	if (ret) {
		dev_err(dev, "Failed to enable DVDD supply: %d\n", ret);
		goto err_disable_io_analog;
	}

	/*
	 * Step 4: Enable XVCLK and deassert XSHUTDN HIGH (logical 0) (t2 >= 0 ns)[cite: 1].
	 */
	ret = clk_prepare_enable(ov9234->xvclk);
	if (ret) {
		dev_err(dev, "Failed to enable XVCLK: %d\n", ret);
		goto err_disable_dvdd;
	}

	gpiod_set_value_cansleep(ov9234->xshutdn_gpio, 0);

	/*
	 * Step 5: Wait at least t3/t4 (8192 XVCLK cycles; ~342 us at 24 MHz,
	 * recommended driver delay 1.0 ms) before the first SCCB transaction[cite: 1].
	 */
	settle_us = DIV_ROUND_UP_ULL((u64)OV9234_PWRUP_MIN_XVCLK_CYCLES * USEC_PER_SEC,
				     ov9234->xvclk_freq);
	settle_us = max_t(u32, settle_us, OV9234_PWRUP_MIN_DELAY_US);
	fsleep(settle_us);

	return 0;

err_disable_dvdd:
	regulator_disable(ov9234->dvdd_supply);
err_disable_io_analog:
	regulator_bulk_disable(ARRAY_SIZE(ov9234->io_analog_supplies),
			       ov9234->io_analog_supplies);
	return ret;
}

static int ov9234_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov9234 *ov9234 = to_ov9234(sd);
	u32 wait_us;

	/*
	 * Step 1: Ensure software standby (0x0100 = 0x00)[cite: 1].
	 * Step 2: Maintain XVCLK active for at least 512 XVCLK cycles (t1/t2)
	 * after the last SCCB transaction before asserting XSHUTDN LOW[cite: 1].
	 */
	wait_us = DIV_ROUND_UP_ULL((u64)OV9234_PWRDN_MIN_XVCLK_CYCLES * USEC_PER_SEC,
				   ov9234->xvclk_freq);
	fsleep(wait_us);

	/*
	 * Step 3: Assert XSHUTDN LOW (logical 1) and gate XVCLK[cite: 1].
	 */
	gpiod_set_value_cansleep(ov9234->xshutdn_gpio, 1);
	clk_disable_unprepare(ov9234->xvclk);

	/*
	 * Step 4: Disable DVDD (1.2V) first (t8 >= 0 ns)[cite: 1].
	 */
	regulator_disable(ov9234->dvdd_supply);

	/*
	 * Step 5: Disable AVDD (2.8V) and DOVDD (1.8V) after DVDD falls[cite: 1].
	 */
	regulator_bulk_disable(ARRAY_SIZE(ov9234->io_analog_supplies),
			       ov9234->io_analog_supplies);

	return 0;
}

/*
 * V4L2 Subdevice Pad & Streaming Operations (Linux 7.x State API)
 */
static int ov9234_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct ov9234 *ov9234 = to_ov9234(sd);
	const struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (streams_mask != BIT_ULL(0))
		return -EINVAL;

	ret = pm_runtime_resume_and_get(ov9234->dev);
	if (ret < 0)
		return ret;

	/* Issue software reset and wait 1 ms settling time[cite: 1] */
	ret = ov9234_write_reg(ov9234, OV9234_REG_SOFTWARE_RESET,
			       OV9234_SW_RESET_TRIG);
	if (ret)
		goto err_pm_put;
	fsleep(OV9234_SW_RESET_DELAY_US);

	/* Program common PLL, BLC, DPC, and MIPI registers[cite: 1] */
	ret = ov9234_write_reg_array(ov9234, ov9234_common_init_regs,
				     ARRAY_SIZE(ov9234_common_init_regs));
	if (ret)
		goto err_pm_put;

	/* Program mode-specific windowing, timing, and binning registers[cite: 1] */
	ret = ov9234_write_reg_array(ov9234, ov9234->cur_mode->reg_list,
				     ov9234->cur_mode->num_regs);
	if (ret)
		goto err_pm_put;

	/* Configure RAW10 (Y10) vs RAW8 (Y8) output format[cite: 1] */
	fmt = v4l2_subdev_state_get_format(state, 0);
	ret = ov9234_configure_mbus_format(ov9234, fmt->code);
	if (ret)
		goto err_pm_put;

	/* Apply current V4L2 control values (exposure, gain, flip, blanking) */
	ret = __v4l2_ctrl_handler_setup(&ov9234->ctrl_handler);
	if (ret)
		goto err_pm_put;

	/* Wake from deep software standby and enter streaming mode (0x0100=0x01)[cite: 1] */
	ret = ov9234_write_reg_array(ov9234, ov9234_deep_sleep_off_regs,
				     ARRAY_SIZE(ov9234_deep_sleep_off_regs));
	if (ret)
		goto err_pm_put;

	ov9234->streaming = true;
	return 0;

err_pm_put:
	pm_runtime_put(ov9234->dev);
	return ret;
}

static int ov9234_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct ov9234 *ov9234 = to_ov9234(sd);
	int ret;

	if (streams_mask != BIT_ULL(0))
		return -EINVAL;

	ov9234->streaming = false;

	/* Enter deep software standby sequence (0x0100 = 0x00 + analog/PLL gate)[cite: 1] */
	ret = ov9234_write_reg_array(ov9234, ov9234_deep_sleep_on_regs,
				     ARRAY_SIZE(ov9234_deep_sleep_on_regs));
	if (ret)
		dev_err(ov9234->dev, "Failed to enter software standby: %d\n", ret);

	pm_runtime_put(ov9234->dev);
	return ret;
}

static int ov9234_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov9234_mbus_codes))
		return -EINVAL;

	code->code = ov9234_mbus_codes[code->index];
	return 0;
}

static int ov9234_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int i;
	bool valid_code = false;

	for (i = 0; i < ARRAY_SIZE(ov9234_mbus_codes); i++) {
		if (fse->code == ov9234_mbus_codes[i]) {
			valid_code = true;
			break;
		}
	}

	if (!valid_code || fse->index >= ARRAY_SIZE(ov9234_supported_modes))
		return -EINVAL;

	fse->min_width = ov9234_supported_modes[fse->index].width;
	fse->max_width = ov9234_supported_modes[fse->index].width;
	fse->min_height = ov9234_supported_modes[fse->index].height;
	fse->max_height = ov9234_supported_modes[fse->index].height;

	return 0;
}

static u32 ov9234_validate_mbus_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ov9234_mbus_codes); i++) {
		if (code == ov9234_mbus_codes[i])
			return code;
	}

	return MEDIA_BUS_FMT_Y10_1X10;
}

static void ov9234_fill_pad_format(const struct ov9234_mode *mode,
				   u32 mbus_code,
				   struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = ov9234_validate_mbus_code(mbus_code);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov9234_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ov9234 *ov9234 = to_ov9234(sd);
	const struct ov9234_mode *mode;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	s32 vblank_def, vblank_min, hblank;

	if (ov9234->streaming)
		return -EBUSY;

	mode = v4l2_find_nearest_size(ov9234_supported_modes,
				      ARRAY_SIZE(ov9234_supported_modes),
				      width, height,
				      format->format.width,
				      format->format.height);

	ov9234_fill_pad_format(mode, format->format.code, &format->format);

	fmt = v4l2_subdev_state_get_format(state, 0);
	*fmt = format->format;

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = mode->crop;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ov9234->cur_mode = mode;

		vblank_def = mode->vts_def - mode->height;
		vblank_min = mode->vts_min - mode->height;
		__v4l2_ctrl_modify_range(ov9234->vblank_ctrl, vblank_min,
					 OV9234_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(ov9234->vblank_ctrl, vblank_def);

		hblank = mode->hts - mode->width;
		__v4l2_ctrl_modify_range(ov9234->hblank_ctrl, hblank, hblank,
					 1, hblank);
	}

	return 0;
}

static int ov9234_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV9234_NATIVE_WIDTH;
		sel->r.height = OV9234_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 8;
		sel->r.top = 8;
		sel->r.width = OV9234_ACTIVE_WIDTH;
		sel->r.height = OV9234_ACTIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ov9234_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct ov9234_mode *def_mode = &ov9234_supported_modes[0];
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_state_get_format(state, 0);
	ov9234_fill_pad_format(def_mode, MEDIA_BUS_FMT_Y10_1X10, fmt);

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = def_mode->crop;

	return 0;
}

static const struct v4l2_subdev_video_ops ov9234_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov9234_pad_ops = {
	.enum_mbus_code = ov9234_enum_mbus_code,
	.enum_frame_size = ov9234_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov9234_set_fmt,
	.get_selection = ov9234_get_selection,
	.enable_streams = ov9234_enable_streams,
	.disable_streams = ov9234_disable_streams,
};

static const struct v4l2_subdev_ops ov9234_subdev_ops = {
	.video = &ov9234_video_ops,
	.pad = &ov9234_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov9234_internal_ops = {
	.init_state = ov9234_init_state,
};

/*
 * Chip Identification (Registers 0x300A, 0x300B, 0x302A)[cite: 1]
 */
static int ov9234_detect_chip(struct ov9234 *ov9234)
{
	u8 id_high = 0, id_low = 0, rev = 0;
	u16 chip_id;
	int ret;

	ret = ov9234_read_reg(ov9234, OV9234_REG_CHIP_ID_HIGH, &id_high);
	if (ret)
		return ret;

	ret = ov9234_read_reg(ov9234, OV9234_REG_CHIP_ID_LOW, &id_low);
	if (ret)
		return ret;

	chip_id = ((u16)id_high << 8) | id_low;
	if (chip_id != OV9234_CHIP_ID) {
		dev_err(ov9234->dev,
			"Unexpected Chip ID 0x%04x (expected 0x%04x)\n",
			chip_id, OV9234_CHIP_ID);
		return -ENODEV;
	}

	ret = ov9234_read_reg(ov9234, OV9234_REG_CHIP_REVISION, &rev);
	if (ret)
		return ret;

	dev_info(ov9234->dev,
		 "Detected OmniVision OV9234 (Chip ID 0x%04x, Rev 0x%02x)\n",
		 chip_id, rev);

	return 0;
}

static int ov9234_init_controls(struct ov9234 *ov9234)
{
	const struct ov9234_mode *mode = ov9234->cur_mode;
	struct v4l2_ctrl_handler *hdl = &ov9234->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	u32 vblank_def, vblank_min, hblank, max_expo;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 10);
	if (ret)
		return ret;

	/* Link Frequency (180 MHz DDR = 360 Mbps MIPI CSI-2)[cite: 1] */
	ov9234->link_freq_ctrl =
		v4l2_ctrl_new_int_menu(hdl, &ov9234_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(ov9234_link_freq_menu) - 1,
				       0, ov9234_link_freq_menu);
	if (ov9234->link_freq_ctrl)
		ov9234->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Pixel Rate (36 MHz system timing domain)[cite: 1] */
	ov9234->pixel_rate_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_PIXEL_RATE,
				  OV9234_SYS_PIXEL_RATE_HZ,
				  OV9234_SYS_PIXEL_RATE_HZ, 1,
				  OV9234_SYS_PIXEL_RATE_HZ);

	/* Horizontal & Vertical Blanking[cite: 1] */
	vblank_def = mode->vts_def - mode->height;
	vblank_min = mode->vts_min - mode->height;
	ov9234->vblank_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_min, OV9234_VTS_MAX - mode->height,
				  1, vblank_def);

	hblank = mode->hts - mode->width;
	ov9234->hblank_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_HBLANK,
				  hblank, hblank, 1, hblank);
	if (ov9234->hblank_ctrl)
		ov9234->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Manual Exposure in lines (Min = 4, Max = VTS - 4, Q16.4 in hardware)[cite: 1] */
	max_expo = mode->vts_def - OV9234_EXPO_MARGIN_LINES;
	ov9234->exposure_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_EXPOSURE,
				  OV9234_EXPO_MIN_LINES, max_expo, 1,
				  OV9234_EXPO_DEFAULT_LINES);

	/* Global Analog Gain in Q6.4 units (16 = 1.0x to 248 = 15.5x)[cite: 1] */
	ov9234->again_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
				  OV9234_ANA_GAIN_MIN, OV9234_ANA_GAIN_MAX,
				  OV9234_ANA_GAIN_STEP,
				  OV9234_ANA_GAIN_DEFAULT);

	/* Horizontal Mirror & Vertical Flip (toggles BLC_Flip with V-Flip)[cite: 1] */
	ov9234->hflip_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_HFLIP,
				  0, 1, 1, 0);
	ov9234->vflip_ctrl =
		v4l2_ctrl_new_std(hdl, &ov9234_ctrl_ops, V4L2_CID_VFLIP,
				  0, 1, 1, 0);

	/* Test Pattern Generator (0x5080)[cite: 1] */
	v4l2_ctrl_new_std_menu_items(hdl, &ov9234_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov9234_test_pattern_menu) - 1,
				     0, 0, ov9234_test_pattern_menu);

	/* Parse optional orientation/rotation properties from Device Tree */
	ret = v4l2_fwnode_device_parse(ov9234->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &ov9234_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(ov9234->dev, "Control handler init failed: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ov9234->sd.ctrl_handler = hdl;
	return 0;
}

static int ov9234_parse_fwnode(struct ov9234 *ov9234)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned int i;
	bool freq_matched = false;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(ov9234->dev), NULL);
	if (!endpoint) {
		dev_warn(ov9234->dev, "CSI-2 endpoint node not found, bypassing check for ACPI\n");
		return 0; /* Bypass the strict check to allow the I2C probe to continue */
	}
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(ov9234->dev, ret,
				     "Failed to parse CSI-2 endpoint\n");

	/* OV9234 uses a 1-lane MIPI CSI-2 D-PHY interface[cite: 1] */
	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 1) {
		ret = dev_err_probe(ov9234->dev, -EINVAL,
				    "Unsupported data lanes: %u (expected 1)\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out_free_bus;
	}

	if (bus_cfg.nr_of_link_frequencies > 0) {
		for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
			if (bus_cfg.link_frequencies[i] ==
			    OV9234_LINK_FREQ_360MBPS) {
				freq_matched = true;
				break;
			}
		}
		if (!freq_matched) {
			ret = dev_err_probe(ov9234->dev, -EINVAL,
					    "360 Mbps (180 MHz) link-frequencies required\n");
			goto out_free_bus;
		}
	}

out_free_bus:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int ov9234_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov9234 *ov9234;
	int ret;

	ov9234 = devm_kzalloc(dev, sizeof(*ov9234), GFP_KERNEL);
	if (!ov9234)
		return -ENOMEM;

	ov9234->dev = dev;
	ov9234->cur_mode = &ov9234_supported_modes[0];

	ret = ov9234_parse_fwnode(ov9234);
	if (ret)
		return ret;

	ov9234->regmap = devm_regmap_init_i2c(client, &ov9234_regmap_config);
	if (IS_ERR(ov9234->regmap))
		return dev_err_probe(dev, PTR_ERR(ov9234->regmap),
				     "Failed to initialize I2C regmap\n");

	/* Acquire XVCLK (24 MHz typical; 6 to 27 MHz supported)[cite: 1] */
	ov9234->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov9234->xvclk))
		return dev_err_probe(dev, PTR_ERR(ov9234->xvclk),
				     "Failed to get xvclk\n");

	ov9234->xvclk_freq = clk_get_rate(ov9234->xvclk);
	if (ov9234->xvclk_freq < OV9234_XVCLK_MIN_HZ || 
	    ov9234->xvclk_freq > OV9234_XVCLK_MAX_HZ)
		return dev_err_probe(dev, -EINVAL,
				     "Unsupported xvclk rate %u Hz (expected 6-27 MHz)\n",
				     ov9234->xvclk_freq);
	
	if (ov9234->xvclk_freq != OV9234_XVCLK_TYP_HZ)
		dev_warn(dev, "XVCLK is %u Hz, but PLL registers expect 24 MHz. Streaming timing may be incorrect.\n", 
			 ov9234->xvclk_freq);

	/* Acquire XSHUTDN hardware reset/powerdown GPIO (active low)[cite: 1] */
	ov9234->xshutdn_gpio = devm_gpiod_get_optional(dev, "reset",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(ov9234->xshutdn_gpio))
		return dev_err_probe(dev, PTR_ERR(ov9234->xshutdn_gpio),
				     "Failed to get reset (XSHUTDN) GPIO\n");

	/* Acquire AVDD (2.8V), DOVDD (1.8V), and DVDD (1.2V) regulators[cite: 1] */
	ov9234->io_analog_supplies[0].supply = "avdd";
	ov9234->io_analog_supplies[1].supply = "dovdd";
	ret = devm_regulator_bulk_get(dev,
				      ARRAY_SIZE(ov9234->io_analog_supplies),
				      ov9234->io_analog_supplies);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get AVDD/DOVDD regulators\n");

	ov9234->dvdd_supply = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(ov9234->dvdd_supply))
		return dev_err_probe(dev, PTR_ERR(ov9234->dvdd_supply),
				     "Failed to get DVDD regulator\n");

	/* Initialize V4L2 subdevice and Media Entity */
	v4l2_i2c_subdev_init(&ov9234->sd, client, &ov9234_subdev_ops);
	ov9234->sd.internal_ops = &ov9234_internal_ops;
	ov9234->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	ov9234->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ov9234->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov9234->sd.entity, 1, &ov9234->pad);
	if (ret)
		return ret;

	/* Power on hardware to verify Chip ID over SCCB[cite: 1] */
	ret = ov9234_power_on(dev);
	if (ret)
		goto err_clean_entity;

	ret = ov9234_detect_chip(ov9234);
	if (ret)
		goto err_power_off;

	ret = ov9234_init_controls(ov9234);
	if (ret)
		goto err_power_off;

	ov9234->sd.state_lock = ov9234->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&ov9234->sd);
	if (ret)
		goto err_free_ctrls;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&ov9234->sd);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register V4L2 async subdev\n");
		goto err_pm_disable;
	}

	pm_runtime_idle(dev);
	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&ov9234->sd);
err_free_ctrls:
	v4l2_ctrl_handler_free(&ov9234->ctrl_handler);
err_power_off:
	ov9234_power_off(dev);
err_clean_entity:
	media_entity_cleanup(&ov9234->sd.entity);
	return ret;
}

static void ov9234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9234 *ov9234 = to_ov9234(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov9234->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ov9234_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov9234_pm_ops, ov9234_power_off,
				 ov9234_power_on, NULL);

static const struct of_device_id ov9234_of_match[] = {
	{ .compatible = "omnivision,ov9234" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov9234_of_match);

static const struct i2c_device_id ov9234_i2c_id[] = {
	{ "ov9234", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ov9234_i2c_id);

static const struct acpi_device_id ov9234_acpi_match[] = {
	{ "OVTI9234", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ov9234_acpi_match);

static struct i2c_driver ov9234_i2c_driver = {
	.driver = {
		.name = OV9234_DRIVER_NAME,
		.pm = pm_ptr(&ov9234_pm_ops),
		.of_match_table = ov9234_of_match,
		.acpi_match_table = ACPI_PTR(ov9234_acpi_match),
	},
	.probe = ov9234_probe,
	.remove = ov9234_remove,
	.id_table = ov9234_i2c_id,
};
module_i2c_driver(ov9234_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV9234 Monochrome/IR CMOS Image Sensor Driver");
MODULE_AUTHOR("Embedded Camera Subsystem Engineering");
MODULE_LICENSE("GPL");
