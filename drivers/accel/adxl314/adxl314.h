/***************************************************************************//**
 *   @file   adxl314.h
 *   @brief  Header file of ADXL314 Driver.
 *   @author GMois (george.mois@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __ADXL314_H__
#define __ADXL314_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* SPI commands */
#define ADXL314_SPI_READ          (0x80)
#define ADXL314_SPI_WRITE         (0x00)
#define ADXL314_MULTIBIT          NO_OS_BIT(6)

/* Register address */
#define ADXL314_ADDR(x)			  ((x) & 0x3F)

/* Transfer length */
#define GET_ADXL314_TRANSF_LEN(x) (((x) >>  8) & 0x0000FF)
#define SET_ADXL314_TRANSF_LEN(x) (((x) <<  8) & 0x00FF00)

/* Register reset values*/
#define GET_ADXL314_RESET_VAL(x)  (((x) >> 16) & 0x0000FF)
#define SET_ADXL314_RESET_VAL(x)  (((x) << 16) & 0xFF0000)

/* ADXL314 Register Map */
#define ADXL314_DEVID_AD 	 			0x00
#define ADXL314_REG_OFS_AXIS(index)		(0x1E + (index))
#define ADXL314_REG_THRESH_ACT			0x24
#define ADXL314_REG_THRESH_INACT		0x25
#define ADXL314_REG_TIME_INACT			0x26
#define ADXL314_REG_ACT_INACT_CTL		0x27
#define ADXL314_REG_BW_RATE				0x2C
#define ADXL314_REG_POWER_CTL			0x2D
#define ADXL314_REG_INT_EN				0x2E
#define ADXL314_REG_INT_MAP				0x2F
#define ADXL314_REG_INT_SRC				0x30
#define ADXL314_REG_DATA_FORMAT			0x31
#define ADXL314_REG_DATA_AXIS(index)	(0x32 + ((index) * 2))
#define ADXL314_REG_FIFO_CTL			0x38
#define ADXL314_REG_FIFO_STATUS			0x39

/* ADXL314_DEVID_AD value */
#define ADXL314_DEVID					0xE5

/* Maximum number of FIFO asmples */
#define ADXL314_MAX_FIFO_SAMPLES_VAL  0x60

/* Device bandwidth settings, ADXL314_REG_BW_RATE definitions */
#define ADXL314_RATE_MSK				NO_OS_GENMASK(3, 0)
#define ADXL314_LOW_POWER_OP			NO_OS_BIT(4)

/* ADXL314_REG_POWER_CTL definitions */
#define ADXL314_POWER_CTL_LINK          NO_OS_BIT(5)
#define ADXL314_POWER_CTL_AUTO_SLEEP    NO_OS_BIT(4)
#define ADXL314_POWER_CTL_MEASURE  		NO_OS_BIT(3)
#define ADXL314_POWER_CTL_SLEEP			NO_OS_BIT(2)
#define ADXL314_POWER_CTL_WAKEUP(x)	    (((x) & 0x3) << 0)

/* ADXL314_REG_INT_EN definitions */
#define ADXL314_REG_INT_EN_DATA_READY	NO_OS_BIT(7)
#define ADXL314_REG_INT_EN_ACTIVITY		NO_OS_BIT(4)
#define ADXL314_REG_INT_EN_INACTIVITY	NO_OS_BIT(3)
#define ADXL314_REG_INT_EN_WATERMARK	NO_OS_BIT(1)
#define ADXL314_REG_INT_EN_OVERRUN		NO_OS_BIT(0)

/* ADXL314_REG_INT_MAP definitions */
#define ADXL314_REG_INT_MAP_DR_INT2		NO_OS_BIT(7)
#define ADXL314_REG_INT_MAP_ACT_INT2	NO_OS_BIT(4)
#define ADXL314_REG_INT_MAP_INACT_INT2	NO_OS_BIT(3)
#define ADXL314_REG_INT_MAP_WM_INT2		NO_OS_BIT(1)
#define ADXL314_REG_INT_MAP_OVER_INT2	NO_OS_BIT(0)

/* ADXL314_REG_INT_SRC definitions */
#define ADXL314_REG_INT_SRC_DATA_READY	NO_OS_BIT(7)
#define ADXL314_REG_INT_SRC_ACTIVITY	NO_OS_BIT(4)
#define ADXL314_REG_INT_SRC_INACTIVITY	NO_OS_BIT(3)
#define ADXL314_REG_INT_SRC_WATERMARK	NO_OS_BIT(1)
#define ADXL314_REG_INT_SRC_OVERRUN		NO_OS_BIT(0)

/* ADXL314_REG_INT_SRC definitions */
#define ADXL314_REG_INT_SRC_DATA_READY	NO_OS_BIT(7)
#define ADXL314_REG_INT_SRC_ACTIVITY	NO_OS_BIT(4)
#define ADXL314_REG_INT_SRC_INACTIVITY	NO_OS_BIT(3)
#define ADXL314_REG_INT_SRC_WATERMARK	NO_OS_BIT(1)
#define ADXL314_REG_INT_SRC_OVERRUN		NO_OS_BIT(0)

/* ADXL314_REG_DATA_FORMAT definitions */
#define ADXL314_REG_DATA_FORMAT_SELF_TEST		NO_OS_BIT(7)
#define ADXL314_REG_DATA_FORMAT_SPI_3WIRE		NO_OS_BIT(6)
#define ADXL314_REG_DATA_FORMAT_INT_INV			NO_OS_BIT(5)
#define ADXL314_REG_DATA_FORMAT_JUSTIFY_BIT		NO_OS_BIT(2)

/* ADXL314_REG_FIFO_CTL definitions */
#define ADXL314_REG_FIFO_CTL_MODE_MSK			NO_OS_GENMASK(7, 6)
#define ADXL314_REG_FIFO_CTL_TRIGGER_BIT		NO_OS_BIT(5)
#define ADXL314_REG_FIFO_CTL_SAMPLES_MSK		NO_OS_GENMASK(4, 0)

/* ADXL314_REG_FIFO_STATUS definitions */
#define ADXL314_REG_FIFO_FIFO_TRIGGER_BIT		NO_OS_BIT(7)
#define ADXL314_REG_FIFO_STS_ENTRIES_MSK		NO_OS_GENMASK(5, 0)

/*
 * At +/-200g with 13-bit resolution, scale factor is given in datasheet as
 * 50mg/LSB = 0.0500000 * 9.80665 = 0.4903325000 m/s^2.
 */
#define ADXL314_ACC_SCALE_FACTOR_MUL  			((int64_t)4903325ULL)
#define ADXL314_ACC_SCALE_FACTOR_DIV  			((int32_t)10000000)

#define ADXL314_NEG_ACC_MSK        				NO_OS_GENMASK(15, 13)

/* Offset added to the ODR enum for obtaining corresponding register setting */
#define ADXL314_ODR_OFFSET_VAL					6

/* Number of registers read for all axes */
#define ADXL314_REGS_PER_ENTRY					6

/* Max number of FIFO entries */
#define ADXL314_MAX_FIFO_ENTRIES				32

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum adxl314_comm_type
 * @brief Enum for communication type.
 */
enum adxl314_comm_type {
	ADXL314_SPI_COMM = 0,
	ADXL314_I2C_COMM = 1
};

/**
 * @enum adxl					 6314_op_mode
 * @brief Enum for operating mode.
 */enum adxl314_op_mode {
	ADXL314_STDBY = 0,
	ADXL314_MEAS = 1
};

/**
 * @enum adxl314_odr
 * @brief Enum for selecting output data rate.
 */
enum adxl314_odr {
	ADXL314_ODR_6_25HZ,
	ADXL314_ODR_12_5HZ,
	ADXL314_ODR_25HZ,
	ADXL314_ODR_50HZ,
	ADXL314_ODR_100HZ,
	ADXL314_ODR_200HZ,
	ADXL314_ODR_400HZ,
	ADXL314_ODR_800HZ,
	ADXL314_ODR_1600HZ,
	ADXL314_ODR_3200HZ
};

/**
 * @enum adxl314_axis
 * @brief Enum for communication type.
 */
enum adxl314_axis {
	ADXL314_X_AXIS = 0,
	ADXL314_Y_AXIS = 1,
	ADXL314_Z_AXIS = 2
};

/**
 * @enum adxl314_fifo_mode
 * @brief Enum for selecting FIFO operating mode.
 */
enum adxl314_fifo_mode {
	ADXL314_BYPAS_MODE,
	ADXL314_FIFO_MODE,
	ADXL314_STREAM_MODE,
	ADXL314_TRIGGERED_MODE
};

/**
 * @enum adxl314_int_pol
 * @brief Enum for selecting interrupt polarity.
 */
enum adxl314_int_pol {
	ADXL314_INT_ACTIVE_LOW = 0,
	ADXL314_INT_ACTIVE_HIGH = 1
};

/**
 * @enum bit_action
 * @brief Enum for selecting Set/reset bit action.
 */
enum bit_action {
	DISABLE_E = 0,
	ENABLE_E = 1
};

/**
 * @struct adxl314_comm_init_param
 * @brief Structure holding the parameters for ADXL314 comm. initialization.
 */
union adxl314_comm_init_param {
	/** I2C Initialization structure. */
	no_os_i2c_init_param i2c_init;
	/** SPI Initialization structure. */
	no_os_spi_init_param spi_init;
} ;

/**
 * @struct adxl367_fractional_val
 * @brief Data format in which the values are converted.
 */
struct adxl314_fractional_val {
	int64_t integer;
	int32_t fractional;
};

/**
 * @struct adxl314_init_param
 * @brief Structure holding the parameters for ADXL314 device initialization.
 */
struct adxl314_init_param {
	/** Device Communication initialization structure: either SPI or I2C */
	union adxl314_comm_init_param comm_init;
	/** Device Communication type: ADXL355_SPI_COMM, ADXL355_I2C_COMM */
	enum adxl314_comm_type comm_type;
};

struct _adxl314_int_mask {
	uint8_t DRDY_EN1 : 1;
	uint8_t reserved1 : 2;
	uint8_t ACTIVITY_EN1 : 1;
	uint8_t INACTIVITY_EN2 : 1;
	uint8_t reserved2 : 1;
	uint8_t WATERMARK_EN2 : 1;
	uint8_t OVERRUN_EN2 : 1;
};

union adxl314_int_mask {
	struct _adxl314_int_mask fields;
	uint8_t value;
};

// struct _adxl355_sts_reg_flags {
// 	uint8_t DATA_RDY : 1;
// 	uint8_t FIFO_FULL : 1;
// 	uint8_t FIFO_OVR : 1;
// 	uint8_t Activity : 1;
// 	uint8_t NVM_BUSY : 1;
// 	uint8_t reserved : 3;
// };

// union adxl355_sts_reg_flags {
// 	struct _adxl355_sts_reg_flags fields;
// 	uint8_t value;
// };

// struct _adxl355_act_en_flags {
// 	uint8_t ACT_X    : 1;
// 	uint8_t ACT_Y    : 1;
// 	uint8_t ACT_Z    : 1;
// 	uint8_t reserved : 4;
// };

// union adxl355_act_en_flags {
// 	struct _adxl355_act_en_flags fields;
// 	uint8_t value;
// };

// struct adxl355_frac_repr {
// 	int64_t integer;
// 	int32_t fractional;
// } ;

union adxl314_comm_desc {
	/** I2C Descriptor */
	no_os_i2c_desc *i2c_desc;
	/** SPI Descriptor */
	no_os_spi_desc *spi_desc;
};

/**
 * @struct adxl314_dev
 * @brief ADXL314 Device structure.
 */
struct adxl314_dev {
	/** Device communication descriptor */
	union adxl314_comm_desc com_desc;
	/** Device Communication type: ADXL314_SPI_COMM, ADXL314_I2C_COMM */
	enum adxl314_comm_type comm_type;
	enum adxl314_op_mode op_mode;
	enum adxl314_odr odr;
	uint16_t x_offset;
	uint16_t y_offset;
	uint16_t z_offset;
	enum adxl314_fifo_mode		fifo_mode;
	uint8_t fifo_samples;
//	union adxl355_act_en_flags act_en;
	uint8_t act_cnt;
	uint16_t act_thr;
	uint8_t comm_buff[24];
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/*! Init. the comm. peripheral and checks if the ADXL314 part is present. */
int adxl314_init(struct adxl314_dev **device,
		 struct adxl314_init_param init_param);

/*! Free the resources allocated by adxl314_init(). */
int adxl314_remove(struct adxl314_dev *dev);

/* Performs device self-test. */
int adxl367_self_test(struct adxl314_dev *dev);

/*! Place the device into the given operation mode. */
int adxl314_set_op_mode(struct adxl314_dev *dev,
			enum adxl314_op_mode op_mode);

/*! Get the current operation mode of the device. */
int adxl314_get_op_mode(struct adxl314_dev *dev,
			enum adxl314_op_mode *op_mode);

/*! Set output data rate (ODR). */
int adxl314_set_odr(struct adxl314_dev *dev,
		    enum adxl314_odr odr);

/*! Set offset for each axis. */
int adxl314_set_offset(struct adxl314_dev *dev,
		       uint8_t offset,
		       enum adxl314_axis axis);

/*! Read the 3-axis raw data from the accelerometer. */
int adxl314_get_raw_xyz(struct adxl314_dev *dev,
			int16_t *x,
			int16_t *y,
			int16_t *z);

/*! Read the raw output data of each axis and converts it to g. */
int adxl314_get_xyz(struct adxl314_dev *dev,
		    struct adxl314_fractional_val *x,
		    struct adxl314_fractional_val *y,
		    struct adxl314_fractional_val *z);

/* Read the number of FIFO entries register value. */
int adxl314_get_nb_of_fifo_entries(struct adxl314_dev *dev,
				   uint8_t *entries);

/* Set the number of FIFO samples register value. */
int adxl314_set_fifo_samples(struct adxl314_dev *dev,
			     uint8_t samples);

/* Set FIFO mode. */
int adxl314_set_fifo_mode(struct adxl314_dev *dev,
			  enum adxl314_fifo_mode mode);

/*! Read FIFO data and returns the raw values. */
int adxl314_get_raw_fifo_data (struct adxl314_dev *dev,
			       int16_t *raw_x, int16_t *raw_y, int16_t *raw_z, uint8_t *entries);

/*! Read converted values from FIFO. */
int adxl355_get_fifo_data (struct adxl314_dev *dev,
			   struct adxl314_fractional_val *x,
			   struct adxl314_fractional_val *y,
			   struct adxl314_fractional_val *z,
			   uint8_t *entries);

/*! Configure the activity threshold register. */
int adxl314_conf_act_thr(struct adxl314_dev *dev, uint8_t act_thr);

/* Read the status of the watermark bit. */
int adxl314_get_watermark(struct adxl314_dev *dev,
			  uint8_t *status);

/*! Activate/Deactivate AUTOSLEEP. */
int adxl314_autosleep(struct adxl314_dev *dev,
		      enum bit_action enable);

/*! Debug function for checking registers */
void check_regs(struct adxl314_dev *dev);

#endif /* __ADXL314_H__ */
