/*******************************************************************************
 *   @file   adxl314.c
 *   @brief  Implementation of ADXL314 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include "adxl314.h"
#include "no_os_print_log.h"
#include "no_os_util.h"
#include "no_os_delay.h"

// /******************************************************************************/
// /************************ Variable Declarations ******************************/
// /******************************************************************************/
// static uint8_t shadow_reg_val[5] = {0, 0, 0, 0, 0};
// static const uint8_t adxl355_scale_mul[4] = {0, 1, 2, 4};

// /******************************************************************************/
// /************************ Functions Declarations ******************************/
// /******************************************************************************/
// static uint32_t adxl355_accel_array_conv(struct adxl355_dev *dev,
// 		uint8_t *raw_array);
static int64_t adxl314_accel_conv(uint16_t raw_accel);

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/*******************************************************************************
 * @brief Reads data from the device.
 *
 * @param dev          - The device structure.
 * @param base_address - Address of the base register.
 * @param size         - The number of bytes to be read and returned in read_data.
 * @param read_data    - The read data buffer
 *
 * @return ret         - Result of the reading procedure.
*******************************************************************************/
int adxl314_read_device_data(struct adxl314_dev *dev, uint8_t base_address,
			     uint16_t size, uint8_t *read_data)
{
	int ret = -ENXIO;

	if (dev->comm_type == ADXL314_SPI_COMM) {
		dev->comm_buff[0] = ADXL314_SPI_READ | base_address;
		if (size > 1)
			dev->comm_buff[0] |= ADXL314_MULTIBIT;
		ret = no_os_spi_write_and_read(dev->com_desc.spi_desc, dev->comm_buff,
					       1 + size);
		for (uint16_t idx = 0; idx < size; idx++)
			read_data[idx] = dev->comm_buff[idx+1];
	} else {
		ret = no_os_i2c_write(dev->com_desc.i2c_desc, &base_address, 1, 0);
		if (ret)
			return ret;
		ret = no_os_i2c_read(dev->com_desc.i2c_desc, read_data, size, 1);
	}

	return ret;
}

/*******************************************************************************
 * @brief Writes data to the device
 *
 * @param dev          - The device structure.
 * @param base_address - Address of the base register.
 * @param size         - The number of bytes to be written. It is the size
 *                       of the write_data buffer.
 * @param write_data   - The data which is going to be written.
 *
 * @return ret         - Result of the writing procedure.
*******************************************************************************/
int adxl314_write_device_data(struct adxl314_dev *dev, uint8_t base_address,
			      uint16_t size, uint8_t *write_data)
{
	int ret = ENXIO;

	for (uint16_t idx = 0; idx < size; idx++)
		dev->comm_buff[1+idx] = write_data[idx];

	if (dev->comm_type == ADXL314_SPI_COMM) {
		dev->comm_buff[0] = ADXL314_SPI_WRITE | base_address;
		if (size > 1)
			dev->comm_buff[0] |= ADXL314_MULTIBIT;
		ret = no_os_spi_write_and_read(dev->com_desc.spi_desc, dev->comm_buff,
					       size + 1);
	} else {
		dev->comm_buff[0] = base_address;
		ret = no_os_i2c_write(dev->com_desc.i2c_desc, dev->comm_buff, size + 1, 1);
	}

	return ret;
}

/*******************************************************************************
 * @brief Set or clear a bit of a specific register
 *
 * @param dev          - The device structure.
 * @param base_address - Address of the base register.
 * @param action         - The number of bytes to be written. It is the size
 *                       of the write_data buffer.
 * @param write_data   - The data which is going to be written.
 *
 * @return ret - Result of the remove procedure.
*******************************************************************************/
int adxl314_control_reg_bit(struct adxl314_dev *dev,
			    uint8_t base_address,
			    enum bit_action action,
			    uint8_t modified_bit)
{
	int ret = -EFAULT;
	uint8_t reg_data = 0x00;

	/* Get register value */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(base_address),
				       1, &reg_data);
	if (ret)
		goto error_com;

	/* Set bit accorfding to ACTION */
	if (action == DISABLE_E)
		reg_data &= ~modified_bit;
	else
		reg_data |= modified_bit;

	/* Write data to register for turning on desired operation mode. */
	ret = adxl314_write_device_data(dev, ADXL314_ADDR(base_address),
					1, &reg_data);
	if (ret)
		goto error_com;

	return ret;
error_com:
	return -EPIPE;
}

/***************************************************************************//**
 * @brief Performs a masked write to a register.
 *
 * @param dev      - The device structure.
 * @param reg_addr - The register address.
 * @param data     - The register data.
 * @param mask     - The mask.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_reg_write_msk(struct adxl314_dev *dev,
			  uint8_t reg_addr,
			  uint8_t data,
			  uint8_t mask)
{
	int ret = -EINVAL;
	uint8_t reg_data;

	ret = adxl314_read_device_data(dev, ADXL314_ADDR(reg_addr),
				       1, &reg_data);
	if (ret)
		return ret;

	/* Clear data */
	reg_data &= ~mask;
	/* Set information to be written */
	reg_data |= data;

	/* Write data to register */
	return adxl314_write_device_data(dev, ADXL314_ADDR(reg_addr),
					 1, &reg_data);
}

/*******************************************************************************
 * @brief Initializes the communication peripheral and checks if the ADXL314
 *        part is present.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       		   parameters.
 *
 * @return ret       - Result of the initialization procedure.
*******************************************************************************/
int adxl314_init(struct adxl314_dev **device,
		 struct adxl314_init_param init_param)
{
	struct adxl314_dev *dev;
	int ret = -EFAULT;
	uint8_t reg_value = 0;

	dev = (struct adxl314_dev *)calloc(1, sizeof(*dev));

	if (!dev)
		return -ENOMEM;

	dev->comm_type = init_param.comm_type;

	if (dev->comm_type == ADXL314_SPI_COMM) {
		ret = no_os_spi_init(&dev->com_desc.spi_desc, &(init_param.comm_init.spi_init));
		if (ret)
			goto error_dev;
	} else {
		ret = no_os_i2c_init(&dev->com_desc.i2c_desc, &init_param.comm_init.i2c_init);
		if (ret)
			goto error_dev;
	}

	/* Check for device ID */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_DEVID_AD), 1,
				       &reg_value);
	if (ret || (reg_value != ADXL314_DEVID))
		goto error_com;

	*device = dev;

	/* Get ODR */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_BW_RATE), 1,
				       &reg_value);
	if (!ret)
		dev->odr = reg_value & ADXL314_RATE_MSK;

	/* Get offsets */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(0)), 1,
				       &reg_value);
	if (!ret)
		dev->x_offset = reg_value;
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(1)), 1,
				       &reg_value);
	if (!ret)
		dev->y_offset = reg_value;
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(2)), 1,
				       &reg_value);
	if (!ret)
		dev->z_offset = reg_value;

	/* Get fifo mode setting */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_FIFO_CTL), 1,
				       &reg_value);
	if (!ret)
		dev->fifo_mode = (reg_value & ADXL314_REG_FIFO_CTL_MODE_MSK) >> 6;

	/* Get fifo samples setting */
	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_FIFO_CTL), 1,
				       &reg_value);
	if (!ret)
		dev->fifo_samples = (reg_value & ADXL314_REG_FIFO_CTL_SAMPLES_MSK);

	pr_info("ADXL314 successfully initialized.\n");

	return ret;
error_com:
	if (dev->comm_type == ADXL314_SPI_COMM)
		no_os_spi_remove(dev->com_desc.spi_desc);
	else
		no_os_i2c_remove(dev->com_desc.i2c_desc);
	free(dev);//
	return -EPIPE;
error_dev:
	free(dev);
	return -ENODEV;
}

/*******************************************************************************
 * @brief Free the resources allocated by adxl355_init().
 *
 * @param dev - The device structure.
 *
 * @return ret - Result of the remove procedure.
*******************************************************************************/
int adxl314_remove(struct adxl314_dev *dev)
{
	int ret = -EFAULT;

	if (dev->comm_type == ADXL314_SPI_COMM)
		ret = no_os_spi_remove(dev->com_desc.spi_desc);
	else
		ret = no_os_i2c_remove(dev->com_desc.i2c_desc);

	free(dev);

	return ret;
}

/*******************************************************************************
 * @brief Performs self test.
 *
 * @param dev - The device structure.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl367_self_test(struct adxl314_dev *dev)
{
//ToDo
	return 0;
}

/*******************************************************************************
 * @brief Places the device into the given operation mode.
 *
 * @param dev     - The device structure.
 * @param op_mode - Operation mode mode.
 *
 * @return ret    - Result of the setting operation procedure.
*******************************************************************************/
int adxl314_set_op_mode(struct adxl314_dev *dev, enum adxl314_op_mode op_mode)
{
	int ret = -EFAULT;

	/* Set operation mode */
	switch(op_mode) {
	case ADXL314_STDBY:
		ret = adxl314_control_reg_bit(dev, ADXL314_ADDR(ADXL314_REG_POWER_CTL),
					      DISABLE_E, ADXL314_POWER_CTL_MEASURE);
		break;
	case ADXL314_MEAS:
		ret = adxl314_control_reg_bit(dev, ADXL314_ADDR(ADXL314_REG_POWER_CTL),
					      ENABLE_E, ADXL314_POWER_CTL_MEASURE);
		break;
	default:
		pr_err("ADXL314 operation mode not supported!\n");
		goto error_cmd;
		break;
	}

	if (ret)
		goto error_com;

	dev->op_mode = op_mode;

	return ret;
error_com:
	return -EPIPE;
error_cmd:
	return -ENOSYS;
}

/*******************************************************************************
 * @brief Gets the current operation mode of the device
 *
 * @param dev     - The device structure.
 * @param op_mode - Read operation mode.
 *
 * @return ret    - Result of the setting operation procedure (0 for success).
*******************************************************************************/
int adxl314_get_op_mode(struct adxl314_dev *dev,
			enum adxl314_op_mode *op_mode)
{
	int ret = -EINVAL;
	uint8_t reg_value = 0;

	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_POWER_CTL),
				       1, &reg_value);
	if (ret)
		goto error_com;

	if (reg_value & ADXL314_POWER_CTL_MEASURE)
		*op_mode = ADXL314_MEAS;
	else
		*op_mode = ADXL314_STDBY;

	return ret;
error_com:
	return -EPIPE;
}

/*******************************************************************************
 * @brief Set output data rate (ODR)
 *
 * @param dev     - The device structure.
 * @param odr     - Data rate.
 *
 * @return ret    - Result of the setting operation procedure (0 for success).
*******************************************************************************/
int adxl314_set_odr(struct adxl314_dev *dev, enum adxl314_odr odr)
{
	int ret = -EINVAL;
	uint8_t reg_value = 0;
	uint8_t data = 0;

	if (odr > ADXL314_ODR_3200HZ)
		return -EINVAL;

	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_BW_RATE),
				       1, &reg_value);
	if (ret)
		goto error_com;

	/* Clear data rate value */
	reg_value &= ~ADXL314_RATE_MSK;

	/* Set new register value. */
	data = (odr + ADXL314_ODR_OFFSET_VAL) & ADXL314_RATE_MSK;
	reg_value |= data;

	/* Write data to register for setting odr. */
	ret = adxl314_write_device_data(dev, ADXL314_ADDR(ADXL314_REG_BW_RATE),
					1, &reg_value);
	if (ret)
		goto error_com;

	dev->odr = odr;

	return ret;
error_com:
	return -EPIPE;
}

/*******************************************************************************
 * @brief Set offset for each axis.
 *
 * @param dev      - The device structure.
 * @param offset   - Offset.
 * @param axis     - Axis to apply offset.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_set_offset(struct adxl314_dev *dev, uint8_t offset,
		       enum adxl314_axis axis)
{
	int ret = -EINVAL;

	if (axis > ADXL314_Z_AXIS)
		return -EINVAL;

	/* Write data to register for setting odr. */
	ret = adxl314_write_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(axis)),
					1, &offset);
	if (ret)
		return ret;

	switch (axis) {
	case ADXL314_X_AXIS:
		dev->x_offset = offset;
		break;
	case ADXL314_Y_AXIS:
		dev->y_offset = offset;
		break;
	case ADXL314_Z_AXIS:
		dev->z_offset = offset;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*******************************************************************************
 * @brief Reads the 3-axis raw data from the accelerometer.
 *
 * @param dev - The device structure.
 * @param x   - Stores the X-axis data (as two's complement).
 * @param y   - Stores the Y-axis data (as two's complement).
 * @param z   - Stores the Z-axis data (as two's complement).
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_get_raw_xyz(struct adxl314_dev *dev,
			int16_t *x,
			int16_t *y,
			int16_t *z)
{
	int ret = -EINVAL;
	uint8_t xyz_values[6] = { 0 };

	ret = adxl314_read_device_data(dev,
				       ADXL314_ADDR(ADXL314_REG_DATA_AXIS(0)),
				       ADXL314_REGS_PER_ENTRY,
				       xyz_values);
	if (ret)
		return ret;

	// result is 13 bits long with sign extended
	*x = ((int16_t)xyz_values[1] << 8) + (xyz_values[0]);
	*y = ((int16_t)xyz_values[3] << 8) + (xyz_values[2]);
	*z = ((int16_t)xyz_values[5] << 8) + (xyz_values[4]);

	return ret;
}

/*******************************************************************************
 * @brief Read the raw output data of each axis and convert it to g.
 *
 * @param dev - The device structure.
 * @param x   - X-axis data.
 * @param y   - Y-axis data.
 * @param z   - Z-axis data.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_get_xyz(struct adxl314_dev *dev,
		    struct adxl314_fractional_val *x,
		    struct adxl314_fractional_val *y,
		    struct adxl314_fractional_val *z)
{
	int ret = -EINVAL;
	int16_t raw_accel_x = 0;
	int16_t raw_accel_y = 0;
	int16_t raw_accel_z = 0;

	ret = adxl314_get_raw_xyz(dev, &raw_accel_x, &raw_accel_y, &raw_accel_z);
	if (ret)
		return ret;

	x->integer = no_os_div_s64_rem(adxl314_accel_conv(raw_accel_x),
				       ADXL314_ACC_SCALE_FACTOR_DIV, &(x->fractional));
	y->integer = no_os_div_s64_rem(adxl314_accel_conv(raw_accel_y),
				       ADXL314_ACC_SCALE_FACTOR_DIV, &(y->fractional));
	z->integer = no_os_div_s64_rem(adxl314_accel_conv(raw_accel_z),
				       ADXL314_ACC_SCALE_FACTOR_DIV, &(z->fractional));

	return ret;
}

/***************************************************************************//**
 * @brief Reads the number of FIFO entries.
 *
 * @param dev     - The device structure.
 * @param entries - Entries number. Each entry contains data for the three axis.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_get_nb_of_fifo_entries(struct adxl314_dev *dev,
				   uint8_t *entries)
{
	int ret = -EINVAL;
	uint8_t reg_val = 0;

	ret = adxl314_read_device_data(dev,
				       ADXL314_ADDR(ADXL314_REG_FIFO_STATUS),
				       1,
				       &reg_val);
	if (ret)
		return ret;

	// last 2 bits from FIFO_ENTRIES_H and 8 bits from FIFO_ENTRIES_L
	*entries = reg_val & ADXL314_REG_FIFO_STS_ENTRIES_MSK;

	return 0;
}

/***************************************************************************//**
 * @brief Sets the number of FIFO sample sets.
 *
 * @param dev     - The device structure.
 * @param samples - Sample sets number. For example, if ADXL367_FIFO_FORMAT_XYZ
 * 			is selected, a value of 2 will represent 6 entries.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_set_fifo_samples(struct adxl314_dev *dev,
			     uint8_t samples)
{
	int ret = -EINVAL;

	if (samples > ADXL314_MAX_FIFO_SAMPLES_VAL)
		return -EINVAL;

	// write no of samples to ADXL314_REG_FIFO_SAMPLES
	ret = adxl314_reg_write_msk(dev,
				    ADXL314_ADDR(ADXL314_REG_FIFO_CTL),
				    no_os_field_prep(ADXL314_REG_FIFO_CTL_SAMPLES_MSK, samples),
				    ADXL314_REG_FIFO_CTL_SAMPLES_MSK);
	if (ret)
		return ret;

	dev->fifo_samples = samples;

	return ret;
}

/***************************************************************************//**
 * @brief Set FIFO mode.
 *
 * @param dev  - The device structure.
 * @param mode - FIFO mode.
 * 			   Accepted values: ADXL314_FIFO_DISABLED,
 *								ADXL314_OLDEST_SAVED,
 *								ADXL314_STREAM_MODE,
 *								ADXL314_TRIGGERED_MODE
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_set_fifo_mode(struct adxl314_dev *dev,
			  enum adxl314_fifo_mode mode)
{
	int ret = -EINVAL;

	if (mode > ADXL314_TRIGGERED_MODE)
		return -EINVAL;

	ret = adxl314_reg_write_msk(dev,
				    ADXL314_ADDR(ADXL314_REG_FIFO_CTL),
				    no_os_field_prep(ADXL314_REG_FIFO_CTL_MODE_MSK, mode),
				    ADXL314_REG_FIFO_CTL_MODE_MSK);
	if (ret)
		return ret;

	dev->fifo_mode = mode;

	return ret;
}

/*******************************************************************************
 * @brief Read FIFO data and returns the raw values.
 *
 * @param dev          - The device structure.
 * @param entries      - The number of fifo entries.
 * @param raw_x        - Raw x-axis data.
 * @param raw_y        - Raw y-axis data.
 * @param raw_z        - Raw z-axis data.
 *
 * @return ret         - Result of the configuration procedure.
 * 						 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl314_get_raw_fifo_data(struct adxl314_dev *dev,
			      int16_t *raw_x, int16_t *raw_y, int16_t *raw_z, uint8_t *entries)
{
	int ret = -EINVAL;
	uint8_t buff_idx = 0;
	uint8_t xyz_values[ADXL314_MAX_FIFO_ENTRIES * ADXL314_REGS_PER_ENTRY] = { 0 };

	/* Get the number of FIFO entries */
	ret = adxl314_get_nb_of_fifo_entries(dev, entries);
	if (ret)
		return ret;

	if ((*entries) > 0) {
		for (uint8_t idx = 0; idx < (*entries); idx++) {
			ret = adxl314_read_device_data(dev,
						       ADXL314_ADDR(ADXL314_REG_DATA_AXIS(0)),
						       ADXL314_REGS_PER_ENTRY,
						       &xyz_values[idx * ADXL314_REGS_PER_ENTRY]);
			if (ret)
				return ret;

			//wait 5us
			no_os_udelay(5);
		}

		for (uint8_t idx = 0; idx < (*entries); idx++) {
			buff_idx = idx * ADXL314_REGS_PER_ENTRY;
			raw_x[idx] = ((int16_t)xyz_values[buff_idx+1] << 8) + (xyz_values[buff_idx]);
			raw_y[idx] = ((int16_t)xyz_values[buff_idx+3] << 8) + (xyz_values[buff_idx+2]);
			raw_z[idx] = ((int16_t)xyz_values[buff_idx+5] << 8) + (xyz_values[buff_idx+4]);
		}
	}

	return ret;
}

/*******************************************************************************
 * @brief Read converted values from FIFO.
 *
 * @param dev       - The device structure.
 * @param x  	    - X axis fractional data buffer.
 * @param y  	    - Y axis fractional data buffer.
 * @param z  	    - Z axis fractional data buffer.
 * @param entries   - Number of read entries.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int adxl355_get_fifo_data(struct adxl314_dev *dev,
			  struct adxl314_fractional_val *x,
			  struct adxl314_fractional_val *y,
			  struct adxl314_fractional_val *z,
			  uint8_t *entries)
{
	int ret = -EINVAL;
	int16_t raw_x[ADXL314_MAX_FIFO_ENTRIES] = {0};
	int16_t raw_y[ADXL314_MAX_FIFO_ENTRIES] = {0};
	int16_t raw_z[ADXL314_MAX_FIFO_ENTRIES] = {0};

	ret = adxl314_get_raw_fifo_data(dev, raw_x, raw_y, raw_z, entries);
	if (ret)
		return ret;

	pr_info("no of fifo entries %d.\n", *entries);

	if ((*entries) > 0) {
		for (uint8_t idx = 0; idx < (*entries); idx++) {
			x[idx].integer = no_os_div_s64_rem(adxl314_accel_conv(raw_x[idx]),
							   ADXL314_ACC_SCALE_FACTOR_DIV, &(x[idx].fractional));
			y[idx].integer = no_os_div_s64_rem(adxl314_accel_conv(raw_y[idx]),
							   ADXL314_ACC_SCALE_FACTOR_DIV, &(y[idx].fractional));
			z[idx].integer = no_os_div_s64_rem(adxl314_accel_conv(raw_z[idx]),
							   ADXL314_ACC_SCALE_FACTOR_DIV, &(z[idx].fractional));
		}
	}

	return ret;
}

/*******************************************************************************
 * @brief Configure the activity threshold register.
 *        The scale factor is 780 mg/LSB.
 *
 * @param dev     - The device structure.
 * @param act_thr - Activity threshold value.
 *
 * @return ret    - Result of the configuration procedure.
*******************************************************************************/
int adxl314_conf_act_thr(struct adxl314_dev *dev, uint8_t act_thr)
{
	int ret = -EINVAL;
	uint8_t data[1] = {act_thr};

	ret = adxl314_write_device_data(dev,
					ADXL314_ADDR(ADXL314_REG_THRESH_ACT),
					1,
					data);

	if (!ret)
		dev->act_thr = act_thr;

	return ret;
}

/*******************************************************************************
 * @brief Read the status of the watermark bit.
 *
 * @param dev     - The device structure.
 * @param dev     - Status (0 or 1).
 *
 * @return ret    - Result of the setting operation procedure.
*******************************************************************************/
int adxl314_get_watermark(struct adxl314_dev *dev,
			  uint8_t *status)
{
	int ret = -EINVAL;
	uint8_t reg_val = 0;

	ret = adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_INT_SRC),
				       1, &reg_val);
	if (ret)
		return ret;

	*status = (reg_val & ADXL314_REG_INT_SRC_WATERMARK) >> 1;

	return ret;
}

/*******************************************************************************
 * @brief Activate/Deactivate AUTOSLEEP.
 *
 * @param dev     - The device structure.
 *
 * @return ret    - Result of the setting operation procedure.
*******************************************************************************/
int adxl314_autosleep(struct adxl314_dev *dev, enum bit_action enable)
{
	int ret = -EFAULT;

	/* Set STDBY operation mode */
	ret = adxl314_set_op_mode(dev, ADXL314_STDBY);
	if (ret)
		goto error_com;

	/* Clear the AUTOSLEEP bit */
	ret = adxl314_control_reg_bit(dev, ADXL314_ADDR(ADXL314_REG_POWER_CTL),
				      enable, ADXL314_POWER_CTL_AUTO_SLEEP);
	if (ret)
		goto error_com;

	/* Set STDBY operation mode */
	ret = adxl314_set_op_mode(dev, ADXL314_MEAS);
	if (ret)
		goto error_com;

	return ret;
error_com:
	return -EPIPE;
}

/*******************************************************************************
 * @brief Debug function for checking registers
 * ToDo REMOVE - DEBUG function
 *
 * @param dev       - The device structure.
 *
 * @return ret      - None.
*******************************************************************************/
void check_regs(struct adxl314_dev *dev)
{
	uint8_t reg_value = 0;

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(0)),
				 1, &reg_value);

	pr_info("X offset: %x\n",reg_value);

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(1)),
				 1, &reg_value);

	pr_info("Y offset: %x\n",reg_value);

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_OFS_AXIS(2)),
				 1, &reg_value);

	pr_info("Z offset: %x\n",reg_value);

	pr_info("Z offset dev: %x\n",dev->z_offset);

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_BW_RATE),
				 1, &reg_value);

	pr_info("Rate: %x\n",reg_value);
	pr_info("Rate dev : %x\n",dev->odr);

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_FIFO_CTL),
				 1, &reg_value);

	pr_info("FIFO_CTL: %x\n",reg_value);

	adxl314_read_device_data(dev, ADXL314_ADDR(ADXL314_REG_FIFO_STATUS),
				 1, &reg_value);

	pr_info("FIFO_STATUS: %x\n",reg_value);
}

/*******************************************************************************
 * @brief Converts raw acceleration value to g value.
 *
 * @param raw_accel - Raw acceleration value.
 *
 * @return ret      - Converted data.
*******************************************************************************/
static int64_t adxl314_accel_conv(uint16_t raw_accel)
{
	int64_t accel_data;

	/* Convert from 16-bit unsigned to 16-bit signed number. */
	accel_data = (int16_t)raw_accel;

	/* Apply scale factor based on the selected range. */
	return ((int64_t)(accel_data * ADXL314_ACC_SCALE_FACTOR_MUL));
}
