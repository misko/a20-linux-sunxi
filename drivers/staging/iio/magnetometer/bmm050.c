/*
 * Last modified: Jul 26th, 2012
 * Revision: V2.4
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "bmm050.h"
#include "bs_log.h"

static struct bmm050 *p_bmm050;

BMM050_RETURN_FUNCTION_TYPE bmm050_init(struct bmm050 *bmm050)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	p_bmm050 = bmm050;

	p_bmm050->dev_addr = BMM050_I2C_ADDRESS;

	/* set device from suspend into sleep mode */
	bmm050_set_powermode(BMM050_ON);

	/* wait two millisecond for bmc to settle */
	p_bmm050->delay_msec(BMM050_DELAY_SETTLING_TIME);

	/*Read CHIP_ID and REv. info */
	comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_CHIP_ID, a_data_u8r, 1);
	p_bmm050->company_id = a_data_u8r[0];

	/* Function to initialise trim values */
	bmm050_init_trim_registers();
	bmm050_set_presetmode(BMM050_PRESETMODE_REGULAR);
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_presetmode(unsigned char mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	switch (mode) {
	case BMM050_PRESETMODE_LOWPOWER:
		/* Set the data rate for Low Power mode */
		comres = bmm050_set_datarate(BMM050_LOWPOWER_DR);
		/* Set the XY-repetitions number for Low Power mode */
		comres |= bmm050_set_repetitions_XY(BMM050_LOWPOWER_REPXY);
		/* Set the Z-repetitions number  for Low Power mode */
		comres |= bmm050_set_repetitions_Z(BMM050_LOWPOWER_REPZ);
		break;
	case BMM050_PRESETMODE_REGULAR:
		/* Set the data rate for Regular mode */
		comres = bmm050_set_datarate(BMM050_REGULAR_DR);
		/* Set the XY-repetitions number for Regular mode */
		comres |= bmm050_set_repetitions_XY(BMM050_REGULAR_REPXY);
		/* Set the Z-repetitions number  for Regular mode */
		comres |= bmm050_set_repetitions_Z(BMM050_REGULAR_REPZ);
		break;
	case BMM050_PRESETMODE_HIGHACCURACY:
		/* Set the data rate for High Accuracy mode */
		comres = bmm050_set_datarate(BMM050_HIGHACCURACY_DR);
		/* Set the XY-repetitions number for High Accuracy mode */
		comres |= bmm050_set_repetitions_XY(BMM050_HIGHACCURACY_REPXY);
		/* Set the Z-repetitions number  for High Accuracyr mode */
		comres |= bmm050_set_repetitions_Z(BMM050_HIGHACCURACY_REPZ);
		break;
	case BMM050_PRESETMODE_ENHANCED:
		/* Set the data rate for Enhanced Accuracy mode */
		comres = bmm050_set_datarate(BMM050_ENHANCED_DR);
		/* Set the XY-repetitions number for High Enhanced mode */
		comres |= bmm050_set_repetitions_XY(BMM050_ENHANCED_REPXY);
		/* Set the Z-repetitions number  for High Enhanced mode */
		comres |= bmm050_set_repetitions_Z(BMM050_ENHANCED_REPZ);
		break;
	default:
		comres = E_BMM050_OUT_OF_RANGE;
		break;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_functional_state(
		unsigned char functional_state)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		switch (functional_state) {
		case BMM050_NORMAL_MODE:
			comres = bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres |= bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
						BMM050_DELAY_SUSPEND_SLEEP);
			}
			{
				comres |= p_bmm050->BMM050_BUS_READ_FUNC(
						p_bmm050->dev_addr,
						BMM050_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
				v_data1_u8r = BMM050_SET_BITSLICE(
						v_data1_u8r,
						BMM050_CNTL_OPMODE,
						BMM050_NORMAL_MODE);
				comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
						p_bmm050->dev_addr,
						BMM050_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
			}
			break;
		case BMM050_SUSPEND_MODE:
			comres = bmm050_set_powermode(BMM050_OFF);
			break;
		case BMM050_FORCED_MODE:
			comres = bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres = bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
						BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_OPMODE, BMM050_ON);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		case BMM050_SLEEP_MODE:
			bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres = bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
						BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_OPMODE,
					BMM050_SLEEP_MODE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		default:
			comres = E_BMM050_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_functional_state(
		unsigned char *functional_state)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_OPMODE__REG,
				&v_data_u8r, 1);
		*functional_state = BMM050_GET_BITSLICE(
				v_data_u8r, BMM050_CNTL_OPMODE);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ(struct bmm050_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
				BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
				BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
				BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
				BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
				BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16)((((BMM050_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ_s32(
	struct bmm050_mdata_s32 *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
				BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
				BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
				BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
				BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
				BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16)((((BMM050_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_s32(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_s32(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_s32(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

#ifdef ENABLE_FLOAT
BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ_float(
	struct bmm050_mdata_float *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
				BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
				BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
				BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
				BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
				BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16)((((BMM050_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_float(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_float(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_float(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);

	    /* Output raw resistance value */
	    mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}
#endif

BMM050_RETURN_FUNCTION_TYPE bmm050_read_register(unsigned char addr,
		unsigned char *data, unsigned char len)
{

#ifdef CONFIG_MACH_LGE
//LGE_CHANGE : 2012-11-09 Sanghun,Lee(eee3114.@lge.com)wbt sanity check
//wbt 412962 	'comres' is used uninitialized in this function.	
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
#else
	BMM050_RETURN_FUNCTION_TYPE comres;
#endif
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			addr, data, len);
   }
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_write_register(unsigned char addr,
	    unsigned char *data, unsigned char len)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
			addr, data, len);
   }
   return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_selftest(unsigned char selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr, BMM050_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r, BMM050_CNTL_S_TEST, selftest);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr, BMM050_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_self_test_XYZ(
		unsigned char *self_testxyz)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[5], v_result_u8r = 0x00;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr, BMM050_DATAX_LSB_TESTX__REG,
				a_data_u8r, 5);

		v_result_u8r = BMM050_GET_BITSLICE(a_data_u8r[4],
				BMM050_DATAZ_LSB_TESTZ);

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM050_GET_BITSLICE(
					a_data_u8r[2], BMM050_DATAY_LSB_TESTY));

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM050_GET_BITSLICE(
					a_data_u8r[0], BMM050_DATAX_LSB_TESTX));

		*self_testxyz = v_result_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_spi3(unsigned char value)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
			BMM050_POWER_CNTL_SPI3_EN, value);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
		    BMM050_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_datarate(unsigned char data_rate)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_DR__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
				BMM050_CNTL_DR, data_rate);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_DR__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_datarate(unsigned char *data_rate)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_DR__REG,
				&v_data_u8r, 1);
		*data_rate = BMM050_GET_BITSLICE(v_data_u8r,
				BMM050_CNTL_DR);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_perform_advanced_selftest(
		BMM050_S16 *diff_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres;
	BMM050_S16 result_positive, result_negative;
	struct bmm050_mdata mdata;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		/* set sleep mode to prepare for forced measurement.
		 * If sensor is off, this will turn it on
		 * and respect needed delays. */
		comres = bmm050_set_functional_state(BMM050_SLEEP_MODE);

		/* set normal accuracy mode */
		comres |= bmm050_set_repetitions_Z(BMM050_LOWPOWER_REPZ);
		/* 14 repetitions Z in normal accuracy mode */

		/* disable X, Y channel */
		comres |= bmm050_set_control_measurement_x(
				BMM050_CHANNEL_DISABLE);
		comres |= bmm050_set_control_measurement_y(
				BMM050_CHANNEL_DISABLE);

		/* enable positive current and force a
		 * measurement with positive field */
		comres |= bmm050_set_adv_selftest(
				BMM050_ADVANCED_SELFTEST_POSITIVE);
		comres |= bmm050_set_functional_state(BMM050_FORCED_MODE);
		/* wait for measurement to complete */
		p_bmm050->delay_msec(4);

		/* read result from positive field measurement */
		comres |= bmm050_read_mdataXYZ(&mdata);
		result_positive = mdata.dataz;

		/* enable negative current and force a
		 * measurement with negative field */
		comres |= bmm050_set_adv_selftest(
				BMM050_ADVANCED_SELFTEST_NEGATIVE);
		comres |= bmm050_set_functional_state(BMM050_FORCED_MODE);
		p_bmm050->delay_msec(4); /* wait for measurement to complete */

		/* read result from negative field measurement */
		comres |= bmm050_read_mdataXYZ(&mdata);
		result_negative = mdata.dataz;

		/* turn off self test current */
		comres |= bmm050_set_adv_selftest(
				BMM050_ADVANCED_SELFTEST_OFF);

		/* enable X, Y channel */
		comres |= bmm050_set_control_measurement_x(
				BMM050_CHANNEL_ENABLE);
		comres |= bmm050_set_control_measurement_y(
				BMM050_CHANNEL_ENABLE);

		/* write out difference in positive and negative field.
		 * This should be ~ 200 mT = 3200 LSB */
		*diff_z = (result_positive - result_negative);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_init_trim_registers(void)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_X1, (unsigned char *)&p_bmm050->dig_x1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Y1, (unsigned char *)&p_bmm050->dig_y1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_X2, (unsigned char *)&p_bmm050->dig_x2, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Y2, (unsigned char *)&p_bmm050->dig_y2, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_XY1, (unsigned char *)&p_bmm050->dig_xy1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_XY2, (unsigned char *)&p_bmm050->dig_xy2, 1);

	/* shorts can not be recasted into (unsigned char*)
	 * due to possible mixup between trim data
	 * arrangement and memory arrangement */

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Z1_LSB, a_data_u8r, 2);
	p_bmm050->dig_z1 = (BMM050_U16)((((BMM050_U16)((unsigned char)
						a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Z2_LSB, a_data_u8r, 2);
	p_bmm050->dig_z2 = (BMM050_S16)((((BMM050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Z3_LSB, a_data_u8r, 2);
	p_bmm050->dig_z3 = (BMM050_S16)((((BMM050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_Z4_LSB, a_data_u8r, 2);
	p_bmm050->dig_z4 = (BMM050_S16)((((BMM050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_DIG_XYZ1_LSB, a_data_u8r, 2);
	a_data_u8r[1] = BMM050_GET_BITSLICE(a_data_u8r[1], BMM050_DIG_XYZ1_MSB);
	p_bmm050->dig_xyz1 = (BMM050_U16)((((BMM050_U16)
					((unsigned char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_adv_selftest(unsigned char adv_selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		switch (adv_selftest) {
		case BMM050_ADVANCED_SELFTEST_OFF:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_ADV_ST,
					BMM050_ADVANCED_SELFTEST_OFF);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMM050_ADVANCED_SELFTEST_POSITIVE:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_ADV_ST,
					BMM050_ADVANCED_SELFTEST_POSITIVE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMM050_ADVANCED_SELFTEST_NEGATIVE:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_ADV_ST,
					BMM050_ADVANCED_SELFTEST_NEGATIVE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		default:
			break;
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_adv_selftest(unsigned char *adv_selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
			BMM050_CNTL_ADV_ST__REG, &v_data_u8r, 1);
		*adv_selftest = BMM050_GET_BITSLICE(v_data_u8r,
			BMM050_CNTL_ADV_ST);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_presetmode(
	unsigned char *mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char data_rate = 0;
	unsigned char repetitionsxy = 0;
	unsigned char repetitionsz = 0;

	/* Get the current data rate */
	comres = bmm050_get_datarate(&data_rate);
	/* Get the preset number of XY Repetitions */
	comres |= bmm050_get_repetitions_XY(&repetitionsxy);
	/* Get the preset number of Z Repetitions */
	comres |= bmm050_get_repetitions_Z(&repetitionsz);
	if ((data_rate == BMM050_LOWPOWER_DR) && (
		repetitionsxy == BMM050_LOWPOWER_REPXY) && (
		repetitionsz == BMM050_LOWPOWER_REPZ)) {
		*mode = BMM050_PRESETMODE_LOWPOWER;
	} else {
		if ((data_rate == BMM050_REGULAR_DR) && (
			repetitionsxy == BMM050_REGULAR_REPXY) && (
			repetitionsz == BMM050_REGULAR_REPZ)) {
			*mode = BMM050_PRESETMODE_REGULAR;
		} else {
			if ((data_rate == BMM050_HIGHACCURACY_DR) && (
				repetitionsxy == BMM050_HIGHACCURACY_REPXY) && (
				repetitionsz == BMM050_HIGHACCURACY_REPZ)) {
					*mode = BMM050_PRESETMODE_HIGHACCURACY;
			} else {
				if ((data_rate == BMM050_ENHANCED_DR) && (
				repetitionsxy == BMM050_ENHANCED_REPXY) && (
				repetitionsz == BMM050_ENHANCED_REPZ)) {
					*mode = BMM050_PRESETMODE_ENHANCED;
				} else {
					*mode = E_BMM050_UNDEFINED_MODE;
				}
			}
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_powermode(unsigned char *mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		*mode = BMM050_GET_BITSLICE(v_data_u8r,
				BMM050_POWER_CNTL_PCB);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_powermode(unsigned char mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
				BMM050_POWER_CNTL_PCB, mode);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_repetitions_XY(
		unsigned char *no_repetitions_xy)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
		*no_repetitions_xy = v_data_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_repetitions_XY(
		unsigned char no_repetitions_xy)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_xy;
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_repetitions_Z(
		unsigned char *no_repetitions_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_NO_REPETITIONS_Z,
				&v_data_u8r, 1);
		*no_repetitions_z = v_data_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_repetitions_Z(
		unsigned char no_repetitions_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_z;
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
				BMM050_NO_REPETITIONS_Z, &v_data_u8r, 1);
	}
	return comres;
}

BMM050_S16 bmm050_compensate_X(BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_x != BMM050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMM050_S16)(((BMM050_U16)
				((((BMM050_S32)p_bmm050->dig_xyz1) << 14) /
				 (data_R != 0 ? data_R : p_bmm050->dig_xyz1))) -
				((BMM050_U16)0x4000)));
		inter_retval = ((BMM050_S16)((((BMM050_S32)mdata_x) *
				((((((((BMM050_S32)p_bmm050->dig_xy2) *
			      ((((BMM050_S32)inter_retval) *
				((BMM050_S32)inter_retval)) >> 7)) +
			     (((BMM050_S32)inter_retval) *
			      ((BMM050_S32)(((BMM050_S16)p_bmm050->dig_xy1)
			      << 7)))) >> 9) +
			   ((BMM050_S32)0x100000)) *
			  ((BMM050_S32)(((BMM050_S16)p_bmm050->dig_x2) +
			  ((BMM050_S16)0xA0)))) >> 12)) >> 13)) +
			(((BMM050_S16)p_bmm050->dig_x1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S32 bmm050_compensate_X_s32 (BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	BMM050_S32 retval;

	retval = bmm050_compensate_X(mdata_x, data_R);
	if (retval == (BMM050_S32)BMM050_OVERFLOW_OUTPUT)
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_X_float (BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_x != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
	   ) {
		if (data_R != 0) {
			inter_retval = ((((float)p_bmm050->dig_xyz1)*16384.0f
				/data_R)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_x * ((((((float)p_bmm050->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval*((float)p_bmm050->dig_xy1)/16384.0f))
			+ 256.0f) *	(((float)p_bmm050->dig_x2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm050->dig_x1) * 8.0f))/16.0f;
	} else {
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM050_S16 bmm050_compensate_Y(BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_y != BMM050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMM050_S16)(((BMM050_U16)(((
			(BMM050_S32)p_bmm050->dig_xyz1) << 14) /
			(data_R != 0 ?
			 data_R : p_bmm050->dig_xyz1))) -
			((BMM050_U16)0x4000)));
		inter_retval = ((BMM050_S16)((((BMM050_S32)mdata_y) *
				((((((((BMM050_S32)
				       p_bmm050->dig_xy2) *
				      ((((BMM050_S32) inter_retval) *
					((BMM050_S32)inter_retval)) >> 7)) +
				     (((BMM050_S32)inter_retval) *
				      ((BMM050_S32)(((BMM050_S16)
				      p_bmm050->dig_xy1) << 7)))) >> 9) +
				   ((BMM050_S32)0x100000)) *
				  ((BMM050_S32)(((BMM050_S16)p_bmm050->dig_y2)
					  + ((BMM050_S16)0xA0))))
				 >> 12)) >> 13)) +
			(((BMM050_S16)p_bmm050->dig_y1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S32 bmm050_compensate_Y_s32 (BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	BMM050_S32 retval;

	retval = bmm050_compensate_Y(mdata_y, data_R);
	if (retval == BMM050_OVERFLOW_OUTPUT)
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_Y_float(BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_y != BMM050_FLIP_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		if (data_R != 0) {
			inter_retval = ((((float)p_bmm050->dig_xyz1)*16384.0f
			/data_R)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_y * ((((((float)p_bmm050->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval * ((float)p_bmm050->dig_xy1)/16384.0f)) +
			256.0f) * (((float)p_bmm050->dig_y2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm050->dig_y1) * 8.0f))/16.0f;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM050_S16 bmm050_compensate_Z(BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	BMM050_S32 retval;
	if ((mdata_z != BMM050_HALL_OVERFLOW_ADCVAL)	/* no overflow */
	   ) {
		retval = (((((BMM050_S32)(mdata_z - p_bmm050->dig_z4)) << 15) -
					((((BMM050_S32)p_bmm050->dig_z3) *
					  ((BMM050_S32)(((BMM050_S16)data_R) -
						  ((BMM050_S16)
						   p_bmm050->dig_xyz1))))>>2)) /
				(p_bmm050->dig_z2 +
				 ((BMM050_S16)(((((BMM050_S32)
					 p_bmm050->dig_z1) *
					 ((((BMM050_S16)data_R) << 1)))+
						 (1<<15))>>16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMM050_POSITIVE_SATURATION_Z) {
			retval =  BMM050_POSITIVE_SATURATION_Z;
		} else {
			if (retval < BMM050_NEGATIVE_SATURATION_Z)
				retval = BMM050_NEGATIVE_SATURATION_Z;
		}
	} else {
		/* overflow */
		retval = BMM050_OVERFLOW_OUTPUT;
	}
	return (BMM050_S16)retval;
}

BMM050_S32 bmm050_compensate_Z_s32(BMM050_S16 mdata_z, BMM050_U16 data_R)
{
   BMM050_S32 retval;
   if (mdata_z != BMM050_HALL_OVERFLOW_ADCVAL) {
		retval = (((((BMM050_S32)(mdata_z - p_bmm050->dig_z4)) << 15) -
			((((BMM050_S32)p_bmm050->dig_z3) *
			((BMM050_S32)(((BMM050_S16)data_R) -
			((BMM050_S16)p_bmm050->dig_xyz1))))>>2)) /
			(p_bmm050->dig_z2 +
			((BMM050_S16)(((((BMM050_S32)p_bmm050->dig_z1) *
			((((BMM050_S16)data_R) << 1)))+(1<<15))>>16))));
   } else {
      retval = BMM050_OVERFLOW_OUTPUT_S32;
   }
   return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_Z_float (BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_z != BMM050_HALL_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		inter_retval = ((((((float)mdata_z)-((float)p_bmm050->dig_z4))*
		131072.0f)-(((float)p_bmm050->dig_z3)*(((float)data_R)-
		((float)p_bmm050->dig_xyz1))))/((((float)p_bmm050->dig_z2)+
		((float)p_bmm050->dig_z1)*((float)data_R)/32768.0)*4.0))/16.0;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
#endif

BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_x(
		unsigned char enable_disable)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
				BMM050_SENS_CNTL_CHANNELX,
				enable_disable);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_y(
		unsigned char enable_disable)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_SENS_CNTL_CHANNELY,
				enable_disable);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_soft_reset(void)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		v_data_u8r = BMM050_ON;

		comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_SRST7__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
				BMM050_POWER_CNTL_SRST7,
				BMM050_SOFT_RESET7_ON);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_SRST7__REG, &v_data_u8r, 1);

		comres |= p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
				BMM050_POWER_CNTL_SRST1,
				BMM050_SOFT_RESET1_ON);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);

		p_bmm050->delay_msec(BMM050_DELAY_SOFTRESET);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_raw_xyz(struct bmm050_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[6];
	if (p_bmm050 == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
				BMM050_DATAX_LSB, a_data_u8r, 6);

		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
				BMM050_DATAX_LSB_VALUEX);
		mdata->datax = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[1]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[0]);

		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
				BMM050_DATAY_LSB_VALUEY);
		mdata->datay = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[3]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[2]);

		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
				BMM050_DATAZ_LSB_VALUEZ);
		mdata->dataz = (BMM050_S16)((((BMM050_S16)
						((signed char)a_data_u8r[5]))
					<< SHIFT_LEFT_7_POSITION)
				| a_data_u8r[4]);
	}
	return comres;
}


/* sensor specific */
#define SENSOR_NAME "bmm050"

#define SENSOR_CHIP_ID_BMM (0x32)

#define BMM_REG_NAME(name) BMM050_##name
#define BMM_VAL_NAME(name) BMM050_##name
#define BMM_CALL_API(name) bmm050_##name

#define BMM_I2C_WRITE_DELAY_TIME 1

#define BMM_DEFAULT_REPETITION_XY BMM_VAL_NAME(REGULAR_REPXY)
#define BMM_DEFAULT_REPETITION_Z BMM_VAL_NAME(REGULAR_REPZ)
#define BMM_DEFAULT_ODR BMM_VAL_NAME(REGULAR_DR)
/* generic */
#define BMM_MAX_RETRY_I2C_XFER (100)
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_MAX_RETRY_WAIT_DRDY (100)

#define BMM_DELAY_MIN (1)
#define BMM_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMM_SELF_TEST 1
#define BMM_ADV_TEST 2

#define BMM_OP_MODE_UNKNOWN (-1)

struct op_mode_map {
	char *op_mode_name;
	long op_mode;
};

#ifdef CONFIG_MACH_LGE
	/*LGE_CHANGE : 2012-10-08 Jiyeon.park(jiyeon.park@lge.com)
	* sensor for diag bmm050
	*/
static atomic_t bmm_diag = ATOMIC_INIT(0);
static atomic_t bmm_cnt = ATOMIC_INIT(0);
#endif

static const u8 odr_map[] = {10, 2, 6, 8, 15, 20, 25, 30};
static const struct op_mode_map op_mode_maps[] = {
	{"normal", BMM_VAL_NAME(NORMAL_MODE)},
	{"forced", BMM_VAL_NAME(FORCED_MODE)},
	{"suspend", BMM_VAL_NAME(SUSPEND_MODE)},
	{"sleep", BMM_VAL_NAME(SLEEP_MODE)},
};


struct bmm_client_data {
	struct bmm050 device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;

	struct bmm050_mdata value;
	u8 enable:1;
	s8 op_mode:4;
	u8 odr;
	u8 rept_xy;
	u8 rept_z;

	s16 result_test;

	struct mutex mutex_power_mode;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_odr;
	struct mutex mutex_rept_xy;
	struct mutex mutex_rept_z;

	struct mutex mutex_value;
};

static struct i2c_client *bmm_client;
/* i2c operation for API */
static void bmm_delay(u32 msec);
static char bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static char bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmm_dump_reg(struct i2c_client *client);
static int bmm_wakeup(struct i2c_client *client);
static int bmm_check_chip_id(struct i2c_client *client);

static int bmm_pre_suspend(struct i2c_client *client);
static int bmm_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler);
static void bmm_late_resume(struct early_suspend *handler);
#endif

static int bmm_restore_hw_cfg(struct i2c_client *client);

static int bmm_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), &chip_id, 1);
	PINFO("read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM)
		err = -1;

	return err;
}

static void bmm_delay(u32 msec)
{
	mdelay(msec);
}

static void bmm_dump_reg(struct i2c_client *client)
{
	int i;
#ifdef CONFIG_MACH_LGE
//LGE_CHANGE : 2012-11-09 Sanghun,Lee(eee3114.@lge.com)wbt sanity check
//wbt 412964 	'dbg_buf' array elements are used uninitialized in this function with index range: [16,63].
	u8 dbg_buf[64] = "";
#else
	u8 dbg_buf[64];
#endif
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);
}

static int bmm_wakeup(struct i2c_client *client)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	const u8 value = 0x01;
	u8 dummy;

	PINFO("waking up the chip...");

	while (try_times) {
		err = bmm_i2c_write(client,
				BMM_REG_NAME(POWER_CNTL), (u8 *)&value, 1);
		mdelay(BMM_I2C_WRITE_DELAY_TIME);
		dummy = 0;
		err = bmm_i2c_read(client, BMM_REG_NAME(POWER_CNTL), &dummy, 1);
		if (value == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}

/*	i2c read routine for API*/
static char bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			PERR("i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -1;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
	}

	if (BMM_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

/*	i2c write routine for */
static char bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMM_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMM_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMM_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			PERR("error writing i2c bus");
			return -1;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMM_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(BMM_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMM_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static char bmm_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = bmm_i2c_read(bmm_client, reg_addr, data, len);
	return err;
}

static char bmm_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = bmm_i2c_write(bmm_client, reg_addr, data, len);
	return err;
}

/* this function exists for optimization of speed,
 * because it is frequently called */
static inline int bmm_set_forced_mode(struct i2c_client *client)
{
	int err = 0;

	/* FORCED_MODE */
	const u8 value = 0x02;
	err = bmm_i2c_write(client, BMM_REG_NAME(CONTROL), (u8 *)&value, 1);

	return err;
}

static void bmm_work_func(struct work_struct *work)
{
	struct bmm_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmm_client_data, work);
	struct i2c_client *client = client_data->client;
	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));

	mutex_lock(&client_data->mutex_value);

	mutex_lock(&client_data->mutex_op_mode);
	if (BMM_VAL_NAME(NORMAL_MODE) != client_data->op_mode)
		bmm_set_forced_mode(client);

	mutex_unlock(&client_data->mutex_op_mode);

	BMM_CALL_API(read_mdataXYZ)(&client_data->value);

	input_report_abs(client_data->input, ABS_X, client_data->value.datax);
	input_report_abs(client_data->input, ABS_Y, client_data->value.datay);
	input_report_abs(client_data->input, ABS_Z, client_data->value.dataz);
	mutex_unlock(&client_data->mutex_value);

	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}

static int bmm_set_odr(struct i2c_client *client, u8 odr)
{
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(odr_map); i++) {
		if (odr_map[i] == odr)
			break;
	}

	if (ARRAY_SIZE(odr_map) == i) {
		err = -1;
		return err;
	}

	err = BMM_CALL_API(set_datarate)(i);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);

	return err;
}

static int bmm_get_odr(struct i2c_client *client, u8 *podr)
{
	int err = 0;
	u8 value;

	err = BMM_CALL_API(get_datarate)(&value);
	if (!err)
		*podr = value;

	return err;
}

static ssize_t bmm_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMM);
}

static ssize_t bmm_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_op_mode);
		BMM_CALL_API(get_functional_state)(&op_mode);
		mutex_unlock(&client_data->mutex_op_mode);
	} else {
		op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	}

	mutex_unlock(&client_data->mutex_power_mode);

	PDEBUG("op_mode: %d", op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}


static inline int bmm_get_op_mode_idx(u8 op_mode)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(op_mode_maps); i++) {
		if (op_mode_maps[i].op_mode == op_mode)
			break;
	}

	if (i < ARRAY_SIZE(op_mode_maps))
		return i;
	else
		return -1;
}


static ssize_t bmm_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err = 0;
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	long op_mode;

	err = strict_strtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_power_mode);

	i = bmm_get_op_mode_idx(op_mode);

	if (i != -1) {
		mutex_lock(&client_data->mutex_op_mode);
		if (op_mode != client_data->op_mode) {
			if (BMM_VAL_NAME(FORCED_MODE) == op_mode) {
				/* special treat of forced mode
				 * for optimization */
				err = bmm_set_forced_mode(client);
			} else {
				err = BMM_CALL_API(set_functional_state)(
						op_mode);
			}

			if (!err) {
				if (BMM_VAL_NAME(FORCED_MODE) == op_mode)
					client_data->op_mode =
						BMM_OP_MODE_UNKNOWN;
				else
					client_data->op_mode = op_mode;
			}
		}
		mutex_unlock(&client_data->mutex_op_mode);
	} else {
		err = -EINVAL;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmm_show_odr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_odr);
		err = bmm_get_odr(client, &data);
		mutex_unlock(&client_data->mutex_odr);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (!err) {
		if (data < ARRAY_SIZE(odr_map))
			err = sprintf(buf, "%d\n", odr_map[data]);
		else
			err = -EINVAL;
	}

	return err;
}

static ssize_t bmm_store_odr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	unsigned char data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	u8 power_mode;
	int i;

	err = strict_strtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		for (i = 0; i < ARRAY_SIZE(odr_map); i++) {
			if (odr_map[i] == data)
				break;
		}

		if (i < ARRAY_SIZE(odr_map)) {
			mutex_lock(&client_data->mutex_odr);
			err = bmm_set_odr(client, i);
			if (!err)
				client_data->odr = i;

			mutex_unlock(&client_data->mutex_odr);
		} else {
			err = -EINVAL;
		}
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);
	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_xy(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_xy);
		err = BMM_CALL_API(get_repetitions_XY)(&data);
		mutex_unlock(&client_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_xy(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = strict_strtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_xy);
		err = BMM_CALL_API(set_repetitions_XY)(data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_xy = data;
		}
		mutex_unlock(&client_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_z);
		err = BMM_CALL_API(get_repetitions_Z)(&data);
		mutex_unlock(&client_data->mutex_rept_z);
	} else {
		err = -EIO;
	}

	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = strict_strtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&client_data->mutex_rept_z);
		err = BMM_CALL_API(set_repetitions_Z)(data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_z = data;
		}
		mutex_unlock(&client_data->mutex_rept_z);
	} else {
		err = -EIO;
	}
	mutex_unlock(&client_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}


static ssize_t bmm_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int count;

	BMM_CALL_API(read_mdataXYZ)(&client_data->value);

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return count;
}


static ssize_t bmm_show_value_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bmm050_mdata value;
	int count;

	BMM_CALL_API(get_raw_xyz)(&value);

	count = sprintf(buf, "%hd %hd %hd\n",
			value.datax,
			value.datay,
			value.dataz);

	return count;
}


static ssize_t bmm_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = sprintf(buf, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmm_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bmm_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmm_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMM_DELAY_MIN)
		data = BMM_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}

static ssize_t bmm_show_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int err;

	err = sprintf(buf, "%d\n", client_data->result_test);
	return err;
}

static ssize_t bmm_store_test(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	struct i2c_client *client = client_data->client;
	u8 dummy;

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	/* the following code assumes the work thread is not running */
	if (BMM_SELF_TEST == data) {
		/* self test */
		err = BMM_CALL_API(set_functional_state)(
				BMM_VAL_NAME(SLEEP_MODE));
		mdelay(3);
		err = BMM_CALL_API(set_selftest)(1);
		mdelay(3);
		err = BMM_CALL_API(get_self_test_XYZ)(&dummy);
		client_data->result_test = dummy;
	} else if (BMM_ADV_TEST == data) {
		/* advanced self test */
		err = BMM_CALL_API(perform_advanced_selftest)(
				&client_data->result_test);
	} else {
		err = -EINVAL;
	}

	if (!err) {
		BMM_CALL_API(soft_reset)();
		mdelay(BMM_I2C_WRITE_DELAY_TIME);
		bmm_restore_hw_cfg(client);
	}

	if (err)
		count = -1;

	return count;
}

#ifdef CONFIG_MACH_LGE
	/*LGE_CHANGE : 2012-10-08 Jiyeon.park(jiyeon.park@lge.com)
	* sensor for diag bmm050
	*/
static ssize_t bmm_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int count;
	if (atomic_read(&bmm_diag) == 1)
		 atomic_inc(&bmm_cnt);
	BMM_CALL_API(read_mdataXYZ)(&client_data->value);

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return sprintf(buf, "%d\n", client_data->value.datax);

}
static ssize_t bmm_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int count;
	if (atomic_read(&bmm_diag) == 1)
		 atomic_inc(&bmm_cnt);

	BMM_CALL_API(read_mdataXYZ)(&client_data->value);

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return sprintf(buf, "%d\n", client_data->value.datay);
}
static ssize_t bmm_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_client_data *client_data = input_get_drvdata(input);
	int count;
	if (atomic_read(&bmm_diag) == 1)
		 atomic_inc(&bmm_cnt);
	BMM_CALL_API(read_mdataXYZ)(&client_data->value);

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return sprintf(buf, "%d\n", client_data->value.dataz);
}

static ssize_t bmm_diag_cnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&bmm_cnt));
}

static ssize_t bmm_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&bmm_diag));
}
static ssize_t bmm_diag_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if(value == 1){
		atomic_set(&bmm_diag,1);
	}else{
		atomic_set(&bmm_diag,0);
	}
	return count;
}

static DEVICE_ATTR(x, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_x_show, NULL);
static DEVICE_ATTR(y, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_y_show, NULL);
static DEVICE_ATTR(z, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_z_show, NULL);
static DEVICE_ATTR(cnt, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_diag_cnt_show, NULL);
static DEVICE_ATTR(diag, S_IRUGO|S_IWUSR|S_IWGRP,
		bmm_diag_show, bmm_diag_store);
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		bmm_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		bmm_show_op_mode, bmm_store_op_mode);
static DEVICE_ATTR(odr, S_IRUGO|S_IWUSR,
		bmm_show_odr, bmm_store_odr);
static DEVICE_ATTR(rept_xy, S_IRUGO|S_IWUSR,
		bmm_show_rept_xy, bmm_store_rept_xy);
static DEVICE_ATTR(rept_z, S_IRUGO|S_IWUSR,
		bmm_show_rept_z, bmm_store_rept_z);
static DEVICE_ATTR(value, S_IRUGO,
		bmm_show_value, NULL);
static DEVICE_ATTR(value_raw, S_IRUGO,
		bmm_show_value_raw, NULL);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		bmm_show_enable, bmm_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		bmm_show_delay, bmm_store_delay);
static DEVICE_ATTR(test, S_IRUGO|S_IWUSR,
		bmm_show_test, bmm_store_test);

static struct attribute *bmm_attributes[] = {
#ifdef CONFIG_MACH_LGE
	/*LGE_CHANGE : 2012-10-08 Jiyeon.park(jiyeon.park@lge.com)
	* sensor for diag bmm050
	*/
	&dev_attr_x.attr,
	&dev_attr_y.attr,
	&dev_attr_z.attr,
	&dev_attr_diag.attr,
	&dev_attr_cnt.attr,
#endif
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_odr.attr,
	&dev_attr_rept_xy.attr,
	&dev_attr_rept_z.attr,
	&dev_attr_value.attr,
	&dev_attr_value_raw.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_test.attr,
	NULL
};


static struct attribute_group bmm_attribute_group = {
	.attrs = bmm_attributes
};


static int bmm_input_init(struct bmm_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmm_input_destroy(struct bmm_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmm_restore_hw_cfg(struct i2c_client *client)
{
	int err = 0;
#ifdef CONFIG_MACH_LGE
	//LGE_CHANGE : 2012-11-09 Sanghun,Lee(eee3114.@lge.com)wbt sanity check
	//wbt 412963	'value' is used uninitialized in this function.	
	u8 value = 0;
#else
	u8 value;
#endif
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	int op_mode;

	mutex_lock(&client_data->mutex_op_mode);
	err = BMM_CALL_API(set_functional_state)(BMM_VAL_NAME(SLEEP_MODE));

	if (bmm_get_op_mode_idx(client_data->op_mode) != -1)
		err = BMM_CALL_API(set_functional_state)(client_data->op_mode);

	op_mode = client_data->op_mode;
	mutex_unlock(&client_data->mutex_op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		return err;

	PINFO("app did not close this sensor before suspend");

	mutex_lock(&client_data->mutex_odr);
	BMM_CALL_API(set_datarate)(client_data->odr);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	mutex_unlock(&client_data->mutex_odr);

	mutex_lock(&client_data->mutex_rept_xy);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_XY),
			&client_data->rept_xy, 1);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_XY), &value, 1);
	PINFO("BMM_NO_REPETITIONS_XY: %02x", value);
	mutex_unlock(&client_data->mutex_rept_xy);

	mutex_lock(&client_data->mutex_rept_z);
	err = bmm_i2c_write(client, BMM_REG_NAME(NO_REPETITIONS_Z),
			&client_data->rept_z, 1);
	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm_i2c_read(client, BMM_REG_NAME(NO_REPETITIONS_Z), &value, 1);
	PINFO("BMM_NO_REPETITIONS_Z: %02x", value);
	mutex_unlock(&client_data->mutex_rept_z);

	PINFO("register dump after init");
	bmm_dump_reg(client);

	return err;
}

static int bmm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmm_client_data *client_data = NULL;
	int dummy;

	PINFO("function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == bmm_client) {
		bmm_client = client;
	} else {
		PERR("this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err_clean;
	}

	/* wake up the chip */
	dummy = bmm_wakeup(client);
	if (dummy < 0) {
		PERR("Cannot wake up %s, I2C xfer error", SENSOR_NAME);
		err = -EIO;
		goto exit_err_clean;
	}

	PINFO("register dump after waking up");
	bmm_dump_reg(client);
	/* check chip id */
	err = bmm_check_chip_id(client);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bmm_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	mutex_init(&client_data->mutex_power_mode);
	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);
	mutex_init(&client_data->mutex_odr);
	mutex_init(&client_data->mutex_rept_xy);
	mutex_init(&client_data->mutex_rept_z);
	mutex_init(&client_data->mutex_value);

	/* input device init */
	err = bmm_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmm_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmm_work_func);
	atomic_set(&client_data->delay, BMM_DELAY_DEFAULT);

	/* h/w init */
	client_data->device.bus_read = bmm_i2c_read_wrapper;
	client_data->device.bus_write = bmm_i2c_write_wrapper;
	client_data->device.delay_msec = bmm_delay;
	BMM_CALL_API(init)(&client_data->device);

	bmm_dump_reg(client);

	PDEBUG("trimming_reg x1: %d y1: %d x2: %d y2: %d xy1: %d xy2: %d",
			client_data->device.dig_x1,
			client_data->device.dig_y1,
			client_data->device.dig_x2,
			client_data->device.dig_y2,
			client_data->device.dig_xy1,
			client_data->device.dig_xy2);

	PDEBUG("trimming_reg z1: %d z2: %d z3: %d z4: %d xyz1: %d",
			client_data->device.dig_z1,
			client_data->device.dig_z2,
			client_data->device.dig_z3,
			client_data->device.dig_z4,
			client_data->device.dig_xyz1);

	client_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	client_data->op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	client_data->odr = BMM_DEFAULT_ODR;
	client_data->rept_xy = BMM_DEFAULT_REPETITION_XY;
	client_data->rept_z = BMM_DEFAULT_REPETITION_Z;


#if 0
	err = bmm_restore_hw_cfg(client);
#else
	err = BMM_CALL_API(set_functional_state)(
			BMM_VAL_NAME(SUSPEND_MODE));
	if (err) {
		PERR("fail to init h/w of %s", SENSOR_NAME);
		err = -EIO;
		goto exit_err_sysfs;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	client_data->early_suspend_handler.suspend = bmm_early_suspend;
	client_data->early_suspend_handler.resume = bmm_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

	PNOTICE("sensor %s probed successfully", SENSOR_NAME);

	PDEBUG("i2c_client: %p client_data: %p i2c_device: %p input: %p",
			client, client_data, &client->dev, client_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmm_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		bmm_client = NULL;
	}

	return err;
}

static int bmm_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PDEBUG("cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

static int bmm_post_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmm_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)container_of(handler,
			struct bmm_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;
	u8 power_mode;
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		err = bmm_pre_suspend(client);
		err = BMM_CALL_API(set_functional_state)(
				BMM_VAL_NAME(SUSPEND_MODE));
	}
	mutex_unlock(&client_data->mutex_power_mode);

}

static void bmm_late_resume(struct early_suspend *handler)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)container_of(handler,
			struct bmm_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;
	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);

	err = bmm_restore_hw_cfg(client);
	/* post resume operation */
	bmm_post_resume(client);

	mutex_unlock(&client_data->mutex_power_mode);
}
#else
static int bmm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);
	u8 power_mode;

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		err = bmm_pre_suspend(client);
		err = BMM_CALL_API(set_functional_state)(
				BMM_VAL_NAME(SUSPEND_MODE));
	}
	mutex_unlock(&client_data->mutex_power_mode);

	return err;
}

static int bmm_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_power_mode);
	err = bmm_restore_hw_cfg(client);
	/* post resume operation */
	bmm_post_resume(client);

	mutex_unlock(&client_data->mutex_power_mode);

	return err;
}
#endif

static int bmm_remove(struct i2c_client *client)
{
	int err = 0;
	struct bmm_client_data *client_data =
		(struct bmm_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif

		mutex_lock(&client_data->mutex_op_mode);
		if (BMM_VAL_NAME(NORMAL_MODE) == client_data->op_mode) {
			cancel_delayed_work_sync(&client_data->work);
			PDEBUG("cancel work");
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMM_CALL_API(set_functional_state)(
				BMM_VAL_NAME(SUSPEND_MODE));
		mdelay(BMM_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bmm_attribute_group);
		bmm_input_destroy(client_data);
		kfree(client_data);

		bmm_client = NULL;
	}

	return err;
}

static const struct i2c_device_id bmm_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmm_id);

static struct i2c_driver bmm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmm_id,
	.probe = bmm_probe,
	.remove = bmm_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmm_suspend,
	.resume = bmm_resume,
#endif
};

static int __init BMM_init(void)
{
	return i2c_add_driver(&bmm_driver);
}

static void __exit BMM_exit(void)
{
	i2c_del_driver(&bmm_driver);
}

MODULE_AUTHOR("Zhengguang.Guo <Zhengguang.Guo@bosch-sensortec.com>");
MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL");

module_init(BMM_init);
module_exit(BMM_exit);

