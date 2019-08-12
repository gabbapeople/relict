
#include "main.h"
#include "bh1750.h"

I2C_HandleTypeDef 	*bh1750_i2c;	// Handler to I2C interface
bh1750_mode 		Bh1750_Mode;	// Current sensor mode
uint8_t 			Bh1750_Mtreg;	// Current MT register value

BH1750_STATUS BH1750_Init(I2C_HandleTypeDef *hi2c)
{
	bh1750_i2c = hi2c;
	if(BH1750_OK == BH1750_Reset())
	{
		if(BH1750_OK == BH1750_SetMtreg(BH1750_DEFAULT_MTREG)) // Set default value;
			return BH1750_OK;
	}
	return BH1750_ERROR;
}

BH1750_STATUS BH1750_Reset(void)
{
	uint8_t tmp = 0x07;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

BH1750_STATUS BH1750_PowerState(uint8_t PowerOn)
{
	PowerOn = (PowerOn? 1:0);
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &PowerOn, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

BH1750_STATUS BH1750_SetMode(bh1750_mode Mode)
{
	if(!((Mode >> 4) || (Mode >> 5))) return BH1750_ERROR;
	if((Mode & 0x0F) > 3) return BH1750_ERROR;

	Bh1750_Mode = Mode;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &Mode, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

BH1750_STATUS BH1750_SetMtreg(uint8_t Mtreg)
{
	if(Mtreg <= 32 || Mtreg > 254) return BH1750_ERROR;

	Bh1750_Mtreg = Mtreg;

	uint8_t tmp[2];

	tmp[0] = (0x40 | (Mtreg >> 5));
	tmp[1] = (0x60 | (Mtreg & 0x1F));

	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, tmp, 2, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}

BH1750_STATUS BH1750_TriggerManualConversion(void)
{
	if(BH1750_OK == BH1750_SetMode(Bh1750_Mode))
	{
		return BH1750_OK;
	}
	return BH1750_ERROR;
}

BH1750_STATUS BH1750_ReadLight(float *Result)
{
	float result;
	uint8_t tmp[2];

	if(HAL_OK == HAL_I2C_Master_Receive(bh1750_i2c, BH1750_ADDRESS, tmp, 2, 10))
	{
		result = (tmp[0] << 8) | (tmp[1]);

		if(Bh1750_Mtreg != BH1750_DEFAULT_MTREG)
		{
			result *= (float)((uint8_t)BH1750_DEFAULT_MTREG/(float)Bh1750_Mtreg);
		}

		if(Bh1750_Mode == ONETIME_HIGH_RES_MODE_2 || Bh1750_Mode == CONTINUOUS_HIGH_RES_MODE_2)
		{
			result /= 2.0;
		}

		*Result = result / (float)BH1750_CONVERSION_FACTOR;
		return BH1750_OK;
	}
	return BH1750_ERROR;
}
