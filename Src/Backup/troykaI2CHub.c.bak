
#include "main.h"
#include "troykaI2CHub.h"


uint8_t _i2cHubAddr;
I2C_HandleTypeDef 	*troykaI2CHub_i2c;

void troykaI2CHub_init(I2C_HandleTypeDef *hi2c)
{
	troykaI2CHub_i2c = hi2c;
}


_Bool troykaI2CHub_setBusChannel(uint8_t channel)
{
    if (channel >= COUNT_CHANNEL) {
        return 0;
    }

    uint8_t tmp = channel | ENABLE_MASK;
	if(HAL_OK == HAL_I2C_Master_Transmit(troykaI2CHub_i2c, DEFAULT_I2C_HUB_ADDRESS, &tmp, 1, 100))
		return 1;

	return 0;
}
