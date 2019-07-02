#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "sim7020.h"
#include "sim7020_AT.h"

UART_HandleTypeDef *sim7020_ctrl;
UART_HandleTypeDef *sim7020_msg;

extern unsigned char byte;
extern uint8_t buffer_counter;
extern unsigned char *buffer;

char MyImei[32];
char MyIp[16];
char MyMfr[16];
char MyCicc[24];
char MyModel[16];

int32_t TimeOut = TIMEOUT;


void sim7020Init(UART_HandleTypeDef *ctrl, UART_HandleTypeDef *msg)
{
	sim7020_ctrl = ctrl;
	sim7020_msg = msg;
}

void sim7020PowerCycle()
{
    const char info[] = "Power Circle Start\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(2000);


    const char info1[] = "Power Circle Done\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
}

void sim7020Dtr()
{
	const char info[] = "DTR Start\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOC, SIM_SLP_Pin, GPIO_PIN_RESET);

    const char info1[] = "DTR Done\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
}

uint8_t readimei()
{
	const char info[] = "READ IMEI\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBgetimei);
    if (s != 1)
        return (s);
    else {
        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyImei[n++] = buffer[t++];
        MyImei[n] = 0;

		const char info[] = "MY IMEI: ";
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyImei, strlen(MyImei), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

        return (s);
    }
}

uint8_t readmfr()
{
	const char info[] = "READ MFR\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBgetmfr);
    if (s != 1)
        return (s);
    else {

        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyMfr[n++] = buffer[t++];
        MyMfr[n] = 0;

		const char info[] = "MY MFR: ";
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyMfr, strlen(MyMfr), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

        return (s);
    }
}

uint8_t readcicc()
{
	const char info[] = "READ CICC\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBgetcicc);
    if (s != 1)
        return (s);
    else {

        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyCicc[n++] = buffer[t++];
        MyCicc[n] = 0;

		const char info[] = "MY CICC: ";
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyCicc, strlen(MyCicc), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

        return (s);
    }
}

uint8_t readmodel()
{
	const char info[] = "READ MODEL\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBgetmodel);
    if (s != 1)
        return (s);
    else {

        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyModel[n++] = buffer[t++];
        MyModel[n] = 0;

		const char info[] = "MY MODEL: ";
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyModel, strlen(MyModel), HAL_MAX_DELAY);
	    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

        return (s);
    }
}

int32_t runscript(const char** scrpt)
{

	const char info[] = "RUN SCRIPT \r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    int32_t t = 0;
    int32_t s = 1;
    while (strlen(scrpt[t]) > 1) {
        s = writecommand(scrpt[t]);

        if (s == 1)
            ++t;
        else
            break;
    }
    return (s);
}


int32_t writecommand(const char* cmd)
{
	uint8_t n = 0;

	buffer_counter = 0;

	const char info[] = "WRITE AT CMD\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

	HAL_UART_Transmit(sim7020_ctrl, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Transmit(sim7020_ctrl, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

	HAL_UART_Receive_IT(sim7020_ctrl, &byte, 1);

	while(n <= TimeOut){
		HAL_Delay(10);
		n++;
	}

	if (n >= TimeOut){
		HAL_UART_Transmit(sim7020_msg, (uint8_t*)"TIMEOUT\r\n", 10, HAL_MAX_DELAY);
		char val[5];
		sprintf (val, "%d ", (uint8_t)buffer_counter);
		HAL_UART_Transmit(sim7020_msg, (uint8_t*)val, strlen(val), HAL_MAX_DELAY);
		HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

		if (buffer[buffer_counter - 4] == 'O' && buffer[buffer_counter - 3] == 'K'){
			const char info[] = "GOT AT ANSWER\r\n\n";
		    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

			HAL_UART_Transmit(sim7020_msg, (uint8_t*)buffer, buffer_counter, HAL_MAX_DELAY);
			HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

		    return 1;
		} else {
			const char info[] = "GOT AT ERROR\r\n";
		    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
		    return 0;
		}
	}
	return 0;
}









