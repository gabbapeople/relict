#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "sim7020.h"
#include "sim7020_AT.h"

UART_HandleTypeDef* sim7020_ctrl;
UART_HandleTypeDef* sim7020_msg;

extern uint8_t rx_data[2];
extern uint8_t ok_flag;
extern char buffer[rx_buffer_size];
extern uint8_t rx_buffer[rx_buffer_size];

char MyImei[32];
char MyIp[16];
char MyMfr[16];
char MyCicc[24];
char MyModel[16];

int32_t TimeOut = TIMEOUT;

void sim7020Init(UART_HandleTypeDef* ctrl, UART_HandleTypeDef* msg)
{
    sim7020_ctrl = ctrl;

#if (SIM7020_DEBUG == 1)
    sim7020_msg = msg;
#endif
}

void sim7020PowerCycle()
{
#if (SIM7020_DEBUG == 1)
    const char info[] = "Power Circle Start\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(2000);

#if (SIM7020_DEBUG == 1)
    const char info1[] = "Power Circle Done\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
#endif
}

void sim7020Dtr()
{
#if (SIM7020_DEBUG == 1)
    const char info[] = "DTR Start\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif
    HAL_GPIO_WritePin(GPIOC, SIM_SLP_Pin, GPIO_PIN_RESET);
#if (SIM7020_DEBUG == 1)
    const char info1[] = "DTR Done\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
#endif
}

uint8_t readimei()
{

#if (SIM7020_DEBUG == 1)
    const char info[] = "READ IMEI\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

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

        return (s);
    }
}

uint8_t readmfr()
{

#if (SIM7020_DEBUG == 1)
    const char info[] = "READ MFR\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

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

        return (s);
    }
}

uint8_t readcicc()
{

#if (SIM7020_DEBUG == 1)
    const char info[] = "READ CICC\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

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

        return (s);
    }
}

uint8_t readmodel()
{

#if (SIM7020_DEBUG == 1)
    const char info[] = "READ MODEL\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

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

        return (s);
    }
}

int32_t runscript(const char** scrpt)
{

#if (SIM7020_DEBUG == 1)
    const char info[] = "RUN SCRIPT \r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

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
    ok_flag = 0;
    for (uint8_t i = 0; i < 100; i++) {
        rx_buffer[i] = 0;
    }

#if (SIM7020_DEBUG == 1)
    const char info[] = "WRITE AT CMD\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    HAL_UART_Transmit(sim7020_ctrl, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_ctrl, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

    HAL_UART_Receive_IT(sim7020_ctrl, rx_data, 1);

    while (n <= TimeOut) {
        if (ok_flag == 1) {

#if (SIM7020_DEBUG == 1)
            HAL_UART_Transmit(sim7020_msg, (uint8_t*)"OK FOUND\r\n", 10, HAL_MAX_DELAY);
#endif
            return 1;
            break;
        }
        else {
            n++;
            HAL_Delay(100);
        }
    }

    if (n >= TimeOut) {

#if (SIM7020_DEBUG == 1)
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)"TIMEOUT\r\n", 10, HAL_MAX_DELAY);
#endif
        return 0;
    }

    return 0;
}

void sim7020HardwareInfo()
{
    readimei();
    readmfr();
    readcicc();
    readmodel();
#if (SIM7020_DEBUG == 1)
    const char info[] = "MY MODEL: ";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyModel, strlen(MyModel), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    const char info1[] = "MY MFR: ";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyMfr, strlen(MyMfr), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    const char info2[] = "MY CICC: ";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info2, strlen(info2), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyCicc, strlen(MyCicc), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    const char info3[] = "MY IMEI: ";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info3, strlen(info3), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyImei, strlen(MyImei), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
#endif
}
