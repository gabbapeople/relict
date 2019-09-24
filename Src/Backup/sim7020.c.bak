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

char MyCops[24];
char MyCsq[16];

char MyCmqnew[50];
char MyCmqcon[50];
char MyCmqpub[50];

char clientID[MAX_CLIENT_ID_LENGTH];

int32_t TimeOut = TIMEOUT;

char getRandomCharNum(uint8_t low, uint8_t high){
	return (rand() % (high - low + 1)) + low;
}

void generateRandomClientID(){

#if (SIM7020_DEBUG == 1)
    const char info[] = "GENERATE RANDOM CLIENT ID\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

	for(uint8_t i = 0; i < MAX_CLIENT_ID_LENGTH; i++ ){
		clientID[i] = getRandomCharNum(97,122);
	}
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)clientID, strlen(clientID), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

void sim7020_init(UART_HandleTypeDef* ctrl, UART_HandleTypeDef* msg)
{
    sim7020_ctrl = ctrl;

#if (SIM7020_DEBUG == 1)
    sim7020_msg = msg;
#endif
}

void sim7020_powerCycle()
{
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 START POWER CIRCLE\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(700);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_RESET);
    HAL_Delay(700);
    HAL_GPIO_WritePin(GPIOB, SIM_PWR_Pin, GPIO_PIN_SET);

#if (SIM7020_DEBUG == 1)
    const char info1[] = "SIM7020 POWER CIRCLE DONE\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info1, strlen(info1), HAL_MAX_DELAY);
#endif
}

void sim7020_dtr(uint8_t state)
{
	if (state == 1){
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 DTR HIGH\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif
    HAL_GPIO_WritePin(GPIOC, SIM_SLP_Pin, GPIO_PIN_SET);
	}

	if (state == 0){
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 DTR LOW\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif
    HAL_GPIO_WritePin(GPIOC, SIM_SLP_Pin, GPIO_PIN_RESET);
	}

}

uint8_t sim7020_readimei()
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

uint8_t sim7020_readmfr()
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

uint8_t sim7020_readcicc()
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

uint8_t sim7020_readmodel()
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

uint8_t sim7020_NBcops(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "CHECK PROVIDER\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBcops);
    if (s != 1)
        return (s);
    else {

        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyCops[n++] = buffer[t++];
        MyCops[n] = 0;

        return (s);
    }
}

uint8_t sim7020_NBcsq(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "CHECK SINGNAL LEVEL\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBcsq);
    if (s != 1)
        return (s);
    else {
        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyCsq[n++] = buffer[t++];
        MyCsq[n] = 0;

        return (s);
    }
}

uint8_t sim7020_NBcmqnew(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "MQTT NEW CONNECTION\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBcmqnew);
    if (s != 1)
        return (s);
    else {
        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
            MyCmqnew[n++] = buffer[t++];
        MyCmqnew[n] = 0;

        return (s);
    }
}

uint8_t sim7020_NBcmqcon(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "MQTT CONNECT\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    char _NBcmqcon[100];

    char* atStringWrapper = "\"";
    char* atComma = ",";
    char* atCmd = "AT+CMQCON=";
    char* atPar0 = "0";
    char* atPar1 = "3";
    char* atPar2 = "600";
    char* atPar3 = "0";
    char* atPar4 = "0";

    strcpy(_NBcmqcon,atCmd);
    strcat(_NBcmqcon,atPar0);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atPar1);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atStringWrapper);
    strcat(_NBcmqcon,clientID);
    strcat(_NBcmqcon,atStringWrapper);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atPar2);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atPar3);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atPar4);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atStringWrapper);
    strcat(_NBcmqcon,NBcmqconLogin);
    strcat(_NBcmqcon,atStringWrapper);
    strcat(_NBcmqcon,atComma);
    strcat(_NBcmqcon,atStringWrapper);
    strcat(_NBcmqcon,NBcmqconPass);
    strcat(_NBcmqcon,atStringWrapper);

    const char* NBcmqcon[] = {
    		_NBcmqcon,
			"\0"
    };

    HAL_UART_Transmit(sim7020_msg, (uint8_t*)_NBcmqcon, strlen(_NBcmqcon), HAL_MAX_DELAY);
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

    int32_t t = 0, n = 0, s = 0;
    s = runscript(NBcmqcon);
    if (s != 1)
        return (s);
    else {
        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
        	MyCmqcon[n++] = buffer[t++];
        MyCmqcon[n] = 0;

        return (s);
    }
}

uint8_t sim7020_NBcmqdiscon(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "MQTT DROP CONNECTION\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t s = 0;
    s = runscript(NBcmqdiscon);
    return (s);
}

uint8_t sim7020_NBminfun(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 MINIMUM FUNCTIONALITY\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t s = 0;
    s = runscript(NBcfun0);
    return (s);
}

uint8_t sim7020_NBmaxfun(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 MAXIMUM FUNCTIONALITY\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t s = 0;
    s = runscript(NBcfun1);
    return (s);
}

uint8_t sim7020_NBcpin(){
#if (SIM7020_DEBUG == 1)
    const char info[] = "SIM7020 CPIN\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t s = 0;
    s = runscript(NBcpin);
    return (s);
}

uint8_t sim7020_NBcmqpub(const char** mqttmsg){
#if (SIM7020_DEBUG == 1)
    const char info[] = "MQTT PUBLISH\r\n";
    HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
#endif

    int32_t t = 0, n = 0, s = 0;
    s = runscript(mqttmsg);
    if (s != 1)
        return (s);
    else {
        t = 0;
        n = 0;
        while (buffer[t++] != '\n')
            ;
        while (buffer[t] != '\n')
        	MyCmqpub[n++] = buffer[t++];
        MyCmqpub[n] = 0;

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

void sim7020_NBcopsUntillConnect(uint8_t attempts){
	uint8_t res;
	_Bool flag = 0;

	for (uint8_t i = 0; i < attempts; i++ ){

		if (flag == 0){
			res = sim7020_NBcops();
			if (res == 1){
				if(strlen(MyCops) > 9){
					flag = 1;
				}
			}
		}

		if(flag == 1){
			const char info[] = "MY COPS: ";
			HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyCops, strlen(MyCops), HAL_MAX_DELAY);
			HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
			HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
			break;
		}
	}
}

void sim7020_NBcmqnewUntillConnect(uint8_t attempts){
	uint8_t res;
	_Bool flag = 0;

	for (uint8_t i = 0; i < attempts; i++ ){

		if (flag == 0){
			res = sim7020_NBcmqnew();
			if (res == 1){
				flag =1;
			}
		}

		if (flag == 1){
			break;
		}
	}
}

void sim7020_hardwareInfo()
{
	uint8_t res[4];
    res[0] = sim7020_readimei();
    res[1] = sim7020_readmfr();
	res[2] = sim7020_readcicc();
	res[3] = sim7020_readmodel();

#if (SIM7020_DEBUG == 1)
    if (res[0] == 1){
        const char info[] = "MY MODEL: ";
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyModel, strlen(MyModel), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[0] == 0){
    	const char info[] = "MY MODEL: ERROR";
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[1] == 1){
        const char info[] = "MY MFR: ";
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyMfr, strlen(MyMfr), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[1] == 0){
    	const char info[] = "MY MFR: ERROR";
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[2] == 1){
        const char info[] = "MY CICC: ";
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyCicc, strlen(MyCicc), HAL_MAX_DELAY);
        HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[2] == 0){
    	const char info[] = "MY CICC: ERROR";
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[3] == 1){
    	const char info[] = "MY IMEI: ";
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)MyImei, strlen(MyImei), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    if (res[3] == 0){
    	const char info[] = "MY IMEI: ERROR";
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
    	HAL_UART_Transmit(sim7020_msg, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
#endif
}


