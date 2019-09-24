
#ifndef SIM7020_H_
#define SIM7020_H_

#define SIM7020_DEBUG 1

#define TIMEOUT 200
#define COMMANDMAX 1024
#define MAX_CLIENT_ID_LENGTH 12

char getRandomCharNum(uint8_t low, uint8_t high);

void sim7020_init(UART_HandleTypeDef *ctrl, UART_HandleTypeDef *msg);
void sim7020_powerCycle();
void sim7020_dtrWakeup();
void sim7020_dtr(uint8_t state);

uint8_t sim7020_readimei();
uint8_t sim7020_readmfr();
uint8_t sim7020_readcicc();
uint8_t sim7020_readmodel();

void generateRandomClientID();

void sim7020_hardwareInfo();
void sim7020_NBcopsUntillConnect(uint8_t attempts);
void sim7020_NBcmqnewUntillConnect(uint8_t attempts);

uint8_t sim7020_NBcops();
uint8_t sim7020_NBcsq();
uint8_t sim7020_NBcmqnew();
uint8_t sim7020_NBcmqcon();
uint8_t sim7020_NBcmqdiscon();
uint8_t sim7020_NBcmqpub(const char** mqttmsg);
uint8_t sim7020_NBminfun();
uint8_t sim7020_NBmaxfun();
uint8_t sim7020_NBcpin();

int32_t runscript(const char** scrpt);
int32_t writecommand(const char* cmd);

#endif /* SIM7020_H_ */
