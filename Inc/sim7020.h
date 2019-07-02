
#ifndef SIM7020_H_
#define SIM7020_H_

#define TIMEOUT 200
#define COMMANDMAX 1024

void sim7020Init(UART_HandleTypeDef *ctrl, UART_HandleTypeDef *msg);
void sim7020PowerCycle();
void sim7020Dtr();

uint8_t readimei();
uint8_t readmfr();
uint8_t readcicc();
uint8_t readmodel();

int32_t runscript(const char** scrpt);
int32_t writecommand(const char* cmd);

#endif /* SIM7020_H_ */
