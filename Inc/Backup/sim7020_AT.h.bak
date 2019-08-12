
#define BAUDRATES 15
const long int Baudrates[BAUDRATES] = {0,110,300,1200,2400,4800,9600,19200,38400,57600,115200,230400,460800,921600,3000000};

const char* NBstart[] = {
  "ATZ",
  "AT+CFUN=0",
  "AT+CREG=2",
  "AT*MCGDEFCONT=\"IP\",\"cdp.iot.t-mobile.nl\"",
  "AT+CFUN=1",
  "AT+CBAND=8",
  "AT+COPS=1,2,\"20416\"",  // sign up to T-Moble NL
  "AT+CGCONTRDP",
  "AT+CSQ",
  "\0"  // end script with 0x00
};

const char* NBopensocket[] = {
  "AT+CSOC=1,2,1",
  "AT+CSOCON=0,15683,\"172.27.131.100\"", // T-Mobile Server socket 0
  "\0"     // end script with 0x00
};

const char* NBclosesocket[] = { 
  "AT+CSODIS=0",
  "AT+CSOCL=0",
  "AT+CGACT=0,1",
  "\0"     // end script with 0x00
};

const char* NBhelloworld[] = {
  "AT+CSOSEND=0,0,\"Hello World!\"",
  "\0"     // end script with 0x00
};

const char* NBgetimei[] = {
  "AT+GSN",
  "\0"     // end script with  0x00
};

const char* NBgetmfr[] = {
  "AT+GMI",
  "\0"     // end script with  0x00
};

const char* NBgetcicc[] = {
  "AT+CCID",
  "\0"     // end script with  0x00
};

const char* NBgetmodel[] = {
  "AT+GMM",
  "\0"     // end script with  0x00
};

const char* NBdefault[] = {
  "ATZ",
  "\0"     // end script with  0x00
};

const char* NBat[] = {
  "AT",
  "\0"     // end script with  0x00
};

const char* NBss921600[] = {
  "AT+IPR=921600",
  "\0"     // end script with  0x00
};

const char* NBss0[] = {
  "AT+IPR=0",
  "\0"     // end script with  0x00
};
