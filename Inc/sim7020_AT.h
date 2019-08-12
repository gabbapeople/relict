
#define BAUDRATES 15
const long int Baudrates[BAUDRATES] = {0,110,300,1200,2400,4800,9600,19200,38400,57600,115200,230400,460800,921600,3000000};

//

const char* NBcmqnew[] = {
  "AT+CMQNEW=\"52.7.124.212\",\"1883\",12000,100",
  "\0"
};

const char* NBcmqcon[] = {
  "AT+CMQCON=0,3,\"abdsjjdfdsfkslfkdsfkdsfsdfsdfa\",600,0,0,\"Gabbapeople\",\"142bca1da7344c5cb1b189625b9d5bc9\"",
  "\0"
};

const char* NBcmqpub[] = {
   "AT+CMQPUB=0,\"Gabbapeople/feeds/relict.data/\",1,0,0,8,\"31323334\""
  "\0"
};

//

const char* NBcsq[] = {
  "AT+CSQ",
  "\0"
};

const char* NBcpin[] = {
   "AT+CPIN?",
  "\0"
};

const char* NBcreg[] = {
   "AT+CREG?",
  "\0"
};

const char* NBcgreg[] = {
   "AT+CGREG?",
  "\0"
};

const char* NBcops[] = {
   "AT+COPS?",
  "\0"
};

//

const char* NBgetimei[] = {
  "AT+GSN",
  "\0"
};

const char* NBgetmfr[] = {
  "AT+GMI",
  "\0"
};

const char* NBgetcicc[] = {
  "AT+CCID",
  "\0"
};

const char* NBgetmodel[] = {
  "AT+GMM",
  "\0"
};

//

const char* NBdefault[] = {
  "ATZ",
  "\0"
};

const char* NBat[] = {
  "AT",
  "\0"
};

