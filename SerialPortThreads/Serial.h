
#include <stdint.h>
//#define DEBUG
enum{NOTSET,READONLYMODE,WRITEONLYMODE,DUALCOMBINEDPORT,DUALSEPERATEMODE};

//CONFIG STRUCT
typedef struct Serial9BitConfig_t{
    char sendPort [50];
    char receivePort[50];
    int mode ;
    int baudrate ;
    int sendFD;
    int receiveFD;
}Serial9BitConfig;

//CONFIG FUNCTIONS
void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit);
int Setup_Serial_Send(const char* port, int speed);
int Setup_Serial_Receive(const char* port, int speed);
int Setup_Serial_SendAndReceive(const char* port, int speed);
bool Setup_Serial_Port(Serial9BitConfig* config);
bool Is_Valid_Baud_Rate(int speed);




//SENDING FUNCTIONS
int Write_Char_9bit(int fd,unsigned char data,bool wakeupbit);
int Write_String_9bit(int fd, char* data,int wakeupbitset);
int Write_Array_9bit(int fd,uint16_t* dataArray,int size);

//RECEIVING FUNCTIONS
typedef struct received_data {
    char letter;
    bool parityerror;
    bool bit9;
} DataFrame_9bit;

int Process_9bit(char* buf,DataFrame_9bit* data ,int size,int fd);

void print_results(char* buf, int size);
void Print_processed_data(DataFrame_9bit* data, int size);

