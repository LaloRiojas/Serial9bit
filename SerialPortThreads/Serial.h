
#include <stdint.h>
//#define DEBUG
enum{READ,WRITE,READWRITE};

void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit);

int Setup_Serial_Send(const char* port);
int Setup_Serial_Receive(const char* port);
int Setup_Serial_SendAndReceive(const char* port);
int Setup_Serial_Port(const char *port,uint8_t type);



//SENDING FUNCTIONS
int Write_Char_9bit(int fd,unsigned char data,bool wakeupbit);
int Write_String_9bit(int fd, char* data,int wakeupbitset);
int Write_Array_9bit(int fd,uint16_t* dataArray,int size);

//RECEIVING FUNCTIONS
typedef struct received_data {
    char letter;
    bool parityerror;
    bool paritybit;
} DataFrame_9bit;

void Process_9bit(char* buf,DataFrame_9bit* data ,int size);
void print_results(char* buf, int size);
void Print_processed_data(DataFrame_9bit* data, int size);
