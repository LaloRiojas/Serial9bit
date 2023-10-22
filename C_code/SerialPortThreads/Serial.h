
#include <stdint.h>

enum{READ,WRITE,READWRITE};

void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit);
void Init_Parity_Table(void);

int Setup_Serial_Send(const char* port);
int Setup_Serial_Receive(const char* port);
int Setup_Serial_SendAndReceive(const char* port);
int Setup_Serial_Port(const char *port,uint8_t type);