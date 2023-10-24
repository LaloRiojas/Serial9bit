#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>


#include "Serial.h"

uint8_t oddEvenParityTable [255];
bool tableInitialized = false;
void Init_Parity_Table(void){
    uint8_t count =0;
    for(;count<255;count++ ){
        uint8_t temp = count;
        uint8_t parity = 0;
        while(temp){
            parity = parity ^ (temp & 1) ;
            temp >>= 1;
        }
        oddEvenParityTable[count] = parity & 1; // 1 if odd. 
                                                //0 if even
    }
    tableInitialized = true; 
}
void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit){
    if(tableInitialized == false){
        Init_Parity_Table(); // make sure the table has been initilaized
    }


    bool odd = oddEvenParityTable[*data]; // look up table for the even/odd high bits of the data (only for 8 bit data)


    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag|= PARENB;
    ;

    if((odd&&parity_bit)||(!parity_bit&&!odd)){ 
        options.c_cflag|= PARODD; //odd parity
        #ifdef DEBUG
            printf(" ,parity bit %1d ,oddness %1d ,odd parity\n",parity_bit,odd)
        #endif
    }
    else{
        options.c_cflag&= ~PARODD; //even parity
        #ifdef DEBUG
            printf(" ,parity bit %1d ,oddness %1d ,even parity\n",parity_bit,odd)
        #endif
    }

    
    tcsetattr(fd, TCSADRAIN ,&options);

    tcdrain(fd);
}

uint8_t read_table(uint8_t data){
    if(tableInitialized == false){
        Init_Parity_Table(); // make sure the table has been initilaized
    }
    return oddEvenParityTable[data];
}

int Setup_Serial_Receive(const char* port){
    int fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("Error opening serial port");
        exit(3);
    }

    struct termios options;
    cfsetispeed(&options, B9600);
    tcgetattr(fd, &options);

    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity but will be switched depending on data being sent  when sending

    options.c_iflag |= PARMRK; // mark parity errors
    options.c_iflag &= ~IGNPAR; // dont ignore parity errors
    options.c_iflag |=INPCK; // enable parity checking

    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits

    //local mode (ignore modem control lines)
    options.c_cflag |= CLOCAL;

    //enable the reader
    options.c_cflag |= CREAD;

    //blocks until either 100 bytes are read or 10 deciseconds (1 second) interbyte delay has passed
    options.c_cc[VMIN] = 100;
    options.c_cc[VTIME] = 10;

    //disabling other random stuff that might be on
    options.c_lflag &= ~ ECHO;// no echo
    options.c_lflag &= ~ICANON;// stuff is read byte by byte instead of by line
    options.c_lflag &= ~ECHOE; // no echo
    options.c_lflag &= ~ISIG;// no signalling from ctrl c or ctrl z
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;
    options.c_iflag &= ~IXANY;
    
    tcsetattr(fd, TCSADRAIN,&options);
    tcdrain(fd);
    return fd;

}
int Setup_Serial_Send(const char* port){
    int fd = open(port, O_WRONLY| O_NOCTTY|O_SYNC );
    if (fd == -1) {
        perror("Error opening serial port");
        exit(3);
    }
    struct termios options;
    cfsetospeed(&options, B9600);
    tcgetattr(fd, &options);

    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity but will be switched depending on data being sent  when sending
    
    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits
    //local mode (ignore modem control lines)
    options.c_cflag |= (CLOCAL);

    //disabling other random stuff that might be on
    options.c_lflag &= ~(ICANON | ECHO | ECHOE |ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSADRAIN,&options);
    tcdrain(fd);
    return fd;
    
}
int Setup_Serial_SendAndReceive(const char* port){
        int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("Error opening serial port");
        exit(3);
    }

    struct termios options;
    cfsetispeed(&options, B9600);
    tcgetattr(fd, &options);

    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity but will be switched depending on data being sent  when sending

    options.c_iflag |= PARMRK; // mark parity errors
    options.c_iflag &= ~IGNPAR; // dont ignore parity errors
    options.c_iflag |=INPCK; // enable parity checking

    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits

    //local mode (ignore modem control lines)
    options.c_cflag |= CLOCAL;

    //enable the reader
    options.c_cflag |= CREAD;

    //make readding blocking unitl 1 byte is read
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    //disabling other random stuff that might be on
    options.c_lflag &= ~ ECHO;// no echo
    options.c_lflag &= ~ICANON;// stuff is read byte by byte instead of by line
    options.c_lflag &= ~ECHOE; // no echo
    options.c_lflag &= ~ISIG;// no signalling from ctrl c or ctrl z
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;
    options.c_iflag &= ~IXANY;
    
    tcsetattr(fd, TCSADRAIN,&options);
    tcdrain(fd);
    return fd;

}
int Setup_Serial_Port(const char *port,uint8_t type) {
    if(type == READ){
        return Setup_Serial_Receive(port);
    }
    else if(type == WRITE){
       return  Setup_Serial_Send(port);
    }
    else if(type == READWRITE){
        return Setup_Serial_SendAndReceive(port);
    }
    else{
        printf("error in setupSerialPort type is not READ, WRITE or READWRITE");
        exit(1);
    }

}



//SENDING FUNCTIONS

int Write_Char_9bit(int fd,unsigned char data,bool wakeupbit){
    #ifdef DEBUG
        printf("writing %c/%d",data,data);
    #endif
        Set_Parity_Bit(&data, fd, wakeupbit);
    
    return (write(fd, &data, 1) == -1)? -1:0;
}
// wakeupbitset is a bitset of which characters are going to have the 9th bit high. 
// 0b00000001 or 1 means the first character will have the 9th bit high
// 0b00000011 or 3 means the first and second character will have the 9th bit high
// etc
int Write_String_9bit(int fd, char* data,int wakeupbitset){ 
    int len = strlen(data);
    for (uint32_t i=0 ; i<len; i++){
        int errno =  Write_Char_9bit(fd, data[i],wakeupbitset&1);
        if(errno==-1){
            return -1;
        }
        wakeupbitset = wakeupbitset>>1;
    }
    return 0;
}
int Write_Array_9bit(int fd,uint16_t* dataArray,int size){
    for(int i=0;i<size;i++){
        if(Write_Char_9bit(fd, 0xFF&(dataArray[i]>>1),dataArray[i]&1) == -1){
            return -1;
        }
    }
    
}
// RECEIVING FUNCTIONS
void Process_9bit(char* buf,DataFrame_9bit* data, int size){
    int i,index;
    for (i = 0, index=0; i<size; i++,index++){
        if(buf[i] ==-1 && buf[i+1] == 0 ){
            i+=2;   //skip the wakeup sequence
            data[index].letter = buf[i];
            data[index].parityerror = true;
        }
        else{
            data[index].letter = buf[i];
            data[index].parityerror = false;
        }
        bool odd = read_table(data[index].letter);
        bool par_error = data[index].parityerror;
        //parity bit is on if (even and no error) or (odd and error)
        //parity bit is off if (even and error) or (odd and no error)
        if((odd&&par_error)||!(odd||par_error)){
            data[index].paritybit = true;
        }
        else{
            data[index].paritybit = false;
        }
    }
}
void red () {
  printf("\033[1;31m");
}
void reset () {
  printf("\033[0m");
}
void print_results(char* buf, int size){
    for(int i=0;i<size;i++){
        char c = buf[i];
        if(c==0){
            printf("%d",buf[i]);
        }
        else if(c==-1){
            printf("n");
        }
        else{
            red();
            printf("%c",buf[i]);
            reset();
        }
    }

    printf("\n");
}

void Print_processed_data(DataFrame_9bit* data, int size){
    printf("Parritybit \t Letter \t 9bit data\n");
    for(int i =0;i< size;i++){
        printf( "%1d \t\t %c \t\t %03X\n",data[i].paritybit,data[i].letter,(data[i].letter<<1)|data[i].paritybit);
    }
}

void Encode_9bit_data(DataFrame_9bit* data,uint16_t* result, int size){
    for( int i =0;i<size;i++){
        data[i].paritybit = (data[i].paritybit)? 1:0;
        result[i] = (data[i].letter<<1)|data[i].paritybit;
    }
}

/*
Receiving serial bits:
whenever the linux serial interface reads a byte, it will check for parity errors. The parity setup is set to check even parity within the 8bit data. 
in other words the parity bit (9th bit) his high whenever there is an even number of high bits in the data bits and low when there is an odd number of high bits in the data. 
parity errors are marked when the data does not conform to this standard. 
parity errors are marked by the serial interface by puttin two preceding bytes in the buffer.
the first byte is 0xFF and the second byte is 0x00.
so if you have a parity error on the byte 0x01, the serial interface will put "0xFF 0x00 0x01" in the buffer.

the parity bit cannot be directly read from the serial interface but it can be calculated.
the value of the parity bit can be determined based on whether there was a parity error and the even/odd characteristic of the data bits. 
if the data bits are even then the parity bit would be high if there was no parity error because it conformed to even parity checking. 
if the data bits are odd then the parity bit would be low if there was no parity error because it conformed to even parity checking.
if there was a parity error then this logic is flipped because it did not conform to even parity checking.


thus 
parity = 1 if EVEN AND NO PARITY ERROR or ODD AND PARITY ERROR
parity = 0 if EVEN AND PARITY ERROR or ODD AND NO PARITY ERROR

*/