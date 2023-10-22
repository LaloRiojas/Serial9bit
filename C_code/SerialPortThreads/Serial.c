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

void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit){
    if(tableInitialized == false){
        Init_Parity_Table(); // make sure the table has been initilaized
    }


    uint8_t odd_even = oddEvenParityTable[*data];
    uint8_t parity_setting;
    if(parity_bit){ // if you want the 9th bit to be 1
        parity_setting = odd_even; // you set the parity setting to be the same as the parity of the data
    }
    else{ // if yout want the 9th bit to be 0
        parity_setting = !odd_even; // you set the parity setting to be the opposite of the parity of the data
    }


    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag|= PARENB;
    if(parity_setting){
        options.c_cflag|= PARODD;
    }
    else{
        options.c_cflag&= ~PARODD;
    }
    tcsetattr(fd, TCSADRAIN ,&options);
    tcdrain(fd);
}
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