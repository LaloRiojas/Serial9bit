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



bool check9bit( int fd, unsigned char* data){ // data will always be the last real byte sent. it will never be the 0xff or 0x00 byte that comes from the parity bit.
    read(fd,data,sizeof(*data));
    printf("%c",*data);
    if(*data ==0xff){//catch byte
        read(fd,data,sizeof(*data));
        printf("%c",*data);
        if(*data = 0x00){//parity bit set.
            read(fd,data,sizeof(*data));// read the next bit. this is the real data
            printf("%c",*data);
            return true;
        }
        else if (*data ==0xff){ // the real data was 0xff. the stream was 0xff/0xff
            return false;
        }
        else{
            printf("error in check9bit");
            return false;
        }

    }
    return false;
}

void* ReceivingThread(void* port){

    printf("ReceivingThread started\n");
    port = (char*)port;
    int fd;
    fd = Setup_Serial_Receive(port);

    int count = 0;
    unsigned char data;
    printf("\n\n");
    while (count <10){
        read(fd,&data,1);
        printf("%c",data);
        count++;
    }
    printf("\n\n");

    pthread_exit( NULL);
}