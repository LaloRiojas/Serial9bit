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
#include "receive.h"

#define buffer_size 30
void* ReceivingThread(void* c){

    Serial9BitConfig* config = c;
    int fd = config->receiveFD;
    printf("Reading 9 bit data\n");

    char buf [buffer_size];
    DataFrame_9bit data[buffer_size];

    while(true){
        //read the data
        int messagesize = read(fd,&buf,buffer_size);
        //process the data
        int processedSize = Process_9bit(buf,data,messagesize,config->receiveFD);
        printf("received %d bytes",processedSize);
        //print the data
        Print_processed_data(data,processedSize);
    }
}


