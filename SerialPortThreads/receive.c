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
void* ReceivingThread(int fd){

    char buf [buffer_size];
    DataFrame_9bit data[buffer_size];
    int messagesize = read(fd,&buf,buffer_size);
    
    Process_9bit(buf,data,messagesize);
    Print_processed_data(data,messagesize);

    pthread_exit( NULL);
}


