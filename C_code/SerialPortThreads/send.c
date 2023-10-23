#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include "Serial.h"
#include <time.h>

void* SendingThread(void* port){
    port = (char*)port;
    int fd = Setup_Serial_Send(port);
    int errno = Write_String_9bit(fd, "Hello World", 0b0000001);
    
    pthread_exit( NULL);
}