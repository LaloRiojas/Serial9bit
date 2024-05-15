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

bool portAvailable = true;
void* SendingThread(void* c){
    Serial9BitConfig* config = c;
    int fd = Setup_Serial_Send(config->sendPort,config->baudrate);
    char message[64];
    while(portAvailable){
        printf("Enter message to send max 63 char:");
        scanf("%63s", message);
        printf("add a wakeup bit? [y/n]");
        char wakeup;    
        scanf("%c", &wakeup);
        if(wakeup == 'y'){
            Write_String_9bit(fd,message,1);
        }
        else{
            Write_String_9bit(fd,message,0);
        }
    }
    pthread_exit(NULL);
}