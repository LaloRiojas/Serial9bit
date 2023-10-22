#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include "send.h"
#include "receive.h"
#include "Serial.h"


pthread_t SendThread, ReceiveThread;
int main() {
    printf("starting program\n");
    const char *Sendport = "/dev/ttyS0";
    const char *Receiveport = "/dev/ttyS1";

    if(pthread_create(&SendThread, NULL, SendingThread, (void*)Sendport) !=0 ){
        printf("error creating SendThread\n");
        return 1;
    }
    printf("Send Thread created\n");

    if( pthread_create(&ReceiveThread, NULL, ReceivingThread, (void*)Receiveport) !=0){
        printf("error creating SendThread\n");
        return 1;
    }

    printf("Receive Thread Created\n");

    pthread_join(SendThread, NULL);
    pthread_join(ReceiveThread, NULL);

    printf("threads done");
    return 0;
}

