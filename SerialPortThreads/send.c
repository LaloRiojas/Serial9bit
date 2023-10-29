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
void* SendingThread(int fd){
    char message[100];
    
    while(portAvailable){
        printf("Enter message to send: ");
        scanf("%s", message);
        clock_t start = clock();
        if(Write_String_9bit(fd, message, 0b0000001)==-1){
            printf("Error writing to port '%s'\n");
            portAvailable = false;
        }
        clock_t end = clock();
        double time_spent = (double)(end - start) / CLOCKS_PER_SEC;
        printf("message sent in %f seconds\n", time_spent);
    }
    
    pthread_exit( NULL);
}