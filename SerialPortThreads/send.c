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
    int fd = config->sendFD;
    char message[64];
    while(portAvailable){
        printf("Enter message to send max 63 char:\n");
        scanf("%63s", message);
        
        char wakeup[4];

        //ask if they would like a wakeup bit
        printf("Would you like to add a wakeup bit to the first byte [y/n]\n");
        while(true){
            scanf("%3s", wakeup);
            
            if(*wakeup == 'y' || *wakeup == 'Y'){
                Write_String_9bit(fd,message,1);
                break;
            }
            else if (*wakeup == 'n' || *wakeup == 'N'){
                Write_String_9bit(fd,message,0);
                break;
            }
            else{
                printf("expected a y or n\n");
            }
        }
    }
    pthread_exit(NULL);
}