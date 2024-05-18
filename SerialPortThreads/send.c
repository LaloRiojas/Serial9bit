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

//reads amount-1 chars from STDIN and does not mess up with whitespace
bool readSTDIN (char* buff, int amount){
        if (fgets(buff, amount , stdin) != NULL) {
            // Remove the newline character if present
            size_t len = strlen(buff);
            if (len > 0 && buff[len - 1] == '\n') {
                buff[len - 1] = '\0';
            }
            return true;
        }
        return false;
}

bool portAvailable = true;
void* SendingThread(void* c){
    Serial9BitConfig* config = c;
    int fd = config->sendFD;
    char message[65];
    while(portAvailable){
        printf("\nEnter message to send max 64 char:\n");
        if(!readSTDIN(message,65)){//read 64 chars
            printf("ERROR: could not read from STDIN");
        }

        char wakeup[65];

        //ask if they would like a wakeup bit
        printf("\nWould you like to add a wakeup bit to the first byte. [y/n]\n");
        printf("Otherwise you can enter a series of 1 and 0 for the 9thbit of each char starting at the first char.\n");
        printf("EXAMPLE: 111001 sends 9thbit=1 on the first 3 chars and char 6 and 9thbit=0 otherwise\n");
        while(true){
            if(!readSTDIN(wakeup,65)){
                printf("ERROR: could not read from STDIN");
            }
            
            if(*wakeup == 'y' || *wakeup == 'Y'){
                Write_String_9bit(fd,message,1);
                break;
            }
            else if (*wakeup == 'n' || *wakeup == 'N'){
                Write_String_9bit(fd,message,0);
                break;
            }
            else if(*wakeup == '1' || *wakeup == '0'){
                uint64_t bitSet =0;
                
                for(int i=0;i<64 && wakeup[i]!='\0';i++){
                    // only take into account 1s dont care about nothing else
                    if(wakeup[i]=='1'){
                        bitSet |= 1<<i;
                    }
                    // dont need to clear since the bitset init at 0
                    // else if(wakeup[i]=='0'){
                    //    bitSet &= ~(1<<i);
                    // }
                }
                Write_String_9bit(fd,message,bitSet);
                break;
            }
            else{
                printf("expected a y or n or bitset\n");
            }
        }
    }
    pthread_exit(NULL);
}