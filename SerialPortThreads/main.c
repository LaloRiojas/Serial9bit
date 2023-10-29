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

int supportedSpeeds[] = {9600, 19200, 38400, 57600, 115200};
int supportedSpeedsSize = 5;

void* Remove_Spaces(char* dest, char* src){
    for (int i = 0; i < strlen(src); i++)
    {
        if(src[i] != ' ' && src[i] != '\t' && src[i] != '\n' && src[i] != '\r' ){
            dest[i] = src[i];
        }
    }
}

bool Is_Valid_Baud_Rate(int speed){
    for (int i = 0; i < supportedSpeedsSize; i++)
    {
        if(speed == supportedSpeeds[i]){
            return true;
        }
    }
       
    printf("Baud Rate not supported\n supported Baud Rates are: ");
    for (int i = 0; i < supportedSpeedsSize; i++)
    {
        printf("%d ", supportedSpeeds[i]);
    }
    return false;  
}

void Read_Config_Console(char* Sendport, char* Receiveport, int* speed, int* mode){
    int mode;
    while(1){
        printf("Enter Mode: \n 1. Separate ports for receive and send \n 2. Same port for receive and send\n");
        scanf("%d", mode);
        if(mode == 1 || mode == 2){
            break;
        }
        else 
            printf("Invalid mode, please type 1 or 2\n");
            
    }
    if(mode == 1){
        printf("Enter Sending port: ");
        scanf("%s", Sendport);
        printf("Enter Receiving port: ");
        scanf("%s", Receiveport);

    }
    else{
        printf("Enter Sending and Receiving port: ");
        scanf("%s", Sendport);
        strcpy(Receiveport, Sendport);

    }
    while(1){
        printf("Enter Baud Rate: ");
        scanf("%d", speed);
        if(Is_Valid_Baud_Rate(*speed)){
            break;
        }
    }

    
}
bool Read_Config_File(FILE* file, char* sendPort, char* receivePort, int* speed , int* mode){
    char line[100];
    char trimmedLine[100];
    char* sp,rp,token;
    int baudRate;
    while(fgets(line, 100, file)!=NULL){
        Remove_Spaces(trimmedLine,line);
        token = strtok(line, ":=,->");
        if(strcmp(token, "Mode")==0){
            token = strtok(NULL, " \n\t");
            if(strcmp(token, "Separate")==0){
                *mode = 1;
            }
            else if(strcmp(token, "Combined")==0){
                *mode = 2;
            }
        }
    }
    if(*mode == 0){
        printf("Mode not specified in config file\n");
        return false;
    }



    while(fgets(line, 100, file)!=NULL){
        Remove_Spaces(trimmedLine,line);
        token = strtok(line, ":=,->");
        if(*mode == 1){
            if(strcmp(token, "Send Port")==0){
                token = strtok(NULL, " \n\t");
                sp = token;
            }
            else if(strcmp(token, "Receive Port")==0){
                token = strtok(NULL, " \n\t");
                rp = token;
            }
        }
        else if(*mode == 2){
            if(strcmp(token, "Combined Port")==0){
                token = strtok(NULL, " \n\t");
                sp = token;
                rp = token;
            }
        }
        if(strcmp(token, "Baud Rate")==0){
            token = strtok(NULL, " \n\t");
            baudRate = atoi(token);
        }
    }
    strcpy(sendPort, sp);
    strcpy(receivePort, rp);
    if(Is_Valid_Baud_Rate(baudRate))
        *speed = baudRate;
    else
        return false;
}


pthread_t SendThread, ReceiveThread;
int main(int argc,char* argv[]) {
    const char Sendport [30] ;
    const char Receiveport [30];
    const char sendAndReceivePort [30];
    uint8_t mode;
    int speed;
    FILE* configFile;

    if(argc <2 ){
        printf("No config file specified\n");
        Read_Config_Console(Sendport, Receiveport, &speed, &mode);
    }
    else{
        if(( configFile =fopen(argv[1], "r"))!=NULL){
            if(!Read_Config_File(configFile, Sendport, Receiveport, &speed, &mode));
                Read_Config_Console(Sendport, Receiveport, &speed, &mode);
        }
        else{
            Read_Config_Console(Sendport, Receiveport, &speed, &mode);
        }
    }

    if(mode == 1){ // separate port mode
        int sendFD = Setup_Serial_Send(Sendport, speed);
        int receiveFD = Setup_Serial_Receive(Receiveport, speed);

        if(pthread_create(&SendThread, NULL, SendingThread,sendFD ) !=0 ){
            printf("error creating SendThread\n");
            return 1;
        }
        if( pthread_create(&ReceiveThread, NULL, ReceivingThread,receiveFD) !=0){
            printf("error creating SendThread\n");
            return 1;
        }

    }
    else if(mode == 2){ // combined port mode
        int sendAndReceiveFD = Setup_Serial_SendAndReceive(sendAndReceivePort, speed);
        if(pthread_create(&SendThread, NULL, SendingThread,sendAndReceiveFD ) !=0 ){
            printf("error creating SendThread\n");
            return 1;
        }
        if( pthread_create(&ReceiveThread, NULL, ReceivingThread,sendAndReceiveFD) !=0){
            printf("error creating SendThread\n");
            return 1;
        }
    }
    else{
        printf("Invalid mode\n");
        return 1;
    }


    pthread_join(SendThread, NULL);
    pthread_join(ReceiveThread, NULL);

    printf("threads done");
    return 0;
}

