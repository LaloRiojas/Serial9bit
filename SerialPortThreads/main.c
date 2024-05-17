#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include<bits/getopt_posix.h>

#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include "send.h"
#include "receive.h"
#include "Serial.h"





void* Remove_Spaces(char* dest, char* src){
    for (int i = 0; i < strlen(src); i++)
    {
        if(src[i] != ' ' && src[i] != '\t' && src[i] != '\n' && src[i] != '\r' ){
            dest[i] = src[i];
        }
    }
}


void Print_Usage_Message(){
    printf("4 Modes Available\nw: Write Only\nr: Read Only\nd: Dual Mode with different ports for each\nc: Dual Mode in the same port\n\n");
    printf("Usage: -s -S <send port> -B [POSIX SUPPORTED BAUD RATE] |\n-r -R <receive port> -B [POSIX SUPPORTED BAUD RATE] |\n-d -S <send port> -R <receive port> -B [POSIX SUPPORTED BAUD RATE] |\n-c <dual combined port> -B [POSIX SUPPORTED BAUD RATE]\n");
}

bool Handle_Arguements(int argc, char* argv[],Serial9BitConfig* config){
    int c;
    bool sendPortSet = false, receivePortSet = false, baudRateSet = false;

    while((c = getopt(argc,argv,"hsrdcS:R:B:"))!=-1){
        switch(c){

            //get the mode
            //write only mode
            case 's':
                if(config->mode != NOTSET){
                    printf("Multiple Modes Selected\n");
                    Print_Usage_Message();
                    return false;
                }
                config->mode=WRITEONLYMODE;
                break;

            //read only mode
            case 'r':
                if(config->mode != NOTSET){
                    printf("Multiple Modes Selected\n");
                    Print_Usage_Message();
                    return false;
                }
                config->mode=READONLYMODE;
                break;

            case 'd':
            //dual mode with seperate ports
                if(config->mode != NOTSET){
                    printf("Multiple Modes Selected\n");
                    Print_Usage_Message();
                    return false;
                }
                config->mode=DUALSEPERATEMODE;
                break;
            
            //dual mode with combined ports
            case 'c':
                if(config->mode != NOTSET){
                    printf("Multiple Modes Selected\n");
                    Print_Usage_Message();
                    return false;
                }
                config->mode=DUALCOMBINEDPORT;
                break;
            
            //get the ports and baud rate

            //send port
            case 'S':
                //check if the mode is in read only mode
                if(config->mode == READONLYMODE){
                    printf("Send Port not needed in read only mode\n");
                    Print_Usage_Message();
                    return false;
                }
                //check if the port exists
                if(access(optarg,F_OK)==-1){
                    printf("Port %s not available\n",optarg);
                    return false;
                }
                strcpy(config->sendPort,optarg);
                sendPortSet = true;
                break;
            
            //receive port
            case 'R':
                //check if the mode is in write only mode
                if(config->mode == WRITEONLYMODE){
                    printf("Read Port not needed during write only mode\n");
                    Print_Usage_Message();
                    return false;
                }
                //check if the port exists
                if(access(optarg,F_OK)==-1){
                    printf("Port %s not available\n",optarg);
                    return false;
                }
                strcpy(config->receivePort,optarg);
                receivePortSet = true;
                break;

            //get the baud rate
            case 'B':
                int baud = atoi(optarg);
                if(!Is_Valid_Baud_Rate(baud)){
                    printf("Invalid Baud Rate %d\n",baud);
                    Print_Usage_Message();
                    return false;
                }
                config->baudrate = baud;
                baudRateSet = true;
                break; 
        
            case 'h':
            case '?':
                Print_Usage_Message();
                return false;
                break;
            default:
                printf("Invalid Option %c\n",c);
                Print_Usage_Message();
                break;
        }
    }
    if(config->mode == NOTSET){
        printf("Mode not set\n");
        Print_Usage_Message();
        return false;
    }
    else if(!baudRateSet){
        printf("Baud Rate not set\n");
        Print_Usage_Message();
        return false;
    }
    else if(config->mode == WRITEONLYMODE && !sendPortSet){
        printf("Send Port not set\n");
        Print_Usage_Message();
        return false;
    }
    else if(config->mode == READONLYMODE && !receivePortSet){
        printf("Receive Port not set\n");
        Print_Usage_Message();
        return false;
    }
    else if(config->mode == DUALSEPERATEMODE && (!sendPortSet || !receivePortSet)){
        printf("Send or Receive Port not set in dual seperate mode\n");
        Print_Usage_Message();
        return false;
    }
    else if(config->mode == DUALCOMBINEDPORT && !sendPortSet && !receivePortSet ){
        printf("No Port set in dual combined mode\n");
        Print_Usage_Message();
        return false;
    }
    else 
        return true;




}

pthread_t SendThread, ReceiveThread;
int main(int argc,char* argv[]) {
    Serial9BitConfig config; 
    config.mode = NOTSET;
    config.baudrate = 0;
    config.receivePort[0] = '\0';
    config.sendPort[0]='\0';

    if(!Handle_Arguements(argc,argv,&config)){
        return 1;
    }
    


    if(!Setup_Serial_Port(&config)){
        printf("Error setting up Serial Port(s). Exiting program....\n");
        exit(1);
    }
    
    switch(config.mode){
        case WRITEONLYMODE:
            SendingThread(&config);
            break;
        case READONLYMODE:
            ReceivingThread(&config);
            break;
        case DUALSEPERATEMODE:
            pthread_create(&SendThread,NULL,SendingThread,&config);
            pthread_create(&ReceiveThread,NULL,ReceivingThread,&config);
            //wait for both threads to finish
            pthread_join(SendThread,NULL);
            pthread_join(ReceiveThread,NULL);
            break;
        case DUALCOMBINEDPORT:
            pthread_create(&SendThread,NULL,SendingThread,&config);
            pthread_create(&ReceiveThread,NULL,ReceivingThread,&config);
            //wait for both threads to finish
            pthread_join(SendThread,NULL);
            pthread_join(ReceiveThread,NULL);
            break;
    }

    printf("threads done");
    return 0;
}

