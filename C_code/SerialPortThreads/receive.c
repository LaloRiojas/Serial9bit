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

void red () {
  printf("\033[1;31m");
}

void yellow() {
  printf("\033[1;33m");
}

void reset () {
  printf("\033[0m");
}


bool check9bit( int fd, unsigned char* data){ // data will always be the last real byte sent. it will never be the 0xff or 0x00 byte that comes from the parity bit.
    read(fd,data,sizeof(*data));
    printf("%c",*data);
    if(*data ==0xff){//catch byte
        read(fd,data,sizeof(*data));
        printf("%c",*data);
        if(*data = 0x00){//parity bit set.
            read(fd,data,sizeof(*data));// read the next bit. this is the real data
            printf("%c",*data);
            return true;
        }
        else if (*data ==0xff){ // the real data was 0xff. the stream was 0xff/0xff
            return false;
        }
        else{
            printf("error in check9bit");
            return false;
        }

    }
    return false;
}

void print_results(char* buf, int size){
    for(int i=0;i<size;i++){
        char c = buf[i];
        if(c==0){
            printf("%d",buf[i]);
        }
        else if(c==-1){
            printf("n");
        }
        else{
            red();
            printf("%c",buf[i]);
            reset();
        }
    }

    printf("\n");
}

#define buffer_size 30
void* ReceivingThread(void* port){

    printf("ReceivingThread started\n");
    port = (char*)port;
    int fd;
    fd = Setup_Serial_Receive(port);
    sleep(1);
    char buf [buffer_size];
    printf("\n\n");
    read(fd,&buf,buffer_size);
    printf("\n\n");
    print_results((char*)&buf,buffer_size);

    pthread_exit( NULL);
}


