#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
pthread_t SendThread, ReceiveThread;
typedef struct {
    char data[250];
    int index;
}GlobalsendData;

int setupSerialPort(const char *port, int* fd) {
    *fd = open(port, O_RDWR | O_NOCTTY);
    if (*fd == -1) {
        perror("Error opening serial port");
        return -1;
    }
    struct termios options;
    tcgetattr(*fd, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= CMSPAR; // this controls the parity bit
    options.c_cflag &= ~PARENB; // this sets the partiy bit to 0 as default

    options.c_cflag &= ~CSTOPB; // one stop bit
    options.c_cflag |= PARENB; // engable parity bit generation
    options.c_cflag |= INPCK; // enable parity checking
    options.c_cflag |= PARMRK; // mark parity errors
    options.c_cflag &= ~IGNPAR; // don't ignore parity bits received

    if(tcsetattr(*fd, TCSANOW,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }


    return 0;
}
void* ReceivingThread(void* port){
    printf("ReceivingThread started\n");
    port = (char*)port;
    int fd;
    if(0!= setupSerialPort(port,&fd)){
        perror("error in setupSerialPort");
        exit(3);
    }
    //sleep for 1ms
    usleep(1000);
    return "receiving Thread DONE\n";

}
void* SendingThread(void* port){
    printf("SendingThread started\n");
    port = (char*)port;
    int fd;
    if(0!= setupSerialPort(port,&fd)){
        perror("error in setupSerialPort");
        exit(3);
    }
    //sleep for 1ms
    usleep(1000);
    return "sending Thread DONE\n";


}

int main() {
    printf("starting program\n");
    const char *Sendport = "/dev/ttyS0";
    const char *Receiveport = "/dev/ttyS1";

    if(pthread_create(&SendThread, NULL, SendingThread, (void*)Sendport) !=0 ){
        printf("error creating SendThread\n");
        return 1;
    }
    printf("SendThread created\n");
    if( pthread_create(&ReceiveThread, NULL, ReceivingThread, (void*)Receiveport) !=0){
        printf("error creating SendThread\n");
        return 1;
    }
    printf("ReceiveThread created\n");
    void *sendResult = NULL, *receiveResult =NULL;

    pthread_join(SendThread, sendResult);
    pthread_join(ReceiveThread, receiveResult);
    printf("SendThread result: %s\n", (char*)sendResult);
    printf("ReceiveThread result: %s\n", (char*)receiveResult);




}

