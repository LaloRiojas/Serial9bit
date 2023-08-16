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
int send9bit(int fd,unsigned char data){
    struct termios options;
    if(tcgetattr(fd, &options) != 0){
        perror("error in tcgetattr");
        return (-1);
    }
    options.c_cflag |= PARENB|CMSPAR|PARODD;
    if(tcsetattr(fd, TCSANOW,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }
    if(write(fd,&data,1) != 1){
        perror("error in write");
        return -1;
    }
    options.c_cflag &= ~(PARODD);
    if(tcsetattr(fd, TCSANOW,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }
    return 0;


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

void* ReceivingThread(void* port){

    printf("ReceivingThread started\n");
    port = (char*)port;
    int fd;
    if(0!= setupSerialPort(port,&fd)){
        perror("error in setupSerialPort");
        exit(3);
    }
    int count = 0;
    unsigned char data;
    char String [20];
    int stringIndex = 0;
    bool wakeupbit = false;
    while (!wakeupbit){
         wakeupbit = check9bit(fd, &data);
         printf(": wakeupbit = %d\n",wakeupbit);
         count ++;
         if(count>20){
                printf("error in wakeupbit");
                pthread_exit(NULL);
         }
    }
     // wait for fisrt wakeup bit
     //count =1;
/*    while(count<5){
            do {
                String[stringIndex] = data;
                stringIndex++;
                wakeupbit = check9bit(fd, &data);
            }
            while(!wakeupbit);
            String[stringIndex] = '\0';
            printf("received data == :%s\n", String);
            stringIndex = 0;
            memset(String,0,sizeof(String));
            count++;
    }*/

    pthread_exit( NULL);
}
void* SendingThread(void* port){
    int ret =0;
    printf("SendingThread started\n");
    port = (char*)port;
    int fd;
    if(0!= setupSerialPort(port,&fd)){
        perror("error in setupSerialPort");
        exit(ret = 3);
    }
    for (int i = 0; i < 5; ++i) {
        //send9bit(fd, 0x01 & 0x80);// 0x01 is the address of gaming machine 1. 0x80 is offset according to SAS protocol
        send9bit(fd, 'W');
        write(fd, "hello", 5);
        usleep(200000);//200ms polling
    }





    pthread_exit( &ret);
}

int main() {
    printf("starting program\n");
    const char *Sendport = "/dev/ttyS0";
    const char *Receiveport = "/dev/ttyS1";

    if(pthread_create(&SendThread, NULL, SendingThread, (void*)Sendport) !=0 ){
        printf("error creating SendThread\n");
        return 1;
    }

    if( pthread_create(&ReceiveThread, NULL, ReceivingThread, (void*)Receiveport) !=0){
        printf("error creating SendThread\n");
        return 1;
    }
    
    pthread_join(SendThread, NULL);
    pthread_join(ReceiveThread, NULL);




}

