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
    unsigned char data;
    bool parity;
    bool isODD;
} message;

int setupSerialPort(const char *port, int* fd) {
    *fd = open(port, O_RDWR | O_NOCTTY);
    if (*fd == -1) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(*fd, &options);


    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);



    fcntl(*fd, F_SETFL, 0);
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |=CMSPAR;
    options.c_cflag |=PARODD;
    options.c_cflag |=PARENB;
    options.c_iflag |=INPCK;
    options.c_iflag |=PARMRK;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE |ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    if(tcsetattr(*fd, TCSANOW,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }
    tcflush(*fd,TCIFLUSH);


    return 0;
}
int send9bit(int fd,unsigned char data){
    struct termios options;
    if(tcgetattr(fd, &options) != 0){
        perror("error in tcgetattr");
        return (-1);
    }
   options.c_cflag |= 0x40000000;//CMSPAR;
    if(tcsetattr(fd, TCSANOW,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }
    if(write(fd,&data,1) != 1){
        perror("error in write");
        return -1;
    }
/*    if(tcsetattr(fd, TCSADRAIN,&options) != 0){
        perror("error in tcsetattr");
        return -1;
    }*/
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
         if(wakeupbit){
                printf("wakeupbit found\n");
                break;
         }
         if(count>1000){
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
    for (int i = 0; i < 1000; ++i) {
        //send9bit(fd, 0x01 & 0x80);// 0x01 is the address of gaming machine 1. 0x80 is offset according to SAS protocol
        send9bit(fd, 'W');
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

