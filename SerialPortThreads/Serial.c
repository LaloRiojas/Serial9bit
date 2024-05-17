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

uint8_t oddParityTable [256] = {1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1};
bool tableInitialized = false;


//Inity_Parity_Table() longer needed since its a constant always.

/*
void Init_Parity_Table(void){
    //calculate the even parity for all possible 8 bit values. 256 total  
    for(uint8_t count =0;count<256;count++ ){
        uint8_t temp = count;
        uint8_t parity = 0;
        while(temp){
            parity = parity ^ (temp & 1) ;
            temp >>= 1;
        }
        oddParityTable[count] = parity & 1; // 1 if odd. 0 if even
    }
    tableInitialized = true; 
}
*/

int supportedBaud[] = {9600, 19200, 38400, 57600, 115200};
int supportedBuadSetting[] = {B9600,B19200,B38400,B57600,B115200};
#define supportedBaudSize  5
//checks if the speed is in the above POSIX supported speeds. 
//not more speeds can be added but not really needed. 
bool Is_Valid_Baud_Rate(int speed){
    //loop and return if speed is equal to a valid speed in array;
    for (int i = 0; i < supportedBaudSize; i++)
        if(speed == supportedBaud[i])
            return true; 

    return false;  
}

int Get_Baud_Rate_Setting(int baud){
    for (int i = 0; i < supportedBaudSize; i++)
        if(baud == supportedBaud[i])
            return supportedBuadSetting[i]; 

    return -1; 
}

/**
 * @brief changes the parity bit settting depending on the data that is going to be sent.
 * 
 * @param data 8 bit data
 * @param fd port to send it in 
 * @param parity_bit desired 9th bit
 */

void Set_Parity_Bit(uint8_t* data, int fd, bool parity_bit){

    // look up table for the even/odd high bits of the data (only for 8 bit data)
    bool isodd = oddParityTable[*data]; 
    
    //get the fd options to modify
    struct termios options;
    tcgetattr(fd, &options);

    //set the parity flags
    options.c_cflag|= PARENB;

    //odd parity if (evendata and 9thbit=0) or (odddata and 9thbit=1)
    //this can be changed to isodd == paritybit but makes code unclear
    if((isodd&&parity_bit)||(!parity_bit&&!isodd)){ 
        options.c_cflag|= PARODD; //odd parity
        #ifdef DEBUG
            printf(" ,parity bit %1d ,oddness %1d ,odd parity\n",parity_bit,odd)
        #endif
    }

    //even parity if (evendata and 9thbit=1) or (odddata and 9thbit=0)
    else{
        options.c_cflag&= ~PARODD; //even parity
        #ifdef DEBUG
            printf(" ,parity bit %1d ,oddness %1d ,even parity\n",parity_bit,odd)
        #endif
    }

    //set the stuff with drain so that other things are not effected
    //If optional_actions is TCSADRAIN, the change shall occur after all output written to fildes is transmitted. This function should be used when changing parameters that affect output.
    tcsetattr(fd, TCSADRAIN ,&options);


}


int Setup_Serial_Receive(const char* port, int baudrate){
    int fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        printf("could not open Receive port '%s': blocking until available\n",port);
        while ((fd = open(port, O_WRONLY| O_NOCTTY|O_SYNC )) == -1) {
            sleep(1);
        }
        printf("successfully opened Receive port %s\n",port);
    }

    struct termios options;
    tcgetattr(fd, &options);

    int baudSetting = Get_Baud_Rate_Setting(baudrate);
    if(baudSetting!=-1){
        cfsetispeed(&options,baudSetting);
    }
    else{
        printf("invalid baudrate when settting up Send port '%s'\n",port);
        close(fd);
        return -1;
    }    

    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity and that is what we calculate the 9th bit based on 

    options.c_iflag |= PARMRK; // mark parity errors
    options.c_iflag &= ~IGNPAR; // dont ignore parity errors
    options.c_iflag |=INPCK; // enable parity checking

    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits

    //local mode (ignore modem control lines)
    options.c_cflag |= CLOCAL;

    //enable the reader
    options.c_cflag |= CREAD;

    //blocks until either 100 bytes are read or 10 deciseconds (1 second) interbyte delay has passed
    options.c_cc[VMIN] = 100;
    options.c_cc[VTIME] = 10;

    //disabling other random stuff that might be on
    options.c_lflag &= ~ ECHO;// no echo
    options.c_lflag &= ~ICANON;// stuff is read byte by byte instead of by line
    options.c_lflag &= ~ECHOE; // no echo
    options.c_lflag &= ~ISIG;// no signalling from ctrl c or ctrl z
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;
    options.c_iflag &= ~IXANY;
    
    tcsetattr(fd, TCSADRAIN,&options);    
    return fd;
}

int  Setup_Serial_Send(const char* port, int baudrate){
    int fd = open(port, O_WRONLY| O_NOCTTY|O_SYNC );
    if(fd == -1){
        printf("could not open Sending port '%s' : blocking until available\n",port);
        while ((fd = open(port, O_WRONLY| O_NOCTTY|O_SYNC )) == -1) {
            sleep(1);
        }
        printf("successfully opened Send port '%s'\n",port);
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    
    //set the baudrate 
    int baudrateSeting = Get_Baud_Rate_Setting(baudrate);
    if(baudrateSeting!=-1){
        cfsetospeed(&options,baudrateSeting);
    }
    else{
        printf("invalid baudrate when settting up Send port '%s'\n",port);
        close(fd);
        return -1;
    }

    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity but will be switched depending on data being sent  when sending
    
    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits
    //local mode (ignore modem control lines)
    options.c_cflag |= (CLOCAL);

    //disabling other random stuff that might be on
    options.c_lflag &= ~(ICANON | ECHO | ECHOE |ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSADRAIN,&options);

    return fd;
    
}


//UNTESTED AND MAY NOT WORK because reading 9 bit data assumes the parity checking is even.
//However, when sending data over the same serial port in 9 bits you have to constantly switch
//the parity setting for each byte that you send. 
//I am not sure that if you send data at the same time as you read data, the incoming data will
//be affected by the changing parity settings. This is hard to overcome because the incoming buffer say 30 bytes
//may have different parity checking settings along the 30 bytes. 
//I really wish that sending and receiving didnt use the same PARROD flag to set its parity type
int Setup_Serial_SendAndReceive(const char* port, int baudrate){

    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        printf("could not open SendAndReceive port '%s': blocking until available\n",port);
        while ((fd = open(port, O_WRONLY| O_NOCTTY|O_SYNC )) == -1) {
            sleep(1);
        }
        printf("successfully opened Send and Receive port '%s'\n",port);
    }


    struct termios options;
    tcgetattr(fd, &options);

    int baudrateSeting = Get_Baud_Rate_Setting(baudrate);
    if(baudrateSeting!=-1){
        cfsetospeed(&options,baudrateSeting);
        cfsetispeed(&options,baudrateSeting);
    }
    else{
        printf("invalid baudrate when settting up Send port '%s'\n",port);
        close(fd);
        return -1;
    }    


    //setup parrity
    options.c_cflag |= PARENB; // enable parity marking see end of file for description on how to detect parity
    options.c_cflag &= ~PARODD; // default is even parity but will be switched depending on data being sent  when sending

    options.c_iflag |= PARMRK; // mark parity errors
    options.c_iflag &= ~IGNPAR; // dont ignore parity errors
    options.c_iflag |=INPCK; // enable parity checking

    //setup stop bits
    options.c_cflag &= ~CSTOPB; // 1 stop bit default

    // setup data size 
    options.c_cflag &= ~CSIZE; // clear the data size bits
    options.c_cflag |= CS8; // set the data size to 8 bits

    //local mode (ignore modem control lines)
    options.c_cflag |= CLOCAL;

    //enable the reader
    options.c_cflag |= CREAD;

    //make readding blocking unitl 1 byte is read
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    //disabling other random stuff that might be on
    options.c_lflag &= ~ ECHO;// no echo
    options.c_lflag &= ~ICANON;// stuff is read byte by byte instead of by line
    options.c_lflag &= ~ECHOE; // no echo
    options.c_lflag &= ~ISIG;// no signalling from ctrl c or ctrl z
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;
    options.c_iflag &= ~IXANY;
    
    tcsetattr(fd, TCSADRAIN,&options);
    return fd;
}
bool Setup_Serial_Port(Serial9BitConfig* config){
    if(config == NULL || !Is_Valid_Baud_Rate(config->baudrate))return -1;

    //read only mode uses Setup_Serial_Send()
    if(config->mode == READONLYMODE){
        config->receiveFD = Setup_Serial_Receive(config->receivePort,config->baudrate);
        return config->receiveFD!=-1;
    }
    //write only mode uses Setup_Serial_Send()
    else if(config->mode == WRITEONLYMODE){
        config->sendFD =  Setup_Serial_Send(config->sendPort, config->baudrate);
        return config->sendFD != -1;
    }

    else if (config->mode == DUALSEPERATEMODE){

        //set up the ports seperately since we know they are not the same port;
        config->sendFD = Setup_Serial_Send(config->sendPort,config->baudrate) && Setup_Serial_Receive(config->sendPort, config->baudrate);
        config->receiveFD = Setup_Serial_Receive(config->receivePort,config->baudrate);

        //both file descriptors need to be valid to return true;
        return config->sendFD != -1 && config->receiveFD!=-1;
    }

    else if(config->mode == DUALCOMBINEDPORT){
        if(strcmp(config->sendPort,"")!=0){

            config->sendFD =  Setup_Serial_SendAndReceive(config->sendPort,config->baudrate);
            config->receiveFD = config->sendFD;
            //return false if the file descriptor was an error
            return config->sendFD != -1;
        }
        else if(strcmp(config->receivePort,"")!=0){

            config->sendFD = Setup_Serial_SendAndReceive(config->receivePort,config->baudrate);
            config->receiveFD = config->sendFD;
            return config->sendFD != -1;
        }
        else{
            printf("ERROR in Setup_Serial_Port: no valid port");
            return -1;
        }
    }

    else{
        printf("ERROR in Setup_Serial_Port: config->mode is not valid.");
        return -1;
    }

}



//SENDING FUNCTIONS
int Write_Char_9bit(int fd,unsigned char data,bool wakeupbit){
    
    #ifdef DEBUG
        printf("writing %c/%d",data,data);
    #endif
        Set_Parity_Bit(&data, fd, wakeupbit);
    
    return write(fd, &data, 1);
}
// wakeupbitset is a bitset of which characters are going to have the 9th bit high. 
// 0b00000001 or 1 means the first character will have the 9th bit high
// 0b00000011 or 3 means the first and second character will have the 9th bit high
// etc
int Write_String_9bit(int fd, char* data,int wakeupbitset){ 
    int i;
    for (i =0; i<strlen(data); i++){
        int errno =  Write_Char_9bit(fd, data[i],wakeupbitset&1);
        if(errno==-1){
            return -1;
        }
        wakeupbitset = wakeupbitset>>1;
    }
    return i;
}
//writes the first 8 bits as a char and the 9th bit as the parity wakeup bit
int Write_Array_9bit(int fd,uint16_t* dataArray,int size){
    for(int i=0;i<size;i++){
        if(Write_Char_9bit(fd, 0xFF&(dataArray[i]>>1),dataArray[i]&1) == -1){
            return -1;
        }
    }
    
}
// RECEIVING FUNCTIONS

//size is the size of the buffers
//buf is the serial port input stream of data
//Datafrom 9 bit is where the result is stored and is expected to be a pointer to an array = size
//DataFrame_9bit.len <= buf.len 
//result is the amount of data filled out. 
int Process_9bit(char* buf,DataFrame_9bit* data, int size,int fd){
    int i,index;
    //
    for (i = 0, index=0; i<size; i++,index++){

        //check if there is a parity error
        //0xFF 0x00 0x[char with parity error]

        if(i+2<size){//check if we would be going out of range of the array before checking ahead bytes
            if(buf[i] ==0xFF && i+2<size && buf[i+1] == 0x00){
                i+=2;   //skip the wakeup sequence
                data[index].letter = buf[i];
                data[index].parityerror = true;
            }
            else {
            data[index].letter = buf[i];
            data[index].parityerror = false;
            }
        }
        else{
            data[index].letter = buf[i];
            data[index].parityerror = false;
        }

        bool dataIsOddParity = oddParityTable[data[index].letter];
        bool par_error = data[index].parityerror;

        //if the receiving port was checking for EVEN parity  
        //9thbit=1 if (even and no error) or (odd and error)
        //9thbit=0 if (even and error) or (odd and no error)

        //if the receiving port was checking for ODD parity  
        //9thbit=1 if (odd and no error) or (even and error)
        //9thbit=0 if (odd and error) or (even and no error)

        //find out what the receiving port is checking for 
        struct termios options;
        tcgetattr(fd,&options);
        bool oddParityMode = (options.c_cflag & PARODD )!=0;//if this bit was cleared then it would be in even mode


        //should the parity bit be 1? aka does the parity mode match the datas parity 
        bool expectedPartitybit = oddParityMode == dataIsOddParity;

        //9thbit = 1 if (expectedParitybit=1 && parityerror=0) or (expectedParitybit=0 && parityerror =1)
        //9thbit = 0 if (expectedParitybit=0 && parityerror=0) or (expectedParitybit=1 && parityerror =1)
        //this is an xor operation or even 
        //9thbit = (parityerror) ? !expectedparitybit : expectedparitybit
        data[index].bit9 = expectedPartitybit ^ data[index].parityerror;



    }
    return index;
}
void red () {
  printf("\033[1;31m");
}
void reset () {
  printf("\033[0m");
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

void Print_processed_data(DataFrame_9bit* data, int size){
    printf("Parritybit \t Letter \t 9bit data\n");
    for(int i =0;i< size;i++){
        printf( "%1d \t\t %c \t\t %03X\n",data[i].bit9,data[i].letter,(data[i].letter<<1)|data[i].bit9);
    }
}

void Encode_9bit_data(DataFrame_9bit* data,uint16_t* result, int size){
    for( int i =0;i<size;i++){
        data[i].bit9 = (data[i].bit9)? 1:0;
        result[i] = (data[i].letter<<1)|data[i].bit9;
    }
}

/*
Receiving serial bits:
whenever the linux serial interface reads a byte, it will check for parity errors. The parity setup is set to check even parity within the 8bit data. 
in other words the parity bit (9th bit) his high whenever there is an even number of high bits in the data bits and low when there is an odd number of high bits in the data. 
parity errors are marked when the data does not conform to this standard. 
parity errors are marked by the serial interface by puttin two preceding bytes in the buffer.
the first byte is 0xFF and the second byte is 0x00.
so if you have a parity error on the byte 0x01, the serial interface will put "0xFF 0x00 0x01" in the buffer.

the parity bit cannot be directly read from the serial interface but it can be calculated.
the value of the parity bit can be determined based on whether there was a parity error and the even/odd characteristic of the data bits. 
if the data bits are even then the parity bit would be high if there was no parity error because it conformed to even parity checking. 
if the data bits are odd then the parity bit would be low if there was no parity error because it conformed to even parity checking.
if there was a parity error then this logic is flipped because it did not conform to even parity checking.


thus 
parity = 1 if EVEN AND NO PARITY ERROR or ODD AND PARITY ERROR
parity = 0 if EVEN AND PARITY ERROR or ODD AND NO PARITY ERROR

*/