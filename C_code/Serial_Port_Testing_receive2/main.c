#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int main() {
    fflush(stdout);
    const char* port = "/dev/ttyS1"; // this is from fcntl library
    int fd = open(port,O_RDONLY, O_NOCTTY)  ;

    if (fd == -1 ){
        perror("error opening file");
        return 1;

    }
    printf("now running");
    struct termios options; // creating the options struct where you set up the serial port
    tcgetattr(fd,&options); // associate the termios with the open serial port represented by fd
    cfsetispeed(&options,B9600); // set the input speed to 9600
    cfsetospeed(&options,B9600); // set the ouput speed to 9600
    // set more important settings
    options.c_cflag |= (CLOCAL | CREAD); // set the local bit and the read only bit. the local bit ignores all old school modem controls and the read allows you to read data.
    options.c_cflag &= ~PARENB; // clears the parity bit flad. this disables this feature
    options.c_cflag &= ~CSTOPB; // clears the stop bit flag. this means there is only one stop bit for now
    options.c_cflag &= ~CSIZE; // clears the size flag which indicates that the size of the data is determined in the next line
    options.c_cflag |= CS8; //8bit 1 byte
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    tcsetattr(fd , TCSANOW, &options); // applys all the previously set bits.

    char buffer [250];
    int bytes_read =0;
    printf("reading now from %s\n",port);
    bytes_read = read( fd, buffer, sizeof(buffer)-1);
    if( bytes_read >0 ){
        buffer [bytes_read] = '\0';
        printf("received data == :%s\n",buffer);

    }
    else
        printf("No data received/n.");

    close(fd);
    printf("port ttyS0 closed");
    return 1;

}
