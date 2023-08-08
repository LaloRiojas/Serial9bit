#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>


int main() {

    const char* port = "/dev/ttyS0"; // this is from fcntl library
    int fd = open(port, O_RDWR,O_NOCTTY)  ;

    if (fd == -1 ){
        perror("error opening file");
        return 1;

    }
    struct termios options; // creating the options struct where you set up the serial port

    fcntl(serial_fd, F_SETFL, 0);
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

    tcsetattr(fd , TCSANOW, &options); // applys all the;laksjdf;lkajsdf;lkjas;dlkjjfjjfj previously set bits.

    const char *data = "hello, serial port is working";
    write(fd,data,strlen(data));
    printf("messagee sent\n");
    while (1){};
    close(fd);
    printf("port ttyS0 closed");

    return 1;





}
