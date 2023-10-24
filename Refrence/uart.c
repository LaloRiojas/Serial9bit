/*
 * uart.c
 *
 *  Created on: 15 de out de 2016
 *      Author: ptnk
 */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
//#include <sys/signal.h>
#include <time.h>
#include <sys/ioctl.h>

//#define	enable_chirping 1
//#define	NON_BLOCK_SERIAL	1

#define	VERSION				1
#define	RELEASE				2
#define	MINVERSION			3

#define	DST_NONE				0
#define	ONE_MILISECOND			(1000)
#define	ONE_SECOND				((unsigned)1000000)
#define	MAX_TIME_ELAPSED		(ONE_SECOND*60)
#define	MAX_TIME_ELAPSED_SEC	60

#define	MAX_TX_DATA				256
/// | 0000 0E TW | dddd dddd
#define	UART_WAKEUP_BIT		8
#define	UART_RXTX_BIT		9
#define	UART_ERROR_BIT		10
#define UART_TX_TO_BIT		11
#define	UART_ERROR_MASK		(0xFD00)		/// 1111-1100-0000-0000
#define	UART_RXTX_MASK		(~(1 << UART_RXTX_BIT))
#define	UART_WAKEUP_MASK	(~(1 << UART_WAKEUP_BIT))
#define	UART_DATA_MASK		(0xff)
#define	UART_CB_RX			(0 << UART_RXTX_BIT)
#define	UART_CB_TX			(1 << UART_RXTX_BIT)
#define	UART_CB_WAKEUP		(1 << UART_WAKEUP_BIT)
#define	UART_TX_TIMEOUT		((1 << UART_TX_TO_BIT) | (1 << UART_ERROR_BIT))

#define	txbuf		txd->buf															/// tx buffer
#define	txidx		txd->idx															/// index to xmtting byte
#define	txsz		txd->sz																/// size of tx message
#define	LOCK		pthread_mutex_lock(&txbuffer_mutex)		/// lock rx mutex
#define	UNLOCK		pthread_mutex_unlock(&txbuffer_mutex)	/// unlock rx mutex

struct TXDATA{
	int	sz;
	int	idx;
	unsigned char buf[MAX_TX_DATA];
};

static 	void 		(*callback)(int cbtype)= NULL;		/// callback ptr
static	pthread_t	pthread_uart_read,
					pthread_uart_write;
struct	TXDATA 		*txd = NULL;						/// tx transmitter
static int volatile uart_thread_flag = 0;
static int volatile serial_fd;
static	pthread_mutex_t txbuffer_mutex 	= PTHREAD_MUTEX_INITIALIZER;

extern int	(*tprintfl)(char * fname, char *fmt, ...); /// LOG

//---------------------------------------------------------------------------
// DEBUG
//---------------------------------------------------------------------------
static int debug_uart = 0;
static int _tprintfl(char *fmt, ...)
{
	if(!debug_uart ) return -1;

	va_list	 	args;
	time_t 		tsec;
	struct tm *ts;
	struct 		timeval	ctv;
	FILE * 		logfile_fd;
	int				ret=0;

	tsec = time(&tsec);
	ts	= (struct tm*) localtime(&tsec);
	gettimeofday(&ctv,DST_NONE);
	logfile_fd = fopen("uart.log","a+");

	if(logfile_fd != NULL){
		// @ concatenate strings (no timestamp printed)
		if(fmt[0] != '@'){
			fprintf(logfile_fd,
				"%04d-%02d-%02d|%02d:%02d:%02d|%8lu",
				ts->tm_year+1900,
				ts->tm_mon+1,
				ts->tm_mday,
				ts->tm_hour,
				ts->tm_min,
				ts->tm_sec,
				tsec
			);
		}
		else{
			// does not print concatenation char '@'
			fmt++;
		}
		va_start(args,fmt);
		ret = vfprintf(logfile_fd,fmt,args);
		fclose(logfile_fd);
	}
	return ret;
}

///---------------------------------------------------------------------
/// receiver thread
/// --------------------------------------------------------------------
/**
 * read a raw 8-bit byte from serial
 * @return	8 bit data or ERROR bit set
 */
inline static unsigned int serial_read_byte(void)
{
	int n;
	char buf;
	unsigned int b = 0;

	tprintfl("/shock/log/uart.log","%s start\n",__func__);

#ifdef NON_BLOCK_SERIAL
	n = 0;
	while(n == 0){
		n = read(serial_fd,&buf,1);
		if(n == 0 ) usleep(ONE_MILISECOND*10);
	}
#else
		n = read(serial_fd,&buf,1);
#endif

	if(n != 1 ) {
		tprintfl("/shock/log/uart.log","error reading serial %d %d\n",n,b);
		b = (1 << UART_ERROR_BIT);
	} else {
		tprintfl("/shock/log/uart.log","%02X\n",buf);
		b = ((int) buf) & 0xff;
	}

	tprintfl("/shock/log/uart.log","%s end\n",__func__);
	return (int) b;
}
/**
 * read 9 bit serial
 * @return	8 bit data + ERROR and 9th bit flag
 */

inline static unsigned int serial_read9(void)
{
	int b;

	if ((b = serial_read_byte()) & (1<<UART_ERROR_BIT)) return b;
	/* from serial config
	 * 		FF FF    = FF
	 * 		FF 00 XX = XX with 9th bit 1
	 * 		XX       = XX with 9th bit 0
	 */
	if(b != 0xFF) return b;

	if ((b = serial_read_byte()) & (1<<UART_ERROR_BIT)) return b;
	if(b == 0xFF) return b;
	if(b == 0x00)
	{
		if ((b = serial_read_byte()) &(1<<UART_ERROR_BIT))
		{
			return b;
		}
		return b | (1<<UART_WAKEUP_BIT) ;
	}
	return (1<<UART_ERROR_BIT);
}

inline int uart_get_error(int x)
{
	return (x >> UART_ERROR_BIT);
}

/**
 *
 * @param x
 */
static void *uart_read_thread(void * x)
{
	unsigned int c = 0;

	tprintfl("/shock/log/uart.log","U: thread read started %d\n",getpid());

	while(uart_thread_flag){
		c = serial_read9();
		callback(c);
		usleep(ONE_MILISECOND);
	}
	tprintfl("/shock/log/uart.log","U: thread read end %d\n",getpid());
	sleep(5);
	return x;
}

#ifdef USER_TX_THREAD

/**
 *
 * @param x
 */
static void * uart_write_thread(void * x)
{
	int output_size,
		last_tx_size = 0,
		pending_tx_timeout = 0;

	_tprintfl("U: thread write started %d\n",getpid());
	while(uart_thread_flag) {
		usleep(1000*100);
		continue;
	}

	while(uart_thread_flag) {

		if(txsz) {
			/* check for starting timeout counting */
			if(last_tx_size <= 0) {
				if(write(serial_fd,txbuf,txsz) <= 0) {
					_tprintfl("U: T send %d bytes first %02X ERROR\n",txsz,txbuf[0]);

				} else {
					_tprintfl("U: T send %d bytes first %02X Ok\n",txsz,txbuf[0]);
				}
				pending_tx_timeout = 5*txsz;
				last_tx_size = txsz;
				//usleep(10*ONE_MILISECOND);
				usleep(ONE_MILISECOND*3);
				continue;
			}
			/* should never occur, unless something is quite wrong */
			output_size = 0;
			if(ioctl(serial_fd, TIOCOUTQ, &output_size) < 0) {
				_tprintfl("U: T TIOCOUTQ error\n");
				tcflush(serial_fd,TCIOFLUSH);
				usleep(ONE_MILISECOND*10);
				callback(UART_CB_TX | UART_TX_TIMEOUT);
				txsz = 0;
				pending_tx_timeout = 0;
				last_tx_size = -1;
				continue;
			}
			/* bad serial xmiter - cable disconnected ?? */
			_tprintfl("U: T output size %d\n",output_size);
			if(output_size < txsz) {
				if (--pending_tx_timeout == 0) {
					callback(UART_CB_TX | UART_TX_TIMEOUT);
					txsz = 0;
					tcflush(serial_fd,TCIOFLUSH);
					usleep(ONE_MILISECOND*10);
					last_tx_size = -1;
					continue;
				}
				usleep(ONE_MILISECOND*1);
				continue;
			}
			int c = txsz | UART_CB_TX;
			txsz = 0;
			last_tx_size = -1;
			callback(c);
		}
		//usleep(ONE_MILISECOND*20);
		usleep(ONE_MILISECOND*3);
	}
	_tprintfl("U: thread write end %d\n",getpid());
	return x;
}
#endif

///_____________________________________________________________________
///
///	SERIAL PORT CONFIG
///_____________________________________________________________________
///

/**
 *
 */
static void serial_close(void)
{
	fcntl(serial_fd,F_SETFL,FNDELAY);
	if(serial_fd > 0)
		close(serial_fd);
}
/**
 *
 * @param port
 * @param speed
 * @return
 */
inline static int serial_open(char * port, int speed)
{
	struct termios options;

#ifdef NON_BLOCK_SERIAL
	serial_fd = open(port, O_RDWR | O_NOCTTY| O_NONBLOCK) ;
#else
	serial_fd = open(port, O_RDWR | O_NOCTTY) ;
#endif

	tprintfl("/shock/log/uart.log","%s serial %s = %d \n",__func__,port,serial_fd);

	if (serial_fd > 0)
	{
		fcntl(serial_fd, F_SETFL, 0);
		tcgetattr(serial_fd,&options);
		cfsetispeed(&options,speed);
		cfsetospeed(&options,speed);
		options.c_cflag = B19200 | PARENB;
		options.c_cflag |= (CLOCAL | CREAD);
		options.c_cflag |= CS8;
		options.c_lflag &= ~(ICANON | ECHO | ECHOE |ISIG);
		options.c_lflag = 0;
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		/* blocking read */
#ifdef NON_BLOCK_SERIAL
		options.c_cc[VTIME]    = 0;	/* inter-character timer unused */
        options.c_cc[VMIN]     = 0;	/* blocking read until 1 character arrives */
#else
		options.c_cc[VTIME]    = 0;	/* inter-character timer unused */
        options.c_cc[VMIN]     = 1;	/* blocking read until 1 character arrives */
#endif
		options.c_oflag &= ~OPOST;
		tcsetattr(serial_fd,TCSANOW,&options);
		tcflush(serial_fd,TCIFLUSH);
	}
	atexit(serial_close);
	return (serial_fd);
}
/**
 *
 *
 */
int setup_serial_send_9(void)
{
	struct termios tio;
	if(tcgetattr(serial_fd,&tio) < 0)
	{
		perror("tcgetaddr:");
		return -1;
	}
	tio.c_iflag = 0;
	tio.c_lflag = 0;
	tio.c_cflag |= 0x40000000;//CMSPAR;
	tio.c_iflag |= PARMRK;
	tio.c_iflag |= INPCK;
	tio.c_cflag |= PARENB;
	tio.c_cflag |= PARODD;
	if(tcsetattr(serial_fd,TCSANOW,&tio) < 0)
	{
		_tprintfl("U: send9 setup error\n");
		perror("tcsetaddr:");
		return -1;
	}
	return 0;
}
/**
 *
 */
inline static int setup_serial_receive_9(void)
{
	struct termios tio;
	tprintfl("/shock/log/uart.log","%s start\n",__func__);
	if(tcgetattr(serial_fd,&tio) < 0)
	{
		perror("tcgetaddr:");
		tprintfl("/shock/log/uart.log","%s error get addr\n",__func__);
		return -1;
	}
	tio.c_iflag = 0;
	tio.c_lflag = 0;
	tio.c_cflag |= 0x40000000;//CMSPAR;
	tio.c_iflag |= PARMRK;
	tio.c_iflag |= INPCK;
	tio.c_cflag |= PARENB;
	if(tcsetattr(serial_fd,TCSANOW,&tio) < 0)
	{
		_tprintfl("U: send9 setup error\n");
		tprintfl("/shock/log/uart.log","%s error set addr\n",__func__);
		perror("tcsetaddr:");
		return -1;
	}
	tprintfl("/shock/log/uart.log","%s end\n",__func__);

	return 0;

}
/**
 *
 * @param port
 * @param cb
 * @return
 */
int uart_init(char * devx,void (*cb)(int cbtype))
{

	if(devx == NULL)
	{
		tprintfl("/shock/log/uart.log","U: init serial NULL dev\n");
		_tprintfl("U: init serial error nulldev \n");
		return -1;
	}

	char dev[512];

	sprintf(dev,"%s",devx);
	//sprintf(dev,"%s","/dev/ttyS0");

	tprintfl("/shock/log/uart.log","U: init serial dev %s\n",dev);

#ifdef NON_BLOCK_SERIAL
	tprintfl("/shock/log/uart.log","U: init serial %s -- NON BLOCKING\n",dev);
#else
	tprintfl("/shock/log/uart.log","U: init serial %s\n",dev);
#endif

	if(serial_open(dev,B19200) < 0) {
		_tprintfl("U: init serial error %s\n",dev);
		return -2;
	}

	setup_serial_receive_9();

	txd = (struct TXDATA *)malloc(sizeof(struct TXDATA));
	if(txd == NULL)	{
		_tprintfl("U: init error malloc\n");
		return -2;
	}

	callback = cb;
	uart_thread_flag = 1;
	tprintfl("/shock/log/uart.log","%s start thread\n",__func__);

	if(pthread_create(&pthread_uart_read, NULL, uart_read_thread, NULL) != 0) {
		_tprintfl("U: init error uart read\n");
		return -1;
	}
	/*
	if(pthread_create(&pthread_uart_write, NULL, uart_write_thread, NULL) != 0) {
		_tprintfl("U: init error uart write\n");
		return -1;
	}
	*/
	tprintfl("/shock/log/uart.log","%s end\n",__func__);
	_tprintfl("U: init OK\n");
	return 0;
}
/**
 *
 * @param b
 * @param sz
 * @return
 */
int uart_write(unsigned char * b, int sz)
{
#ifdef NON_BLOCK_SERIAL
	int n = write(serial_fd,b,sz);
#else
	int n = write(serial_fd,b,sz);
	tcdrain(serial_fd);
	usleep(1);
#endif
	if (n > 0) {
		int c = sz | UART_CB_TX;
		txsz = 0;
		callback(c);
	} else {
		_tprintfl("U: error sending %d bytes status %X\n",sz,n);
		int c = sz | UART_CB_TX | UART_TX_TIMEOUT;
		txsz = 0;
		callback(c);
	}
	return sz;
#
}
/**
 *
 * @param x
 */
void uart_send_byte_parity_set(unsigned char x)
{

#ifdef enable_chirping
	setup_serial_send_9();
	tcdrain(serial_fd);
	setup_serial_receive_9();
#else
	if(write(serial_fd,&x,1)){}
	if(tcdrain(serial_fd)){}
#endif

}
/**
 *
 * @return
 */
int uart_close(void)
{
	if(uart_thread_flag){
		_tprintfl("U: wait threads to end\n");
		LOCK;
		uart_thread_flag = 0;
		UNLOCK;
		pthread_join(pthread_uart_read,NULL);
		pthread_join(pthread_uart_write,NULL);
		serial_close();
		_tprintfl("U: threads ended\n");
		LOCK;
		if(txd != NULL){
			free(txd);
			txd = NULL;
		}
		UNLOCK;
		callback = NULL;
	}
	_tprintfl("UART: finished \n");
	return 0;
}
