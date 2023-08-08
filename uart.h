/// uart.h

	/// available functions

extern int uart_init(char * dev, void (*callback)(int cbtype),int sp, int prio, unsigned int flags);
extern int uart_write(unsigned char *b, int sz);

extern void uart_set_stick_parity(int p);
extern int uart_get_error(int x);
extern int uart_get_version(char * b);
extern int uart_close(void);
extern void uart_send_byte_parity_set(unsigned char x);

	/// ves 1.02.00 - uart configuration in uart_init
	
#define	UART_CFG_TXBLOCK					(1 << 0)
#define	UART_CFG_THREAD_PRIORITY	(1 << 1)

///#define	UART_PARITY_ERROR	(1 << 15)

	/// uart hardware ports & bits

#define	COM1_PORT				0x3f8
#define	COM2_PORT				0x2f8
#define	COM1_IRQ				4
#define	COM2_IRQ				3

#define	NUM_UART_PORTS	8

#define	DATA_REG				0x00
#define	FIFO_REG				0x02
#define	IE_REG					0x01
#define	CONTROL_REG			0x03
#define	STATUS_REG			0x05

	/// CONTROL REG
#define	PARITY_BIT				(1<<3) 
#define	PARITY_EVEN_BIT		(1<<4)
#define	STICK_BIT					(1<<5)
#define	PARITY_STICK_0		(PARITY_BIT | PARITY_EVEN_BIT | STICK_BIT)
#define	PARITY_STICK_1		(PARITY_BIT | STICK_BIT)
#define	PARITY_STICK_MASK	(~(PARITY_STICK_1 | PARITY_EVEN_BIT))

	/// FIFO REG
#define	ENABLE_FIFO_BIT		(1 << 0)


	/// STATUS REGISTER DEFS

#define	DR_BIT				(1 << 0)		/// data ready
#define	OE_BIT				(1 << 1)		/// overrun error
#define	PE_BIT				(1 << 2)		/// parity error bit
#define	THR_EMPTY_BIT	(1 << 5)		/// Transmitter holding register

	/// CALL BACK DATA DEFS
	/// 16 bit call back data
	///		RX
	///			bits 	0-7		byte transmitted/recieved
	///			bit		8		wake up bit
	///			bit		9		0 - RXBIT
	///			bit		10-15	6 bit error code (00h-3Fh)
	///							00 - OK
	///							3E - DRIVER NOT READY
	///							3F - BUFFER OVERFLOW
	///		TX
	///			bits	0-7		number of bytes xmitted
	///			bit		8		not used
	///			bit		9		1 - TXBIT
	///			bits	10-15	6 bit error code (00h-3Fh)
	///							00 - OK
	///							3D - TX TIMEOUT
	///							3E - DRIVER NOT READY
	///							3F - BUFFER OVERFLOW
	
#define	UART_WAKEUP_BIT				8
#define	UART_RXTX_BIT					9
#define	UART_ERROR_BIT				10
#define	UART_ERROR_MASK				(0xFD00)		/// 1111-1100-0000-0000
#define	UART_RXTX_MASK				(~(1 << UART_RXTX_BIT))
#define	UART_WAKEUP_MASK			(~(1 << UART_WAKEUP_BIT))
#define	UART_DATA_MASK				(0xff)
#define	UART_CB_RX						(0 << UART_RXTX_BIT)
#define	UART_CB_TX						(1 << UART_RXTX_BIT)
#define	UART_GET_ERROR(X)			(X >> UART_ERROR_BIT)
#define	UART_SET_ERROR(X,EC)	(X |= (EC << UART_ERROR_BIT))
#define	UART_CB_WAKEUP				(1 << UART_WAKEUP_BIT)

#define	UART_TX_TIMEOUT_ERROR	0x3D
	
