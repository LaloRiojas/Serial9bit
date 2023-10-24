import serial
def serial():
    serialPort = serial.Serial('dev/tty0',9600,timeout = 1)
    data = b'hello, serial!'
    serialPort.write(data);
    receivedData = serialPort.readline()
    print( receivedData.decode())
    serialPort.close()

if __name__ == "__main__":
    serial()