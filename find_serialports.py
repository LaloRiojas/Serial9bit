import serial
import time
import threading

def send_hello():
    with serial.Serial('/dev/ttyS0', 9600, timeout=1) as ser:
        while True:
            ser.write(b'hello')
            time.sleep(3)

def read_from_port(port):
    with serial.Serial(port, 9600, timeout=1) as ser:
        while True:
            data = ser.readline().decode('utf-8').strip()
            if data:
                print(f"Received from {port}: {data}")

def main():
    # Start the thread to send 'hello' to /dev/ttyS0
    send_thread = threading.Thread(target=send_hello)
    send_thread.daemon = True
    send_thread.start()

    # Create threads to read from /dev/ttyS1 - /dev/ttyS4
    read_ports = ['/dev/ttyS1', '/dev/ttyS2', '/dev/ttyS3', ]
    read_threads = []

    for port in read_ports:
        thread = threading.Thread(target=read_from_port, args=(port,))
        thread.daemon = True
        read_threads.append(thread)
        thread.start()

    # Keep the main thread alive to allow background threads to run
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping the script...")

if __name__ == "__main__":
    main()


