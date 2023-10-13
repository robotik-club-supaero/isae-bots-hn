import serial
import time


class ArduinoCommunicator:
    
    led_button_state = -1  # unknown
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600): #TODO give a linked name to the arduino Nano
        self.ser = serial.Serial(port, baudrate)
        
        # read and write timeouts are None by default (but set for good measure)
        self.ser.timeout = None
        self.ser.write_timeout = None

    def send_command(self, command):
        self.ser.write(command.encode())

    def receive_response(self):
        return self.ser.readline().decode().strip()
    
    
    def establish_communication(self):
        
        print("Waiting for communication to be established ...")
            
        # set read timeout to a value for non blocking reads
        self.ser.timeout = 1.0

        while True: # wait for communication to be established
            
            self.send_command('i')
            res = self.receive_response()  # times out after 1 second
            
            if res == 'o':
                print("Connection established")
                return
            
            time.sleep(0.01)
            
            
    def read_button_state(self):
        
        self.send_command('r')
        res = self.receive_response()
        print('Initial LED button state : ', res)
        self.led_button_state = res
            
            
            
def main():

    # Example Usage
    arduino = ArduinoCommunicator()
    arduino.establish_communication()

    # read the button once at the start
    arduino.read_button_state()


    # begin = time.perf_counter()

    # set value of timeout to None so that we wait until a value comes
    arduino.ser.timeout = None

    while True:
        button_state = arduino.receive_response()
        print(f'New LED button State: {button_state}')
        
        
if __name__ == '__main__':
    main()