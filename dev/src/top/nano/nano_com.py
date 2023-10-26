import serial
import time


class ArduinoCommunicator:
    
    led_button_state = -1  # unknown
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.ser = serial.Serial(port, baudrate)
        
        # read and write timeouts are None by default (but set for good measure)
        self.ser.timeout = None
        self.ser.write_timeout = None

    def send_command(self, command): #TODO be sure to not send newline after the string
        self.ser.write(command.encode())

    def receive_response(self):
        return self.ser.readline().decode().strip()
    
    
    def establish_communication(self):
        
        print("Waiting for communication to be established ...")
            
        # set read timeout to a value for non blocking reads
        self.ser.timeout = 1.0

        while True: # wait for communication to be established
            
            self.send_command("initx")
            print("sent")
            res = self.receive_response()  # times out after 1 second
            print("res : ", res)
            
            if res == 'o':
                print("Connection established")
                return
            
            time.sleep(0.01)
            
            
    def read_button_state(self):
        
        self.send_command("readx")
        res = self.receive_response()
        print('Initial LED button state : ', res)
        self.led_button_state = res
        
        
    def inputSendCommand(self):
        command = input('Enter command : ')
        self.send_command(command)
        res = self.receive_response()
        print('res : ', res)


            
def main():

    # Example Usage
    arduino = ArduinoCommunicator()
    # arduino.establish_communication()

    # read the button once at the start
    # arduino.read_button_state()


    # begin = time.perf_counter()

    # set value of timeout to None so that we wait until a value comes
    arduino.ser.timeout = None

    # wait forever for a button change
    # TODO use a thread for this
    # while True:
    #     # print("waiting for new response")
    #     res = arduino.receive_response()
    #     print(f'New LED button State: {res}')
    #     arduino.led_button_state = res
        
    #     if arduino.led_button_state == 0:
    #         c
    #     else:
    #         arduino.send_command("colbx")
    #     res = arduino.receive_response()
    #     print('res : ', res)
    #     if res == 'o':
    #         print("Changed color")
            
    #     time.sleep(1)
    
    while True:
        arduino.inputSendCommand()

        
        
        
        
if __name__ == '__main__':
    main()