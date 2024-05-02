#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 
#

import os, sys
import threading
import time
import socket
import signal
import subprocess

#### Nano ####
from nano.ArduinoCommunicator import ArduinoCommunicator

#### Speaker ####
from speaker.Speaker import Speaker

#### ISB ####
# from isb.isb_lib import initPin, readPin, writePin
from isb.isb_const import NB_LEDS, NB_BUTTONS
from isb.ISBManager import ISBManager

from top_const import TopServerRequest, TopServerCallback, ButtonColorMode, ButtonPressState, TOPSERVER_PORT, MIN_TIME_FOR_BUTTON_CHANGE


class RoslaunchThread(threading.Thread):
    def __init__(self, stop_event):
        self.stdout = None
        self.stderr = None
        self.stop_event = stop_event
        
        threading.Thread.__init__(self)

    def run(self):
        
        print('Entering run')
        
        ROSprocess = subprocess.Popen('/app/scripts/runMatch_test.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print('Passed')
        
        decoded_output = None
        while decoded_output is None:
            
            try:
                process_output = subprocess.check_output(['roslaunch-logs'], stderr=subprocess.PIPE)
                
                # with subprocess.Popen(['roslaunch-logs'],stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
                #     stdout, stderr = process.communicate()

                decoded_output = process_output.decode('utf-8')#[:-1]
                print(decoded_output)
                
                #NOTE no need to sleep because the command is long (~1s)
                
            except subprocess.CalledProcessError:
                # try again, most likely the roscore is not up
                print('Waiting for roscore to be ready')
            
            
        print("Final : ", decoded_output)
        
        return
        
        while True:
            # this command reads the last line of the matchLog file which is updated by the runMatch.sh script
            with subprocess.Popen(['tail', '-n', '10', 'matchLog.log'],stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
                stdout, stderr = process.communicate()

            output = stdout.decode('utf-8')#[:-1]
            
            print(output)
            
            time.sleep(0.2)
  
  
        process.stdout.close()
        
        print("End of RoslaunchThread run")
        
        return
        

        
        
        
        
        while process.poll() is None:
            print(process.stdout.readline())
            sys.stdout.flush()
        print(process.stdout.read())
        process.stdout.close()
        
        print("End of RoslaunchThread run")

        return
        
        while process.poll() is None:
            line = process.stdout.readline()
            lineError = process.stderr.readline()
            if line is not None:
                print("#", line.decode())
            if lineError is not None:
                print("!", lineError.decode())
            else:
                print("nul")
                
            time.sleep(0.2)
                                   
        print(process.stdout.read().decode())
        process.stdout.close()

        return

        self.stdout, self.stderr = p.communicate()
        
        while not self.stop_event.is_set():
            if self.stdout is not None:
                print("Stdout : ", self.stdout.decode())
            if self.stderr is not None:
                print("Stderr : ", self.stderr.decode())
                # print("smth", self.stdout.decode().split('\n')[-1])
                # print(self.roslaunchThread.stderr.decode())
                
            time.sleep(0.2)
            
        #TODO things to do when roslaunch quits ?
        print("Quit roslaunch")
        if self.stdout is not None:
            print("Stdout : ", self.stdout.decode())
        if self.stderr is not None:
            print("Stderr : ", self.stderr.decode())
    
    
            
        

class TopServer():
    
    nanoCom = None  # nano board
    
    serverSocket = None  # TCP server socket
    
    def __init__(self) -> None:
        
        self.isbManager = ISBManager()
        
        self.nanoCom = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
        
        self.isRosLaunchRunning = False
        self.rosLaunchStopEvent = threading.Event()
        self.roslaunchThread = RoslaunchThread(self.rosLaunchStopEvent)

        self.watchButtonStopEvent = threading.Event()
        self.watchButtonThread = threading.Thread(target=self.watchButton, args=(self.watchButtonStopEvent,))
        self.buttonState, self.previousButtonState = None, None
        self.lastButtonChangeTime = time.perf_counter()
                        
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.clients = {}
    
        
        print("TopServer Initialized")
        
        #TODO vérifier les branchements des éléments top et des autres périphériques de la rpi
        # signaler s'il y a un pb
    
    
    
    def initButton(self):
        
        self.nanoCom.establish_communication()
        
        self.buttonState = self.nanoCom.read_button_state()
        self.previousButtonState = self.buttonState

    
    def watchButton(self, stop_event):
        '''
        The LED button plays an important role in the TopServer so it has its own thread
        It is not sent back to clients so read all the time, and should affect the TopServer without delay
        '''
                
        while not stop_event.is_set():
            
            self.buttonState = self.nanoCom.read_button_state()
            
            # dont assume any result if the message is not ButtonPressState (but ButtonColorMode)
            if not isinstance(self.buttonState, ButtonPressState):
                continue
            
            if self.previousButtonState != self.buttonState:
                
                # filter on button changes to avoid fast toggling
                if time.perf_counter() - self.lastButtonChangeTime < MIN_TIME_FOR_BUTTON_CHANGE:
                    continue
                
                self.lastButtonChangeTime = time.perf_counter()
                
                if self.buttonState == ButtonPressState.BUTTON_PRESS_OFF :
                    print("Button OFF")   
                 
                elif self.buttonState == ButtonPressState.BUTTON_PRESS_ON :
                    print("Button ON")
                    
                    if not self.isRosLaunchRunning:
                        self.isRosLaunchRunning = True
                        self.roslaunchThread.start()
                    # self.roslaunchThread.join()
                    
                    # subProc = subprocess.run(["/app/scripts/runMatch_test.sh"], capture_output=True)
                    
                    # print(f"Return code : {subProc.returncode}")
                    # print(f"Stdout : {subProc.stdout.decode()}")
                    # print(f"Stderr : {subProc.stderr.decode()}")
                    
                else:
                    print(f"ERROR : unknown button callback {self.buttonState}")
                    continue
                
            self.previousButtonState = self.buttonState

            time.sleep(0.01)
            
            
            
    def soundCommand(self, command):
        '''Command received from the socket'''
        
        # play sound command
        
        # set volume command
        
        # set mute command
        
        return
    
    
    def handle_client(self, client_socket, client_address):
        try:
            while True:
                # Receive data from the client
                request = client_socket.recv(1024)
                if not request:
                    print(f"Connection with {client_address} closed.")
                    break
                # print(f"Received message from {client_address}: {request.decode()}")
                
                request_list = list(bytes(request))
                
                if len(request_list) == 0:
                    print("ERROR : no request type")
                    break
                
                request_type = TopServerRequest(request_list[0])
                
                print(f"Got request {request_type} with arguments {request_list[1:]}")

                # request cases
                if request_type == TopServerRequest.REQUEST_READ_BUTTONS:
                    buttonStates = self.isbManager.readButtonStates()
                    client_socket.sendall(bytes([TopServerCallback.CALLBACK_OK] + buttonStates))
                    
                elif request_type == TopServerRequest.REQUEST_READ_TRIGGER:
                    triggerState = self.isbManager.readTriggerState()
                    client_socket.sendall(bytes([TopServerCallback.CALLBACK_OK, triggerState]))

                elif request_type == TopServerRequest.REQUEST_WRITE_LED:
                    if len(request_list) != 3:
                        print("ERROR : wrong arguments for REQUEST_WRITE_LED")
                        client_socket.sendall(bytes([TopServerCallback.CALLBACK_WRONG_ARGUMENTS]))
                        continue
                    
                    [ledId, ledState] = request_list[1:]
                    
                    if ledId >= NB_LEDS or ledState not in (0,1,2):
                        print("ERROR : wrong arguments for REQUEST_WRITE_LED (not in range)")
                        client_socket.sendall(bytes([TopServerCallback.CALLBACK_WRONG_ARGUMENTS]))
                        continue
                    
                    self.isbManager.writeLed(led_id=ledId, ledState=ledState)
                    client_socket.sendall(bytes([TopServerCallback.CALLBACK_OK]))
                    

        except ConnectionResetError:
            print(f"Connection with {client_address} reset.")
            
        except OSError:  # NOTE happens in case of Ctrl-C (bad file descriptor)
            # Close the client socket and return
            client_socket.close()
            
        finally:
            # Remove the client from the dictionary
            del self.clients[client_address]
            # Close the client socket
            client_socket.close()
            
            
    def send_message_to_client(self, client_address, message):
        if client_address in self.clients:
            client_socket = self.clients[client_address]
            client_socket.sendall(message.encode())
        else:
            print(f"Client {client_address} not found.")        
        
        
    def run(self):
        
        #TODO gros signal d'erreur si le topServer n'est pas actif (on n'aurait pas d'ISB)
                
        self.watchButtonThread.start()
        
        # listen on port
        self.serverSocket.bind( ("0.0.0.0", TOPSERVER_PORT) )
        maxclients = 10
        self.serverSocket.listen(maxclients)
        
        
        try:
            while True:
                
                # Accept incoming connection
                client_socket, client_address = self.serverSocket.accept()
                print(f"Accepted connection from {client_address}")

                # Store the client connection in the dictionary
                self.clients[client_address] = client_socket

                # Handle the client connection in a new thread
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
                client_thread.start()
                time.sleep(0.001)

                        
        finally:
            # Close the server socket
            self.serverSocket.close()
                
            
            
        return
        self.conn, self.addr = self.serverSocket.accept()
        
        with self.conn:
            print(f"Connected by {self.addr}")
            while True:
                data = self.conn.recv(1024)
                if not data:
                    print('Not data')
                    break
                
                print(f"Data received : {data}")
                
                # self.conn.sendall(data)
        
        
        
    def sig_handler(self, s_rcv, frame):
        """
        Force topServer to quit on SIGINT.
        """
        print("TopServer forced to terminate with Ctrl-C")
        self.closeServer(exitCode=1)
        
        
    def closeServer(self, exitCode):
        
        # close client connections
        for client_socket in self.clients.values():
            client_socket.close()
        
        # close TCP server
        self.serverSocket.close()
        
        # close threads
        self.watchButtonStopEvent.set()
        self.rosLaunchStopEvent.set()
            
        print(f"Exiting TopServer with exit code {exitCode}")
        sys.exit(exitCode)
        

def main():
    
    topserver = TopServer()
    
    signal.signal(signal.SIGINT, topserver.sig_handler)
    
    topserver.initButton()
    
    topserver.run()

    
    
    '''Test'''
    time.sleep(0.5)
    
    # topserver.speaker.setVolume(120)
        
    topserver.speaker.playSound('windowsStartup')
    
    time.sleep(1)
    
    topserver.speaker.setMute(True)
    
    time.sleep(1)
    
    topserver.speaker.setMute(False)
    
    time.sleep(3)
    
    topserver.speaker.playSound('startup')
    
    time.sleep(50)
    


if __name__ == '__main__':
    main()