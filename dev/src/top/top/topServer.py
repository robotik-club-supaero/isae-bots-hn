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
import re
import json

#### Nano ####
from nano.ArduinoCommunicator import ArduinoCommunicator

#### Speaker ####
from speaker.Speaker import Speaker

#### ISB ####
# from isb.isb_lib import initPin, readPin, writePin
from isb.isb_const import NB_LEDS, NB_BUTTONS
from isb.ISBManager import ISBManager

from oled.Oled_pi import Oled

from top_const import TopServerRequest, TopServerCallback, ButtonColorMode, ButtonPressState, RosLaunchState, \
    TOPSERVER_PORT, MIN_TIME_FOR_BUTTON_CHANGE, \
    sendRequest, receiveCallback


class RoslaunchThread(threading.Thread):
    def __init__(self, stop_event, logOled):
        self.stdout = None
        self.stderr = None
        self.stop_event = stop_event
        self.stopThreadOrder = False
        
        self.ROSLogOled = logOled
        
        self.isStarted = False
        self.isStopped = False
        self.isRunning = False
        self.isReady = False  # means that the match is ready to be launched
                
        threading.Thread.__init__(self)
        
        
        
    def follow(self, thefile):
        '''generator function that yields new lines in a file
        '''
        # seek the end of the file
        thefile.seek(0, os.SEEK_END)
        
        # start infinite loop
        while True:
            # read last line of file
            line = thefile.readline()        # sleep if file hasn't been updated
            if not line:
                time.sleep(0.1)
                continue

            yield line
            
    def run(self):
        
        print("####### Running")
        
        while not self.stopThreadOrder:
            
            if self.isStarted:
                self.runMatch()  # NOTE blocking
                self.isStarted = False
            
            time.sleep(0.01)
            
        print("Ended roslaunch thread")
            
            
    def startMatch(self):
        self.isStarted = True
        
    def stopMatch(self):
        self.isStopped = True
        
    def stopThread(self):
        self.stopThreadOrder = True
        

    def runMatch(self):
        
        print('Running match')
        
        self.isRunning = True
        ROSprocess = subprocess.Popen('/app/scripts/runMatch_test.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
        decoded_output = None
        while decoded_output is None and not self.stopThreadOrder:
            
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
            
        rosLogFilePath = "/root" + decoded_output.split("/root",1)[1].split('\n')[0]
        
        print("Ros log file path : ", rosLogFilePath)
        
        roslaunchFileName = [filename for filename in os.listdir(rosLogFilePath) if filename.startswith("roslaunch-")]
        
        print("roslaunchFileName : ", roslaunchFileName)
        
        if len(roslaunchFileName) == 0:
            print("ERROR : no roslaunch log file found")
            return
        roslaunchFileName = roslaunchFileName[0]
        
        logFile = rosLogFilePath + '/' + roslaunchFileName
        print("Logfile : ", logFile)
        
        PIDPattern = r'-robot-(\d+)\.log'
        roslaunchPID = int( re.split(PIDPattern, roslaunchFileName)[1] )
        
        print("roslaunch PID : ", roslaunchPID)
        
        self.isReady = True #TODO have a real test for roslaunch ready

        NB_LINES = 3
        pattern = r'\[[A-Z]{3}\]|\/[A-Z]{3}' # matches the patterns [AAA] or /AAA with upper case letters A to Z
        
        # loop for match logs and monitoring
        # exits when roslaunch is killed
        self.isStopped = False
        loop_timer_logs = time.perf_counter()
        loop_timer_roslaunch = time.perf_counter()
        while not self.isStopped and not self.stopThreadOrder:
            
            # roslaunch read loop
            if time.perf_counter() - loop_timer_roslaunch > 0.5:
                loop_timer_roslaunch = time.perf_counter()
                try:
                    output_roslaunch = subprocess.check_output(['tail', '-n', '1', logFile]).decode('utf-8')
                except subprocess.CalledProcessError as e:
                    print( "No log files yet")
                    
                # print(output_roslaunch)
                    
                    
            # log analysis loop
            
            if time.perf_counter() - loop_timer_logs > 1.0:
                loop_timer_logs = time.perf_counter()
            
                self.ROSLogOled.oled_clear()
                logList = []
                
                try:
                    output_rosout = subprocess.check_output(['tail', '-n', str(NB_LINES), rosLogFilePath+"/rosout.log"]).decode('utf-8')[:-1]
                except subprocess.CalledProcessError as e:
                    print( "No log files yet")
                else:
                    output__rosout_processed = output_rosout.split('\n')
                    
                    if len(output__rosout_processed) != NB_LINES:
                        print("ERROR : unexpected number of log lines")
                        
                        
                    for k in range (NB_LINES):
                        line = output__rosout_processed[k]
                        output_line = re.split(pattern, line)[-1]
                        if len(output_line) > 0 and output_line[0] == ' ': output_line = output_line[1:]
                        
                        logList.append(output_line[:21])
                        logList.append(output_line[21:])
                        
                    self.ROSLogOled.oled_display_logs(logList)
                    
                            
        
        # received stop event, sending SIGINT to roslaunch
        self.isReady = False
        os.kill(roslaunchPID, signal.SIGINT)
        
        print("Waiting for Roslaunch to shut down ...")
        while ROSprocess.poll() is None and not self.stopThreadOrder:  # ROSprocess shuts down when roslaunch has finished shutting down
            time.sleep(0.1)
            
        # roslaunch has shut down
        self.isRunning = False
        
        print("Roslaunch process shut down")
        
        return
        
        
        logfile = open(logFile, "r")
        loglines = self.follow(logfile) # iterates over the generator
        
        for line in loglines:  # potentially sleeps to wait for the file
            print(line)

        
        
        
        
        
        
        return
            
        # this command reads the last line of the matchLog file which is updated by the runMatch.sh script
        with subprocess.Popen(['tail', '-f', logFile],stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
            stdout, stderr = process.communicate()
            
        while process.poll() is None:
            print(process.stdout.readline())
        print(process.stdout.read())
        process.stdout.close()

        output = stdout.decode('utf-8')#[:-1]
        output_err = stderr.decode('utf-8')
        
        print(output)
        # print("stderr : ", output_err)
        
        time.sleep(0.1)        
        
    
        while True:
            
            # this command reads the last line of the matchLog file which is updated by the runMatch.sh script
            with subprocess.Popen(['tail', '-n', '1', logFile],stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
                stdout, stderr = process.communicate()

            output = stdout.decode('utf-8')#[:-1]
            output_err = stderr.decode('utf-8')
            
            print(output)
            # print("stderr : ", output_err)
            
            time.sleep(0.1)
            
        return

        

class TopServer():
    
    nanoCom = None  # nano board
    
    serverSocket = None  # TCP server socket
    
    def __init__(self) -> None:
        
        self.isbManager = ISBManager()
        
        self.nanoCom = ArduinoCommunicator(port='/dev/ttyUSB0', baudrate=9600) #TODO give a linked name to the arduino Nano
    
        self.speaker = Speaker()
        
        # self.watchButtonStopEvent = threading.Event()
        # self.speakerConstantSoundThread = threading.Thread(target=self.watchButton, args=(self.watchButtonStopEvent,))
        
        self.logOled = Oled()
        
        self.rosLaunchStopEvent = threading.Event()
        self.roslaunchThread = RoslaunchThread(self.rosLaunchStopEvent, self.logOled)
        self.roslaunchState = RosLaunchState.ROSLAUNCH_STOPPED  # also prevents toggling of start/stop of the roslaunch

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
                
        init_callback = self.nanoCom.establish_communication()
        if init_callback == 1:
            print("Couldn't connect to the nano right away, retying once")
            init_callback = self.nanoCom.establish_communication()
            if init_callback == 1:
                print("ERROR : Couldn't connect to the nano after one retry")
                #TODO be more robust
            
        
        self.buttonState = self.nanoCom.read_button_state()
        self.previousButtonState = self.buttonState
        
        self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_BLINKING, color=(255,0,0))
        
        self.logOled.set_bgImage('SRC_OledLogo2.ppm')
        

    def watchButton(self, stop_event):
        '''
        The LED button plays an important role in the TopServer so it has its own thread
        It is not sent back to clients so read all the time, and should affect the TopServer without delay
        This is the thread function that starts and stops the roslaunch process
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
                    
                    if self.roslaunchState in [RosLaunchState.ROSLAUNCH_STARTED, RosLaunchState.ROSLAUNCH_READY]:
                            
                        self.roslaunchState = RosLaunchState.ROSLAUNCH_STOPPING
                        
                        #TODO stop roslaunch thread
                        self.roslaunchThread.stopMatch()
                        
                        self.speaker.playSound('endRos')
                        self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_FADING, color=(255,0,255))
                        
                        #TODO when roslaunch has exited change button color to idle
                        
                                         
                elif self.buttonState == ButtonPressState.BUTTON_PRESS_ON :
                    print("Button ON")
                    
                    if self.roslaunchState == RosLaunchState.ROSLAUNCH_STOPPED:
                        self.roslaunchState = RosLaunchState.ROSLAUNCH_STARTED
                        
                        self.speaker.playSound('startRos')
                        self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_FADING, color=(255,255,0))
                        
                        self.roslaunchThread.startMatch()
                        
                    # self.roslaunchThread.join()
                    
                    # subProc = subprocess.run(["/app/scripts/runMatch_test.sh"], capture_output=True)
                    
                    # print(f"Return code : {subProc.returncode}")
                    # print(f"Stdout : {subProc.stdout.decode()}")
                    # print(f"Stderr : {subProc.stderr.decode()}")
                    
                else:
                    print(f"ERROR : unknown button callback {self.buttonState}")
                    continue
                
            
                
            # manage roslaunch state
                
            if self.roslaunchState == RosLaunchState.ROSLAUNCH_STARTED and self.roslaunchThread.isReady:

                # roslaunch is now ready for action
                self.roslaunchState = RosLaunchState.ROSLAUNCH_READY
                self.speaker.playSound('RosReady')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_STATIC, color=(0,255,0))
                
                
            if self.roslaunchState == RosLaunchState.ROSLAUNCH_STOPPING and not self.roslaunchThread.isRunning:
                
                # roslaunch is now shut down
                self.roslaunchState = RosLaunchState.ROSLAUNCH_STOPPED
                self.speaker.playSound('RosEnded')
                self.nanoCom.changeButtonColor(ButtonColorMode.BUTTON_COLOR_STATIC, color=(255,0,0))
                        
                
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
                
                request_list = json.loads(request.decode())
                                
                if len(request_list) == 0:
                    print("ERROR : no request type")
                    break
                
                request_type = TopServerRequest(request_list[0])
                request_params = request_list[1:]
                
                # print(f"Got request {request_type} with arguments {request_list[1:]}")

                # request cases
                if request_type == TopServerRequest.REQUEST_READ_BUTTONS:
                    buttonStates = self.isbManager.readButtonStates()
                    sendRequest(client_socket, [TopServerCallback.CALLBACK_OK] + buttonStates)
                    # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_OK.value] + buttonStates))
                    
                elif request_type == TopServerRequest.REQUEST_READ_TRIGGER:
                    triggerState = self.isbManager.readTriggerState()
                    sendRequest(client_socket, [TopServerCallback.CALLBACK_OK, triggerState])
                    # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_OK.value, triggerState]))

                elif request_type == TopServerRequest.REQUEST_WRITE_LED:
                    if len(request_params) != 2:
                        print("ERROR : wrong arguments for REQUEST_WRITE_LED")
                        sendRequest(client_socket, [TopServerCallback.CALLBACK_WRONG_ARGUMENTS])
                        # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_WRONG_ARGUMENTS.value]))
                        continue
                    
                    [ledId, ledState] = request_params
                    
                    if ledId >= NB_LEDS or ledState not in (0,1,2):
                        print("ERROR : wrong arguments for REQUEST_WRITE_LED (not in range)")
                        sendRequest(client_socket, [TopServerCallback.CALLBACK_WRONG_ARGUMENTS])
                        # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_WRONG_ARGUMENTS.value]))
                        continue
                    
                    self.isbManager.writeLed(led_id=ledId, ledState=ledState)
                    sendRequest(client_socket, [TopServerCallback.CALLBACK_OK])
                    # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_OK.value]))
                    
                elif request_type == TopServerRequest.REQUEST_PLAY_SOUND:
                    if len(request_params) != 1:
                        print("ERROR : wrong arguments for REQUEST_PLAY_SOUND")
                        sendRequest(client_socket, [TopServerCallback.CALLBACK_WRONG_ARGUMENTS])
                        # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_WRONG_ARGUMENTS.value]))

                    soundName = request_params[0]
                    print("soundName : ", soundName)
                    
                    self.speaker.playSound(soundName)
                    sendRequest(client_socket, [TopServerCallback.CALLBACK_OK])
                    # client_socket.sendall(pickle.dumps([TopServerCallback.CALLBACK_OK.value]))


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
        self.roslaunchThread.start()
        
        # listen on port
        try:
            self.serverSocket.bind( ("0.0.0.0", TOPSERVER_PORT) )
        except OSError:
            print("ERROR : Adress already in use, shutting down TopServer")
            self.closeServer(exitCode=1)
            return
            
        maxclients = 20  #TODO ensure
        self.serverSocket.listen(maxclients)
                
        try:
            while True:
                        
                #### SERVER PART ####
                
                # Accept incoming connection
                client_socket, client_address = self.serverSocket.accept()  # NOTE blocking
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
        # self.rosLaunchStopEvent.set()
        self.roslaunchThread.stopThread()
        
        # close speaker
        self.speaker.stop()
            
        print(f"Exiting TopServer with exit code {exitCode}")
        sys.exit(exitCode)
        

def main():
    
    topserver = TopServer()
        
    # try:
    #     allowSpeakerOutput = subprocess.check_output(['export', 'XDG_RUNTIME_DIR=/run/user/1000;', '/usr/bin/pactl', 'load-module', 'module-native-protocol-tcp' 'port=34567']).decode('utf-8')
    #     # allowSpeakerOutput = subprocess.check_output(['pactl', 'load-module', 'module-native-protocol-tcp', 'port=34567']).decode('utf-8')
    #     print("allowSpeakerOutput : ", allowSpeakerOutput)
    # except subprocess.CalledProcessError:
    #     print("ERROR in allowSpeakerOutput")
    
    topserver.speaker.setMute(False)
    # topserver.speaker.setVolume(80)

    topserver.speaker.playSound('windowsStartup')
    
    signal.signal(signal.SIGINT, topserver.sig_handler)
    
    topserver.initButton()
    
    topserver.run()

    return
    
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