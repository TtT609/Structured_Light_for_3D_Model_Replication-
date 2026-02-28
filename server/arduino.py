import time
import serial
import serial.tools.list_ports

class ArduinoController:
    # Class for managing and controlling a turntable via connection port to an Arduino (or ESP32) board
    def __init__(self):
        # Initialization function when creating the class. Defines that the serial port is not yet connected (set to None initially)
        self.ser = None

    def get_ports(self):
        # Function to scan and return a list of connection ports (COM Port / /dev/tty) 
        # currently plugged in and visible on this computer
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=115200):
        # Function to open a connection to a specified target port at a baud rate of 115200 (for ESP32 chips)
        try:
            # Connect to the Serial port with a timeout; if opening fails within 2 seconds, consider it a failure
            self.ser = serial.Serial(port, baudrate, timeout=2)
            # Once connected, the board may restart itself, so wait for 2 seconds for the board to become ready
            time.sleep(2)  
            # If connected without issues, return True with a completion message
            return True, "Connected" 
        except Exception as e:
            # Catch errors to prevent a program crash and report False
            return False, str(e)

    def disconnect(self):
        # Function to cancel and close the Arduino board connection
        if self.ser: # Check if the system is still connected (self.ser is not None)
            self.ser.close() # Command to close the Serial signal port
            self.ser = None  # Clear the port variable cache back to a disconnected state (None)

    def rotate(self, degrees):
        # Function to send a command to rotate the motor by specified degrees
        if not self.ser: return False # If the port is not open, return failure immediately
        try:
            # Create a command string: the degree value followed by a newline (\n) so Arduino knows the message ends
            cmd = f"{degrees}\n"
            # Send message via Serial; it must be binary encoded using .encode() first
            self.ser.write(cmd.encode())
            # Return success status
            return True 
        except:
            # Return failure status for any errors
            return False 

    def wait_for_done(self, timeout=30):
        # Pause the scanning system and wait until Arduino replies that rotation has finished (max wait 30 seconds)
        if not self.ser: return False # Check if the port connection is still active
        
        start = time.time() # Record the start time
        buffer = "" # Variable for accumulating messages
        
        # Loop continuously to read data until the 30s timeout is reached
        while time.time() - start < timeout: 
            if self.ser.in_waiting: # If there is incoming data from Arduino
                try:
                    # Read incoming data until a newline is found, then strip whitespace
                    line = self.ser.read_until().decode().strip()
                    # If "DONE" is found, motor rotation is complete; exit the loop and proceed
                    if "DONE" in line: return True
                except: 
                    pass # Prevent encoding errors from garbage data; skip and wait for next line
            
            # Sleep for 0.1 seconds before the next loop to reduce CPU usage and stabilize the system
            time.sleep(0.1) 
            
        # If the loop finishes after 30 seconds without exiting, the Arduino timed out
        return False 
