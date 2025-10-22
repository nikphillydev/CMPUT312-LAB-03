# RUN ON BRICK

# Authors:
# Nikolai Philipenko

import socket

RECEIVE_BYTE_SIZE = 1024

class ArmClient:
    """This class handles the client communication on the EV3DEV computer. It connects to the server on the host PC and can listen for robot arm movement requests."""
    def __init__(self, host, port):
        '''Setup a client to connect to server at host IP and port.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        pass
    
    def receive_angles(self):
        '''Perform a blocking receive call for a movement request from the server and return the received angles (radians).'''
        print("Client waiting for angles...")
        string_data = self.sock.recv(RECEIVE_BYTE_SIZE).decode()
        str_theta1, str_theta2 = string_data.split(',')
        angles = [float(str_theta1), float(str_theta2)]
        print("Client received angles (radians): " + str(angles))
        return angles
    
    def send_ack(self):
        '''Send the ACK message back to the server for client receive acknowledgement.'''
        print("Client sending receive acknowledgement")
        self.sock.send("ACK".encode())