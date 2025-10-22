# RUN ON HOST

# Authors:
# Nikolai Philipenko

import socket

RECEIVE_BYTE_SIZE = 1024

class ArmServer:
    """This class handles the server communication on the host PC. It accepts the connection to the client and can send movement requests."""
    def __init__(self, host, port):
        '''Setup a server with host IP and port.'''
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.bind((host, port))
        self.server_sock.listen(5)
        print("Server listening for connections...")
        self.client_sock, self.client_addr = self.server_sock.accept()
        print("Server accepted client connection!")
        pass
    
    def send_angles(self, base_angle, joint_angle):
        '''Send a robot arm movement request to the client for joint angles (radians).'''
        print("Server sending joint angles (radians): " + str(base_angle) + " " + str(joint_angle))
        data = str(base_angle) + ',' + str(joint_angle)
        self.client_sock.send(data.encode())
        reply = self.client_sock.recv(RECEIVE_BYTE_SIZE).decode()
        print ("Server received reply: " + reply)