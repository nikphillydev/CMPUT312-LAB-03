# RUN ON BRICK
    
import socket

RECEIVE_BYTE_SIZE = 1024

class ArmClient:
    """This class handles the client communication on the EV3DEV computer. It connects to the server and can listen for movement requests."""
    def __init__(self, host, port):
        '''Setup a client to connect to server at host IP and port.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        pass
    
    def receive_angles(self):
        '''Performs a blocking receive call for a movement request and returns the received angles (radians).'''
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
    
    

# # This class handles the client side of communication. It has a set of predefined messages to send to the server as well as functionality to poll and decode data.
# class Client:
#     def __init__(self, host, port):
#         # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
#         print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
#         self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
#         self.s.connect((host, port))                               
        
#     # Block until a message from the server is received. When the message is received it will be decoded and returned as a string.
#     # Output: UTF-8 decoded string containing the instructions from server.
#     def pollData(self):
#         print("Waiting for Data")
#         data = self.s.recv(128).decode("UTF-8")
#         print("Data Received")
#         return data
    
#     # Sends a message to the server letting it know that the movement of the motors was executed without any inconvenience.
#     def sendDone(self):
#         self.s.send("DONE".encode("UTF-8"))

#     # Sends a message to the server letting it know that there was an isse during the execution of the movement (obstacle avoided) and that the initial jacobian should be recomputed (Visual servoing started from scratch)
#     def sendReset(self):
#         self.s.send("RESET".encode("UTF-8"))


# host = "169.254.150.69"
# port = 10016
# client = Client(host, port)
# i = 0
# while True:
#     print(client.pollData())
#     time.sleep(1)
#     client.sendDone()