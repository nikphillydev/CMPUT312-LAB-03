# RUN ON LAPTOP

import socket

RECEIVE_BYTE_SIZE = 1024

class ArmServer:
    """This class handles the server communication on the host computer. It accepts the connection to the client and can send movement requests."""
    def __init__(self, host, port):
        '''Setup a server with host IP and port.'''
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.bind(host, port)
        self.server_sock.listen(5)
        self.client_sock, self.client_addr = self.server_sock.accept()
        self.connected = True
        pass
    
    def send_angles(self, base_angle, joint_angle):
        '''Send a movement request to the client for joint angles (radians).'''
        print("Server sending joint angles (radians): " + str(base_angle) + " " + str(joint_angle))
        data = str(base_angle) + ',' + str(joint_angle)
        self.client_sock.send(data.encode())
        reply = self.client_sock.recv(RECEIVE_BYTE_SIZE).decode()
        print ("Server received reply: " + reply)
        



# # This class handles the Server side of the comunication between the laptop and the brick.
# class Server:
#     def __init__(self, host, port):
#        # setup server socket
#         serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
#         # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter that handles the comunication between the PC and the brick
#         print("Setting up Server\nAddress: " + host + "\nPort: " + str(port))
        
#         serversocket.bind((host, port))
#         # queue up to 5 requests
#         serversocket.listen(5) 
#         self.cs, addr = serversocket.accept()
#         print ("Connected to: " + str(addr))

#     # Sends set of angles to the brick via TCP.
#     # Input: base_angle [Float]: The angle by which we want the base to move
#     #        joint_angle [Float]: The angle by which we want to joint to move
#     #        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
#     def sendAngles(self, base_angle, joint_angle, queue):
#         # Format in which the client expects the data: "angle1,angle2"
#         data = str(base_angle) + "," + str(joint_angle)
#         print("Sending Data: (" + data + ") to robot.")
#         self.cs.send(data.encode("UTF-8"))
#         # Waiting for the client (ev3 brick) to let the server know that it is done moving
#         reply = self.cs.recv(128).decode("UTF-8")
#         queue.put(reply)

#     # Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors.
#     def sendTermination(self):
#         self.cs.send("EXIT".encode("UTF-8"))

#     # Lets the client know that it should enable safety mode on its end
#     def sendEnableSafetyMode(self):
#         self.cs.send("SAFETY_ON".encode("UTF-8"))
    
#     # Lets the client know that it should disable safety mode on its end
#     def sendDisableSafetyMode(self):
#         self.cs.send("SAFETY_OFF".encode("UTF-8"))



# host = "169.254.150.69"
# port = 10016
# server = Server(host, port)
# queue = Queue()

# while True:
#     time.sleep(10)
#     server.sendAngles(10, 10, queue)