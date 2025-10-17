from VSMaterial.client import Client
import time

FREQUENCY = 1

host = "169.254.64.237"
port = 10016
client = Client(host, port)


while True:
    data = client.pollData()
    data = data.strip()
    x_str, y_str = data.split(',')
    x = int(x_str)
    y = int(y_str)
    print("x:", x, "y:", y)
    time.sleep(1 / FREQUENCY)
    client.sendDone()
