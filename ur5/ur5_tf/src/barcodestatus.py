import rospy
import socket
from std_msgs.msg import Int16
import json

class Barcode:
    def __init__(self):
        rospy.init_node('barcode_node')
        self.pub = rospy.Publisher('barcode_status', Int16, queue_size=10)

    def receiver(self):
        receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        receiver_ip = '127.0.0.1'
        receiver_port = 8800
        receiver_socket.bind((receiver_ip, receiver_port))
        receiver_socket.listen(1)
        print('Waiting for a connection...')
        connection, address = receiver_socket.accept()
        print('Connected to:', address)
        data = connection.recv(1024)  # Receive data
        message = data.decode()  # Decode the received bytes into a string
        print("Received message:", message)
        if message == 'True':
            self.pub.publish(1)
        # connection.close()
        # receiver_socket.close()

def main():
    bar = Barcode()
    bar.__init__()
    bar.receiver()

if __name__ == '__main__':
    main()
