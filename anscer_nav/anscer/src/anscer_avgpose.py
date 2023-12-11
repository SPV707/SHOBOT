#!/usr/bin/env python  
# -- coding: UTF-8 --

import rospy
import requests
import json
import socket
import subprocess
import threading
from geometry_msgs.msg import Pose


class MoveAncer:
    def __init__(self):
        rospy.init_node('anscer_pos_node')
        self.pos_pub=rospy.Publisher('anscer_pos',Pose,queue_size=10)
        self.BUFFER_SIZE = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.addr = None
        self.token = {}
        self.xp_values = []
        self.yp_values = []
        self.zo_values = []

    def launch_js_file(self):
        def run_js_file():
            try:
                # Execute the JavaScript file using Node.js
                subprocess.run(["node", "/home/sarvesh/anscer_code/dtest.js"], check=True)
            except subprocess.CalledProcessError as e:
                # Handle any errors that occur during execution
                print(f"Error: {e}")
            except FileNotFoundError:
                # Handle the case when Node.js is not installed or not in the system's PATH
                print("Node.js not found. Please make sure it is installed.")
        
        # Create a new thread and run the JS file in the background
        thread = threading.Thread(target=run_js_file)
        thread.start()

    def start_node(self):
        tcp_ip = '127.0.0.1'
        TCP_PORT = 7777
        self.s.bind((tcp_ip, TCP_PORT))
        self.s.listen(1)
        print(f"Waiting for data on {tcp_ip}:{TCP_PORT}...")
        self.conn, self.addr = self.s.accept()
        print('Connection address:', self.addr)

    def close_node(self):
        self.conn.close()

    def connect_to_robot(self):
        url = "http://192.168.1.3:80/api/v1/auth/"
        payload = json.dumps({"email": "cnde@iitm.org", "password": "shopping@123"})
        headers = {
            'x-auth-token': 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJfaWQiOiI2NDFjYjE1NTE2MGE4YzM0MDI2NTI1YTUiLCJuYW1lIjoiY25kZS1paXRtIiwiZW1haWwiOiJjbmRlQGlpdG0ub3JnIiwicm9sZSI6ImFkbWluIiwiaWF0IjoxNjc5NzQ3NDAzLCJleHAiOjE2ODAzNTIyMDN9.-NdaLNBE23iR40OjKzyCw3Zi6N7kx__2jGsqqqsWheM',
            'Content-Type': 'application/json'}
        response = requests.request("POST", url, headers=headers, data=payload)
        Response = response.json()
        self.token = Response["token"]
        print(self.token)

        url = "http://192.168.1.3:80/api/v1/waypoints/"
        payload = {}
        headers = {'x-auth-token': self.token, 'Content-Type': 'application/json'}
        response = requests.request("GET", url, headers=headers, data=payload)

    def pos_anscer(self):
        rospy.sleep(1)
        count = 0  # Counter for recording values
        # check for mission accomplishment
        while count < 5:
            data = self.conn.recv(self.BUFFER_SIZE).decode('utf-8').strip()
            if not data:
                continue
            try:
                parsed_data = json.loads(data)
                pos_info = parsed_data.get('position')
                ort_info = parsed_data.get("orientation")
                if pos_info:
                    xp = pos_info.get('x')
                    yp = pos_info.get('y')
                    zp = pos_info.get('z')
                    xo = ort_info.get('x')
                    yo = ort_info.get('y')
                    zo = ort_info.get('z')
                    wo = ort_info.get('w')
                    pose = [xp, yp, zp, xo, yo, zo, wo]
                    self.xp_values.append(xp)
                    self.yp_values.append(yp)
                    self.zo_values.append(zo)
                    count += 1
                    print(f"Recorded values: {count}/5")
                    rospy.sleep(1)
            except json.JSONDecodeError as e:
                continue

        avg_xp = sum(self.xp_values) / len(self.xp_values)
        avg_yp = sum(self.yp_values) / len(self.yp_values)
        avg_zo = sum(self.zo_values) / len(self.zo_values)
        print(f"Average of xo values: {avg_xp}")
        print(f"Average of yo values: {avg_yp}")
        print(f"Average of zo values: {avg_zo}")
        pose_msg = Pose()
        pose_msg.position.x = avg_xp
        pose_msg.position.y = avg_yp
        pose_msg.orientation.z = avg_zo
        self.pos_pub.publish(pose_msg)
        rospy.sleep(1)
        rospy.signal_shutdown('Shutting down')

def main():
    ancer = MoveAncer()
    ancer.connect_to_robot()
    print('connecting is establish with ancer platform')
    ancer.launch_js_file()
    print('started node.js')
    ancer.start_node()
    print('connecting is establish with node.js')
    ancer.pos_anscer()
    # ancer.close_node()


if __name__ == '__main__':
    main()