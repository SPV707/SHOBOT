#!/usr/bin/env python  
# -- coding: UTF-8 --

import rospy
import requests
import json
import socket
import subprocess
import threading
from std_msgs.msg import Int16



class MoveAncer:
    def __init__(self):
        rospy.init_node('anscer_node')
        self.pub=rospy.Publisher('completion_a',Int16,queue_size=10)
        self.waylist = {'entrance': "641ef86462de1cca49803d9e", 'Home': "641ef77562de1cca49803d93",
                        'Shelf2': "641ee840517c8c093968522b", 'shelf1-cust': "641ee8dc517c8c0939685231"}
        self.mission = {'mission_test': "641ee88d3cf3ac77d089d1cd"}
        self.BUFFER_SIZE = 1024
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.addr = None
        self.token = {}
    
    def launch_js_file(self):
        def run_js_file():
            try:
                # Execute the JavaScript file using Node.js
                subprocess.run(["node", "/home/sarvesh/anscer_code/dtest2.js"], check=True)
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
        TCP_PORT = 8888
        self.s.bind((tcp_ip, TCP_PORT))
        self.s.listen(1)
        print(f"Waiting for data on {tcp_ip}:{TCP_PORT}...")
        self.conn, self.addr = self.s.accept()
        print('Connection address:', self.addr)
        # os.system('node /home/shobot/Downloads/test/test2.js')
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

        # get waypoint
        url = "http://192.168.1.3:80/api/v1/waypoints/"

        payload = {}
        headers = {'x-auth-token': self.token, 'Content-Type': 'application/json'}
        response = requests.request("GET", url, headers=headers, data=payload)

        # self.waylist = ["641ef86462de1cca49803d9e", "641ee840517c8c093968522b"]
        self.waylist = {'entrance': "641ef86462de1cca49803d9e", 'Home': "641ef77562de1cca49803d93",
                        'Shelf2': "641ee840517c8c093968522b", 'shelf1-cust': "641ee8dc517c8c0939685231"}
        
        self.mission = {'mission_Delivery': "641ef29f10e6b156bbaae281", 'mission_Home': "641ef2c910e6b156bbaae288",
                        'mission_Pickup': "641ef2e010e6b156bbaae28f", 'Mission_ItemShelf': '641eef36e8943e76649be9c8','shelf_1': '641ef2f210e6b156bbaae296',
                        'Shelf_1': '641eee99b516494e96b4a43e', 'shelf_2': '641ef44c6dc5c456d25ad966',
                        'shelf_3': '641ef4976dc5c456d25ad96c', 'shelf_4': '641ef5126dc5c456d25ad972'}
        # create mission_ent -entrance, mission_cart - cart

    def execute_mission(self, mission):
        # mission_test = 'mission_test'
        url = f"http://192.168.1.3:80/api/v1/missions/initiate/{mission}"
        payload = "{\n    \"iteration\":1\n}"
        headers = {'x-auth-token': self.token, 'Content-Type': 'application/json'}
        response = requests.request("POST", url, headers=headers, data=payload)
        print(response.text)

    def completion_callback(self, msg):
        if msg.data == 100:
            print("UR5 completed",msg.data)
            self.move_ancer_home()

    def move_ancer_shelf(self):
        self.execute_mission(self.mission['Mission_ItemShelf'])
        print('moving to shelf to pick item')
        rospy.sleep(1)
        # check for mission accomplishment
        while 1:
            data = self.conn.recv(self.BUFFER_SIZE).decode('utf-8').strip()
            # print(data)
            if not data: continue
            try:
                parsed_data = json.loads(data)
                mission_info = parsed_data.get('mission_info')
                if mission_info:
                    current_task_index = mission_info.get('current_task_index')
                    completion_percentage = mission_info.get('completion_percentage')
                    total_loops = mission_info.get('total_loops')
                    current_loop_index = mission_info.get('current_loop_index')
                    mission_info_values = [current_task_index, completion_percentage, total_loops, current_loop_index]
                    # print(mission_info_values)
                    if (mission_info_values[1] == 100): 
                        # self.pub.publish(completion_percentage)
                        rospy.sleep(10)
                        def run_pos():
                            # Run the desired ROS node using rosrun
                            node_name = 'anscer_avgpose.py'
                            package_name = 'anscer'
                            command = ['rosrun', package_name, node_name]
                            subprocess.Popen(command)
                        thread2 = threading.Thread(target=run_pos)
                        thread2.start()

                        def run_ur5():
                            # Run the desired ROS node using rosrun
                            node_name = 'ur5_anscer'
                            package_name = 'ur5_tf'
                            command = ['rosrun', package_name, node_name]
                            subprocess.Popen(command)
                        thread3 = threading.Thread(target=run_ur5)
                        thread3.start()                                                                                                 
                        break

            except json.JSONDecodeError as e:
                # print(f"Failed to parse data: {e}")
                continue
        print('Reached shelf location')
        rospy.Subscriber('completion_u', Int16, self.completion_callback)
        rospy.spin()
    
    def move_ancer_home(self):
        # self.execute_mission(self.mission['mission_Delivery'])
        print('moving to home')
        rospy.sleep(1)
        # check for mission accomplishment
        while 1:
            data = self.conn.recv(self.BUFFER_SIZE).decode('utf-8').strip()
            # print(data)
            if not data: continue
            try:
                parsed_data = json.loads(data)
                mission_info = parsed_data.get('mission_info')
                if mission_info:
                    current_task_index = mission_info.get('current_task_index')
                    completion_percentage = mission_info.get('completion_percentage')
                    total_loops = mission_info.get('total_loops')
                    current_loop_index = mission_info.get('current_loop_index')
                    mission_info_values = [current_task_index, completion_percentage, total_loops, current_loop_index]
                    # print(mission_info_values)
                    if (mission_info_values[1] == 100): 
                        break
            except json.JSONDecodeError as e:
                # print(f"Failed to parse data: {e}")
                continue
        print('Reached home location')
        rospy.signal_shutdown('Shutting down')


def main():
    ancer = MoveAncer()
    ancer.__init__()
    ancer.connect_to_robot()
    print('connecting is establish with ancer platform')
    ancer.launch_js_file()
    print('started node.js')
    ancer.start_node()
    print('connecting is establish with node.js')
    ancer.move_ancer_shelf()


if __name__ == '__main__':
    main()
