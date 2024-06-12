import json
import socket
import sys
import time
import traceback
import threading
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion, Twist
from simple_msgs.msg import TrajectoryData, EgoRecording, OpponentRecording




def log_error(node):
    msg = 'Exception: {}\n{}'.format(time.asctime(), traceback.format_exc())
    node.get_logger().info(msg) 

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def static_publisher(raw, node):
    #rospy.loginfo([type(raw[k]) for k in raw.keys()])
    pub = node.create_publisher( TrajectoryData,"trajectorydata", 10)
    rate = node.create_rate(0.5)
    data = TrajectoryData()    
    # Invertiti 1 e 0 righe successive
    data.left_lane = PoseArray(header=None, poses=[Pose(orientation=None, position=Point(x=el[0],y=el[1],z=0)) for el in raw['left_lane']])
    data.right_lane = PoseArray(header=None, poses=[Pose(orientation=None, position=Point(x=el[0],y=el[1],z=0)) for el in raw['right_lane']])
    data.fast_lane = PoseArray(header=None, poses=[Pose(orientation=None, position=Point(x=el[0],y=el[1],z=0)) for el in raw['fast_lane']])
    data.brake_arr = raw['brake_arr']
    data.throttle_arr = raw['throttle_arr']
    data.speed_arr = raw['speed_arr']
    while rclpy.ok():
        pub.publish(data)
        print("------- Finito iter static -------")
        rate.sleep() 

def opponents_publisher(node):

    # Opponents socket creation
    opp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    opp_sock.connect(("localhost",2346))

    # Receiving number of opponents
    num_opp = int(opp_sock.recv(1024).decode('utf8'))

    # Creating opponents dictionary
    opponents = dict()
    for i in range(1, num_opp + 1):
        topic_name = 'opp'+str(i)+'_data'
        opponents[i] = node.create_publisher(OpponentRecording, topic_name, 1)

    while rclpy.ok():

        # Recieving opponents data
        full_dict = ''
        try:
            inSock = opp_sock.recv(32).decode()
            try:
                # Buffered reading (Header - "HEADER-END" - Payload model)
                msg_len, full_dict = inSock.split("HEADER-END")
                while int(msg_len) - len(full_dict) > 4096:
                    full_dict = full_dict + opp_sock.recv(4096).decode()
                full_dict = full_dict + opp_sock.recv(int(msg_len) - len(full_dict)).decode()
            except:
                log_error(node)
                raise

            # Data dictionary creation
            raw = eval(full_dict)
            
            # Building list of messages (a message for each opponent)
            opponents_messages = []
            for el in raw:
                opp = OpponentRecording()
                opp.id = el.get("id")
                opp.speed_kmh = el.get("speedKMH")
                
                pos = el.get('world_position')
                yaw = el.get('yaw')
                opp.world_position = Pose(position=Point(x = pos[0], y = pos[1], z = pos[2]))# orientation=Quaternion(*euler_to_quaternion(0,0,yaw)))
                q = euler_to_quaternion(0,0,yaw)
                opp.world_position.orientation.x = q[0]
                opp.world_position.orientation.y = q[1]
                opp.world_position.orientation.z = q[2]
                opp.world_position.orientation.w = q[3]
                opponents_messages.append(opp)

            # Publishing messages on relative topics with a single time stamp
            h = Header()
            h.stamp = node.get_clock().now().to_msg() 
            for el in opponents_messages:
                el.header = h
                opponents[el.id].publish(el)
            
            node.get_logger().info("----- Finito iter opponents -----")

        except:
            log_error(node)
            raise

def main():
    # Ego socket creation
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("localhost",2345))

    # ROS node initialization
    rclpy.init()
    node = rclpy.create_node('simulation_data')
    # ROS ego car topic initialization
    pub = node.create_publisher(EgoRecording,"ego_data", 1)
    rate = node.create_rate(50)
    
    # Opponents task initialization and start
    try:
        t = threading.Thread(target=opponents_publisher, args=(node,))
        t.daemon = True
        t.start()
    except:
        log_error(node)
        raise
    

    # -------------------- Static data section --------------------#
    # Receiving static data regarding track
    # Buffered reading (Header - "HEADER-END" - Payload model)
    static_raw = dict()
    inSock = sock.recv(128).decode()

    msg_len, full_dict = inSock.split("HEADER-END")
    while int(msg_len) - len(full_dict) > 4096:
        full_dict = full_dict + sock.recv(4096).decode()
    full_dict = full_dict + sock.recv(int(msg_len) - len(full_dict)).decode()

    static_raw = eval(full_dict)
    # Static data task initialization and start
    '''try:
        t = threading.Thread(target=static_publisher, args=(static_raw,))
        t.daemon = True
        t.start()
    except:
        log_error(node)
        raise'''

    # -------------------- End static data section --------------------#
    # -------------------- Dynamic data section --------------------#
    raw = dict()
    msg_len = 0

    #time_start = time.time()
    # Receiving dynamic data about car and publish on Topic
    while rclpy.ok():

        full_dict = ''
        time_start = time.time()
        try:
            inSock = sock.recv(32).decode()
            try:
                # Buffered reading (Header - "HEADER-END" - Payload model)
                msg_len, full_dict = inSock.split("HEADER-END")
                
                while int(msg_len) - len(full_dict) > 4096:
                    full_dict = full_dict + sock.recv(4096).decode()
                full_dict = full_dict + sock.recv(int(msg_len) - len(full_dict)).decode()
            except:
                node.get_logger().info(inSock)

            # Data dictionary creation
            raw = eval(full_dict)

            n_opponents = int(raw.get('n_opponents', 0))

            # Message building
            msg = EgoRecording()

            msg.car_id = int(raw.get('car_id', 0))
            msg.leaderboard_position = int(raw.get('realtime_leaderboard_position',0))
            msg.driver_name = str(raw.get('driver_name', 'not_found'))

            msg.normalized_spline_position = float(raw.get('NormalizedSplinePosition',0))
            msg.lap_count = int(raw.get('lap_count',-1))
            msg.best_lap = int(raw.get('best_lap',0.0))

            pos = raw.get('world_position')
            yaw = raw.get('yaw')
            roll = raw.get('roll')
            pitch = raw.get('pitch')

            f, u, l = raw.get('velocity_vector')


            to_pub = Pose(position=Point(x = pos[0], y = pos[1], z = pos[2])) #, orientation=Quaternion(0,0,yaw,0)
            to_pub.orientation.x = 0.0
            to_pub.orientation.y = 0.0
            to_pub.orientation.z = yaw
            to_pub.orientation.w = 0.0

            velocity = raw.get('velocity')
            angular_velocity = raw.get('angular_velocity')
            twist = Twist()
            twist.linear.x = float(velocity[2])
            twist.linear.y = float(velocity[0])
            twist.linear.z = float(velocity[1])
            twist.angular.x = float(angular_velocity[2])
            twist.angular.y = float(angular_velocity[0])
            twist.angular.z = float(angular_velocity[1])

            global_velocity = Twist()
            global_velocity.linear.x = float(l)
            global_velocity.linear.y = float(f)
            global_velocity.linear.z = float(u)

            msg.world_position = to_pub
            msg.speed_kmh = float(raw.get('speedKMH', 0.0))
            msg.local_velocity = twist
            msg.global_velocity = global_velocity
            msg.accel_x = float(raw.get('accelY',0.0))*9.81
            msg.accel_y = float(raw.get('accelX',0.0))*9.81
            msg.steer_angle = float(raw.get('steerAngle', 0.0))
            msg.gas_pedal_sts = float(raw.get('gasPedalSts', 0))
            msg.brake_pedal_sts = float(raw.get('brakePedalSts', 0))
            msg.actual_gear = int(raw.get('actualGear', 0))
            msg.rpm = int(float(raw.get('RPM', 0)))
            msg.cg_height = float(raw.get('cgHeight',0))
            # Tyres data
            msg.slipangle_fl, msg.slipangle_fr, msg.slipangle_rl, msg.slipangle_rr = [float(el) for el in raw.get('tyre_slip_angle',0.0)]
            msg.slipratio_fl, msg.slipratio_fr, msg.slipratio_rl, msg.slipratio_rr = [float(el) for el in raw.get('tyre_slip_ratio',0.0)]
            msg.mz_fl, msg.mz_fr, msg.mz_rl, msg.mz_rr = [float(el) for el in raw.get('Mz',0.0)]
            msg.dy_fl, msg.dy_fr, msg.dy_rl, msg.dy_rr = [float(el) for el in raw.get('Dy',0.0)]
            msg.pressure_fl, msg.pressure_fr, msg.pressure_rl, msg.pressure_rr = [float(el) for el in raw.get('dynamic_pressure',0.0)]
            msg.loadedradius_fl, msg.loadedradius_fr, msg.loadedradius_rl, msg.loadedradius_rr = [float(el) for el in raw.get('tyre_loaded_radius',0.0)]
            msg.load_fl, msg.load_fr, msg.load_rl, msg.load_rr = [float(el) for el in raw.get('tyres_load',0.0)]
            msg.nd_slip_fl, msg.nd_slip_fr, msg.nd_slip_rl, msg.nd_slip_rr = [float(el) for el in raw.get('NdSlip',0.0)]
            msg.suspensiontravel_fl, msg.suspensiontravel_fr, msg.suspensiontravel_rl, msg.suspensiontravel_rr = [float(el) for el in raw.get('SuspensionTravel',0.0)]
            msg.camber_fl, msg.camber_fr, msg.camber_rl, msg.camber_rr = [float(el) for el in raw.get('CamberRad',0.0)]

            msg.dist_l = float(raw.get('dist_l', 0.0))
            msg.dist_r = float(raw.get('dist_r', 0.0))

            
            h = Header()
            h.stamp = node.get_clock().now().to_msg()
            
            msg.header = h
            pub.publish(msg)

            node.get_logger().info("----- Finito iter ego -----")

            #time_start = time.time()

            
            #rate.sleep()
            #print("--------------------------------------time after: ", (time.time() - time_start)*1000)
            #rospy.loginfo("Done parsing data " + str(n_opponents))
        except:
            log_error(node)
            node.get_logger().info("Error parsing data")
            sys.exit()


    sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
