import json
import socket
import sys
import time
import traceback
import threading
import math
import numpy as np
import pandas

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, PoseArray, Point, Pose, PoseWithCovariance, Quaternion, Twist
from deep_orange_msgs.msg import TireReport, PtReport, MiscReport, CtReport, RcToCt
from ac_actuation_msgs.msg import WheelSpeedReport, Brake2Report, AcceleratorPedalReport, SteeringReport, BrakeReport
from common_msgs.msg import VehicleState, DetectedVehicle, DetectedVehicles, TrajectoryData
from simulator_output.simulator_output_script import euler_to_quaternion
from pprint import pprint

class TrackCondition(object):
  TC1_RED = 1 # start engine, perform any final checks
  TC2_ORANGE = 2 # Proceed with caution
  TC3_YELLOW = 3 # Race
  TC4_GREEN = 4
  TC_DEFAULT = 255

class VehicleSignal(object):
    VS1_NULL = 1
    VS2_BLACK = 2
    VS4_CHECK = 4
    VS8_PURPLE = 8

class SimulatorOutput(Node):
    def __init__(self):
        super().__init__('simulator_output')
        
        self.declare_parameter('no_opponents', True)
        self.no_opponents = self.get_parameter("no_opponents")

        #TO-DO: parameters
        self.ego_length = 4.9215
        self.ego_width = 1.8860
        self.ego_height = 1.1565
        self.ct_report = CtReport()
        self.counter = 0
        self.current_gear = 1

        # Publishers 
        self.publisher_wheel = self.create_publisher(WheelSpeedReport, "ac_actuation/wheel_speed_report", 1) 
        self.publisher_brake2_report = self.create_publisher(Brake2Report, "ac_actuation/brake_2_report", 1)
        self.publisher_brake = self.create_publisher(BrakeReport, "ac_actuation/brake_report", 1)
        self.publisher_acc = self.create_publisher(AcceleratorPedalReport, "ac_actuation/accelerator_pedal_report", 1)
        self.publisher_steer = self.create_publisher(SteeringReport, "ac_actuation/steering_report", 1)
        self.publisher_tire = self.create_publisher(TireReport, "ac_actuation/tire_report", 1)
        self.publisher_pt = self.create_publisher(PtReport, "ac_actuation/pt_report", 1)
        self.publisher_misc = self.create_publisher(MiscReport, "ac_actuation/misc_report_do", 1)
        self.publisher_vehicle_state = self.create_publisher(VehicleState,"vehicle_state", 1)
        self.publisher_opp_state = self.create_publisher(DetectedVehicles, "detected_vehicles", 1)
        self.publisher_track = self.create_publisher(TrajectoryData, "trajectory_data", 1)
        self.publisher_ct = self.create_publisher(CtReport, "ac_actuation/ct_report", 1)

        # Timers
        timer_period_100 = 1/125  # seconds
        timer_period_25 = 0.04  # seconds
        timer_period_10 = 0.1  # seconds
        self.timer_wheel = self.create_timer(timer_period_100, self.publish_wheel)
        self.timer_brake = self.create_timer(timer_period_100, self.publish_brake)
        self.timer_brake2_report = self.create_timer(timer_period_100, self.publish_brake2_report)
        self.timer_accelerator = self.create_timer(timer_period_100, self.publish_accelerator)
        self.timer_steer = self.create_timer(timer_period_100, self.publish_steer)
        self.timer_tire = self.create_timer(timer_period_100, self.publish_tire)
        self.timer_pt = self.create_timer(timer_period_10, self.publish_pt)
        # self.timer_misc = self.create_timer(timer_period_10, self.publish_misc)
        self.timer_vehicle_state = self.create_timer(timer_period_100, self.publish_vehicle_state)
        # self.timer_opp_state = self.create_timer(timer_period_10, self.publish_opp_state)
        # self.timer_track = self.create_timer(1.0, self.publish_track) # 1Hz
        # self.timer_ct = self.create_timer(timer_period_25, self.publish_ct_report)

        # Create thread to keep the data dict updated
        self.iac_data = dict()
        self.ego_data = dict()
        self.opp_data = dict()

        # create sockets
        self.sock_iac = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_ego = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_opp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.last_ego_position_index = -1

        self.sock_ego.connect(("localhost",2345))
        print("Connected to ego socket")
        if self.no_opponents.value is False:
            self.sock_opp.connect(("localhost",2346))
            print("Connected to opponents socket")
        self.sock_iac.connect(("localhost",2347))
        print("Connected to iac socket")

        try:
            t_iac = threading.Thread(target=self.listen_iac, daemon = True)
            t_ego = threading.Thread(target=self.listen_ego, daemon = True)
            if self.no_opponents.value is False:
                t_opp = threading.Thread(target=self.listen_opp, daemon = True)
                t_opp.start()
            t_iac.start()
            t_ego.start()
        except:
            self.log_error()
            raise


    # Listener for the socket
    def listen_iac(self):
        print("Receiving data on IAC socket...")
        while rclpy.ok():
            #IAC msgs
            inSock = self.sock_iac.recv(128).decode()
            msg_len, full_dict = inSock.split("HEADER-END")

            while int(msg_len) - len(full_dict) > 4096:
                full_dict = full_dict + self.sock_iac.recv(4096).decode()
            full_dict = full_dict + self.sock_iac.recv(int(msg_len) - len(full_dict)).decode()
            
            try:
                self.iac_data = eval(full_dict)
                #self.get_logger().info(str(len(data_dict.keys())))
            except:
                print("error socket listen_iac")
                self.log_error()
                self.sock_iac.close()
                #print(full_dict)
        

    def listen_ego(self):
        print("Receiving data on ego socket...")
        self.static_raw = dict() # track bounds
        inSock = self.sock_ego.recv(128).decode()

        msg_len, full_dict = inSock.split("HEADER-END")
        while int(msg_len) - len(full_dict) > 4096:
            full_dict = full_dict + self.sock_ego.recv(4096).decode()
        full_dict = full_dict + self.sock_ego.recv(int(msg_len) - len(full_dict)).decode()

        self.static_raw = eval(full_dict)

        while rclpy.ok():
            #EGO msgs
            inSock = self.sock_ego.recv(32).decode()
            msg_len, full_dict = inSock.split("HEADER-END")

            while int(msg_len) - len(full_dict) > 4096:
                full_dict = full_dict + self.sock_ego.recv(4096).decode()
            full_dict = full_dict + self.sock_ego.recv(int(msg_len) - len(full_dict)).decode()
            
            try:
                self.ego_data = eval(full_dict)
                #self.get_logger().info(str(len(data_dict.keys())))
            except:
                print("error socket listen_ego")
                self.log_error()
                self.sock_ego.close()
                #print(full_dict)  


    def listen_opp(self):
        print("Receiving data on opponents socket...")
        num_opp = int(self.sock_opp.recv(1024).decode('utf8'))
        print("number of opponents: " + str(num_opp))
        while rclpy.ok():     
            #OPP msg       
            inSock = self.sock_opp.recv(32).decode()
            msg_len, full_dict = inSock.split("HEADER-END")

            while int(msg_len) - len(full_dict) > 4096:
                full_dict = full_dict + self.sock_opp.recv(4096).decode()
            full_dict = full_dict + self.sock_opp.recv(int(msg_len) - len(full_dict)).decode()
            
            try:
                self.opp_data = eval(full_dict)
                #self.get_logger().info(str(len(data_dict.keys())))
            except:
                print("error socket listen_opp")
                self.log_error()
                self.sock_opp.close()
                #print(full_dict)

    # Subscriber callback
    def callback_rc(self, data):
        # vehicle signal acknowledge
        if data.black[15] is True:
            self.ct_report.veh_sig_ack = VehicleSignal.VS2_BLACK
        elif data.checkered[15] is True:
            self.ct_report.veh_sig_ack = VehicleSignal.VS4_CHECK
        elif data.purple[15] is True:
            self.ct_report.veh_sig_ack = VehicleSignal.VS8_PURPLE
        else:
            self.ct_report.veh_sig_ack = VehicleSignal.VS1_NULL
        # track signal acknowledge
        self.ct_report.track_cond_ack = data.track_cond

    # Publish function for each publisher

    def publish_ct_report(self):
        self.ct_report.stamp = self.get_clock().now().to_msg()
        self.ct_report.ct_state = 9
        self.counter+=1
        self.ct_report.rolling_counter = (self.counter) % 256

        self.publisher_ct.publish(self.ct_report)

    def publish_track(self):
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        msg = TrajectoryData()    
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'map'
        msg.header = h
        msg.left_lane = PoseArray(header=Header(), poses=[Pose( orientation=q, position=Point(x=el[0],y=el[1],z=0.0)) for el in self.static_raw['left_lane']])
        msg.right_lane = PoseArray(header=Header(), poses=[Pose( orientation=q, position=Point(x=el[0],y=el[1],z=0.0)) for el in self.static_raw['right_lane']])
        msg.fast_lane = PoseArray(header=Header(), poses=[Pose( orientation=q, position=Point(x=el[0],y=el[1],z=0.0)) for el in self.static_raw['fast_lane']])
        msg.brake_arr = self.static_raw['brake_arr']
        msg.throttle_arr = self.static_raw['throttle_arr']
        #msg.speed_arr = self.static_raw['speed_arr']
        self.publisher_track.publish(msg)
    
    def publish_wheel(self):
        #print("wheel")        
        msg = WheelSpeedReport()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.front_left = self.iac_data.get("front_left", 0.0)
        msg.front_right = self.iac_data.get("front_right", 0.0)
        msg.rear_left = self.iac_data.get("rear_left", 0.0)
        msg.rear_right = self.iac_data.get("rear_right", 0.0)
        self.publisher_wheel.publish(msg)


    def publish_brake(self):
        #print("brake")
        msg = BrakeReport()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        #msg.brake_pressure = float(self.iac_data.get("brake_pressure", 0.0)) # units are bars, gauge pressure 
        msg.pedal_position = float(self.ego_data.get("brakePedalSts", 0.0)) # units are percentage 
        self.publisher_brake.publish(msg)     
    
    def publish_brake2_report(self):
        #print("brake") 
        msg = Brake2Report()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.estimated_road_slope = self.iac_data.get("estimated_road_slope", 0.0) # degrees 
        #msg.brake_pressure = float(self.iac_data.get("brake_pressure", 0.0)) # units are bars, gauge pressure 
        #TO-DO: fix front/rear split
        msg.front_brake_pressure = float(self.ego_data.get("brakePedalSts", 0.0) * (5400*0.6) ) # units are bars, gauge pressure 
        msg.rear_brake_pressure = float(self.ego_data.get("brakePedalSts", 0.0)  * (5400*0.4) ) # split brake output between front and rear by dividing per 2 
        self.publisher_brake2_report.publish(msg)
    
    def publish_accelerator(self):
        #print("accelerator")
        msg = AcceleratorPedalReport()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        #msg.pedal_input = self.iac_data.get("pedal_input", 0.0)
        msg.pedal_input = float(self.ego_data.get("gasPedalSts", 0.0)*100) # (0;1) to (0;100)
        msg.pedal_output = float(self.ego_data.get("gasPedalSts", 0.0)*100) # (0;1) to (0;100) 
        msg.rolling_counter = self.iac_data.get("acc_pedal_rolling_counter", 0)  #name should be changed in sensors_par/structures.py
        self.publisher_acc.publish(msg)
        


    def publish_steer(self):
        #print("steer")
        msg = SteeringReport()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        #msg.steering_wheel_angle = self.iac_data.get("steering_wheel_angle", 0.0)
        msg.steering_wheel_angle = float(-self.ego_data.get("steerAngle", 0.0)) # (-180;180) to (-600;600) degrees
        msg.rolling_counter = self.iac_data.get("steering_wheel_rolling_counter", 0) 
        self.publisher_steer.publish(msg)


    def publish_tire(self):
        #print("tire")
        msg = TireReport() 
        msg.stamp = self.get_clock().now().to_msg()
        msg.fl_tire_temperature.append(self.iac_data.get("fl_tire_temperature_core", 0.0))
        msg.fl_damper_linear_potentiometer = self.iac_data.get("fl_damper_linear_potentiometer", 0.0)
        msg.fl_wheel_load = self.iac_data.get("fl_wheel_load", 0.0)
        msg.fl_tire_pressure = self.iac_data.get("fl_tire_pressure", 0.0)
        msg.fr_tire_temperature.append(self.iac_data.get("fr_tire_temperature_core", 0.0))
        msg.fr_damper_linear_potentiometer = self.iac_data.get("fr_damper_linear_potentiometer", 0.0)
        msg.fr_wheel_load = self.iac_data.get("fr_wheel_load", 0.0)
        msg.fr_tire_pressure = self.iac_data.get("fr_tire_pressure", 0.0)
        msg.rl_tire_temperature.append(self.iac_data.get("rl_tire_temperature_core", 0.0))
        msg.rl_damper_linear_potentiometer = self.iac_data.get("rl_damper_linear_potentiometer", 0.0)
        msg.rl_wheel_load = self.iac_data.get("rl_wheel_load", 0.0)
        msg.rl_tire_pressure = self.iac_data.get("rl_tire_pressure", 0.0)
        msg.rr_tire_temperature.append(self.iac_data.get("rr_tire_temperature_core", 0.0))
        msg.rr_damper_linear_potentiometer = self.iac_data.get("rr_damper_linear_potentiometer", 0.0)
        msg.rr_wheel_load = self.iac_data.get("rr_wheel_load", 0.0)
        msg.rr_tire_pressure = self.iac_data.get("rr_tire_pressure", 0.0)
        self.publisher_tire.publish(msg)


    def publish_pt(self):
        #print("pt")
        msg = PtReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.map_sensor = self.iac_data.get("map_sensor", 0.0)
        msg.lambda_sensor = self.iac_data.get("lambda_sensor", 9.0)
        msg.fuel_level = self.iac_data.get("fuel_level", 0.0)
        msg.fuel_pressure = self.iac_data.get("fuel_pressure", 500.0) #kPa
        msg.engine_oil_pressure = self.iac_data.get("engine_oil_pressure", 400.0)
        msg.engine_oil_temperature = self.iac_data.get("engine_oil_temperature", 90.0) #Celsius
        msg.engine_coolant_temperature = self.iac_data.get("engine_coolant_temperature", 97.0)
        msg.engine_coolant_pressure = self.iac_data.get("engine_coolant_pressure", 500.0)
        msg.engine_rpm = self.iac_data.get("engine_rpm", 0.0)
        msg.engine_on_status = True #the socket can't pass boolean value, if crashes in the decode. It has a problem with the caps letters
        if (self.iac_data.get("current_gear") -1) != 0:
            self.current_gear = self.iac_data.get("current_gear") - 1
        msg.current_gear = self.current_gear
        msg.gear_shift_status = self.iac_data.get("gear_shift_status", 1)
        msg.transmission_oil_pressure = self.iac_data.get("transmission_oil_pressure", 1.0) #bar
        msg.transmission_accumulator_pressure = self.iac_data.get("transmission_accumulator_pressure", 2.0)
        msg.transmission_oil_temperature = self.iac_data.get("transmission_oil_temperature", 95.0)
        self.publisher_pt.publish(msg)


    def publish_misc(self):
        #print("misc")
        msg = MiscReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.battery_voltage = 13.3 #self.iac_data.get("battery_voltage", 13.0)
        msg.off_grid_power_connection = True #the socket can't pass boolean value, if crashes in the decode. It has a problem with the caps letters
        #msg.dbw_ready = self.iac_data.get("dbw_ready", 0)
        #msg.ecu_ready = self.iac_data.get("ecu_ready", 0.0)
        msg.safety_switch_state = self.iac_data.get("safety_switch_state", 0)
        msg.mode_switch_state = True #the socket can't pass boolean value, if crashes in the decode. It has a problem with the caps letters
        msg.sys_state = self.iac_data.get("sys_state", 0)
        self.publisher_misc.publish(msg)

    def publish_vehicle_state(self):        
        msg = VehicleState()

        msg.header.stamp = self.get_clock().now().to_msg()

        # Vehicle Pose
        pos = self.ego_data.get('world_position')
        yaw = self.ego_data.get('yaw')
        roll = self.ego_data.get('roll')
        pitch = self.ego_data.get('pitch')
        if pos is None:
            print("----------------------------------------------------------vehicle_state NOT AVAILABLE")
            return

        q = euler_to_quaternion(0,0,yaw)
        to_pub = Pose(position=Point(x = pos[0], y = pos[1], z = pos[2]))
        to_pub.orientation.x = q[0]
        to_pub.orientation.y = q[1]
        to_pub.orientation.z = q[2]
        to_pub.orientation.w = q[3]
        msg.w_pose.pose = to_pub

        msg.w_pose.covariance[6*0 + 0] = 0.02
        msg.w_pose.covariance[6*1 + 1] = 0.02
        msg.yaw = -yaw #TO-DO: invert in ac socket data sender

        # Speed KPH
        msg.speed = float(self.ego_data.get('speedKMH', 0.0)/3.6)
        # Local velocity
        velocity = self.ego_data.get('velocity')
        angular_velocity = self.ego_data.get('angular_velocity')
        twist = Twist()
        twist.linear.x = float(velocity[2])
        twist.linear.y = float(velocity[0])
        twist.linear.z = float(velocity[1])
        twist.angular.x = float(angular_velocity[2])
        twist.angular.y = float(angular_velocity[0])
        twist.angular.z = float(angular_velocity[1])
        msg.l_velocity.twist = twist

        # Global velocity
        f, u, l = self.ego_data.get('velocity_vector')
        global_velocity = Twist()
        global_velocity.linear.x = float(l)
        global_velocity.linear.y = float(f)
        global_velocity.linear.z = float(u)
        msg.w_velocity.twist = global_velocity

        # Accelerations
        msg.l_acceleration.accel.linear.x = float(self.ego_data.get('accelX',0.0))*9.81
        msg.l_acceleration.accel.linear.y = float(self.ego_data.get('accelY',0.0))*9.81

        self.publisher_vehicle_state.publish(msg)

    #TO-DO: Double check pos correctness
    def publish_opp_state(self):
        msg_list = DetectedVehicles()
        for el in self.opp_data:
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            msg = DetectedVehicle()
            msg.header = h
            msg.id = el.get("id")
            pos = el.get('world_position')
            
            if pos is None:
                print("----------------------------detected_vehicle NOT AVAILABLE")
                return
            yaw = el.get('yaw') -3.14
            #print("-------------- yaw opp: ", yaw)
            msg.pose.pose = Pose(position=Point(x = pos[1], y = -pos[0], z = pos[2]))
            # msg.pose.position.x = pos[0]
            # msg.pose.position.y = pos[1]
            # msg.pose.position.z = pos[2]
            q = euler_to_quaternion(0,0,yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            #msg.pose = PoseWithCovariance(pose_temp)   

            speed_ms = float(el.get('speedKMH', 0.0)) / 3.6
            msg.speed = Vector3(x = speed_ms, y = 0.0, z = 0.0)
            msg.size = Vector3(x = self.ego_length, y = self.ego_width, z = self.ego_height)
            # msg.length = self.ego_length
            # msg.width = self.ego_width
            # msg.height = self.ego_height
            # bbox = self.get_bbox(pos[0], pos[1], yaw, self.ego_length, self.ego_width)
            # corners = [msg.rear_bottom_left, 
            #            msg.front_bottom_left, 
            #            msg.front_bottom_right, 
            #            msg.rear_bottom_right]
            # for i, corner in enumerate(corners):
            #     corner.x = bbox[i, 0]
            #     corner.y = bbox[i, 1]
                
            msg.inside_track = True

            msg_list.detected_vehicles.append(msg)

        #print("detected_vehicles")
        num_opp = len(self.opp_data)
        #msg_list.num_detected = num_opp
        self.publisher_opp_state.publish(msg_list)

    def log_error(self):
        msg_err = 'Exception: {}\n{}'.format(time.asctime(), traceback.format_exc())
        self.get_logger().error(msg_err) 

    def get_bbox(self, x, y, yaw, length, width):
        '''
        arguements:
        x, y : global xy coordinate in [m] (center of gravity)
        yaw : yaw angle in [radians]
        length, width: geometery of car in [m]
        --------------------
        output: ndarray, shape 4x2
        '''
        outline = np.array([[length/2, -width/2],   # rear left
                            [length/2, width/2],    # front left
                            [-length/2, width/2],   # front right
                            [-length/2, -width/2]]) # rear right
        
        R = np.array([[math.cos(yaw), math.sin(yaw)],
                    [-math.sin(yaw), math.cos(yaw)]])
        T = np.array([[x, y]])
        
        transformed = outline @ R
        transformed += T
        return transformed

def main(args=None):
    # Init node
    rclpy.init(args=args)
    simulator_output = SimulatorOutput()
    rclpy.spin(simulator_output)
    
    print("closing sockets...")
    simulator_output.sock_iac.close()
    simulator_output.sock_ego.close()
    simulator_output.sock_opp.close()
    # Delete node
    simulator_output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
