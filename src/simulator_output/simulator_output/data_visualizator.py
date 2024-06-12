import rclpy
import numpy as np
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point, PoseArray, Quaternion, Pose

from simple_msgs.msg import TrajectoryData, EgoRecording, OpponentRecording
from simulator_output.cubic_spline_planner import *

#path_created = False
#trj_received = False
#pub_ego = []
#opponent_topics = dict()

class TrajLists:
    x_ref = []
    y_ref = []
    speed_ref = []
    x_left = []
    y_left = []
    x_right = []
    y_right = []

trj_lists = TrajLists()

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def build_path_msg(xs, ys):
    p_msg = Path()
    p_msg.header.frame_id = "map"
    
    for i in range(len(xs)):
        p = PoseStamped()
        p.pose.position.x = xs[i] 
        p.pose.position.y = ys[i]
        p_msg.poses.append(p)
    return p_msg

def trajectory_callback(data):
    global trj_lists
    global trj_received

    print("trajectory call back started")

    trj_lists.x_ref = [poses.position.x for poses in data.fast_lane.poses]
    trj_lists.y_ref = [poses.position.y for poses in data.fast_lane.poses]

    trj_lists.speed_ref = data.speed_arr

    trj_lists.x_left = [poses.position.x for poses in data.left_lane.poses]
    trj_lists.y_left = [poses.position.y for poses in data.left_lane.poses]

    trj_lists.x_right = [poses.position.x for poses in data.right_lane.poses]
    trj_lists.y_right = [poses.position.y for poses in data.right_lane.poses]

    print("trajectory call back finished")

    trj_received = True

def opponent_callback(data):
    global opponent_topics

    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose = data.world_position
    

    opponent_topics.get(data.id).publish(p)

def ego_callback(data):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position = data.world_position.position
    p.pose.orientation = Quaternion(*euler_to_quaternion(data.world_position.orientation.x,
                                data.world_position.orientation.y,
                                data.world_position.orientation.z))
    data.world_position

    pub_ego.publish(p) #TODO

def get_id_from_name(name):
    return int(name.split("_")[0][4:])

if __name__ ==  "__main__":
    global path_created 
    path_created = False

    global opponent_topics, pub_ego
    pub_ego = []
    opponent_topics = dict()
    
    # ROS node initialization
    rclpy.init()
    node = rclpy.create_node('simulation_data')

    #Publishers
    pub_path = node.create_publisher(Path, 'path')
    pub_left = node.create_publisher(Path, 'left_margin')
    pub_right = node.create_publisher(Path, 'right_margin')
    pub_ego = node.create_publisher(PoseStamped,  'ego_data_vis')
    #TODO capire se va bene iterare ogni nodo
    for nd in node.get_node_names:
        for el in node.get_published_topics():
            if el[0].startswith("/opp"):
                r = node.create_publisher( PoseStamped, str(el[0] + '_vis'))
                opponent_topics[get_id_from_name(el[0])] = r

    #Subscribers
    node.create_subscription(EgoRecording, "/ego_data", ego_callback)
    node.create_subscription(TrajectoryData, "trajectorydata", trajectory_callback)
    for nd in node.get_node_names:
        for el in node.get_published_topics():
            if el[0].startswith("/opp") and not el[0].endswith("_vis"):
                node.create_subscription(el[0], OpponentRecording, opponent_callback)

    

    while rclpy.ok():
        if(trj_received == False):
            print("Trajectory data not received yet")
            continue
        elif(path_created == False):
            # spline to follow
            path = Spline2D(trj_lists.x_ref, trj_lists.y_ref)
            p_msg = build_path_msg([ path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                                    [ path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

            left_path = Spline2D(trj_lists.x_left, trj_lists.y_left)
            left_p_msg = build_path_msg([ left_path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                                    [ left_path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

            right_path = Spline2D(trj_lists.x_right, trj_lists.y_right)
            right_p_msg = build_path_msg([ right_path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                                    [ right_path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

            path_created = True

        pub_path.publish(p_msg)
        pub_left.publish(left_p_msg)
        pub_right.publish(right_p_msg)

    #     for el in opponent_topics:
    #         el.publish

