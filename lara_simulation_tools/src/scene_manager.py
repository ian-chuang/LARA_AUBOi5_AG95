#! /usr/bin/env python3

import rospkg
import rospy
import xacro
import gazebo_msgs.srv
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion
# import controller_manager_msgs.srv 
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
import actionlib
import std_srvs.srv
import std_msgs.msg
from threading import Thread
import os

object_db = {
    'plastic_bin' : {
        'mesh': 'plastic_bin.stl',
        'mass': 1.1022530221461508,
        'iox': 2.5908873064430885e-17,
        'ioy': -6.47721826610772e-17,
        'ioz': 0.0466345179467445,
        'ixx': .0084861,
        'iyy': .0163272,
        'izz': .0183834,
        'ixy': 0,
        'iyz': 0,
        'ixz': 0,
    },
    'cube': {
        'mesh': 'cube.stl',
        'mass': .006784,
        'iox': 0,
        'ioy': 0,
        'ioz': 0.02,
        'ixx': .18e-05,
        'iyy': .18e-05,
        'izz': .18e-05,
        'ixy': 0,
        'iyz': 0,
        'ixz': 0,
    },
}

class SceneManager:
    def __init__(self, package_name, object_db):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package_name)
        self.object_db = object_db

        self.reference_frame = 'world'

    def set_color(self, model_name, color):
        req = gazebo_msgs.srv.SetLightPropertiesRequest()
        req.diffuse = color
        req.attenuation_constant = 0.0
        req.attenuation_linear = 0.0
        req.attenuation_quadratic = 0.0

        self.send_request(
            f'/gazebo_color_plugin/{model_name}',
            gazebo_msgs.srv.SetLightProperties,
            req
        )   

    def set_pose(self, model_name, pose):
        req = gazebo_msgs.srv.SetModelStateRequest()
        req.model_state.model_name = model_name
        req.model_state.pose = pose
        req.model_state.reference_frame = self.reference_frame

        self.send_request(
            f'/gazebo/set_model_state',
            gazebo_msgs.srv.SetModelState,
            req
        ) 

    def reset_arm(self):

        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client.wait_for_server()
        goal = FollowJointTrajectoryActionGoal()
        goal.path

        traj = JointTrajectory()
        traj.joint_names = [
            'elbow_joint'
            'shoulder_lift_joint'
            'shoulder_pan_joint'
            'wrist_1_joint'
            'wrist_2_joint'
            'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [0,0,0,0,0,0]
        point.velocities = [0,0,0,0,0,0]
        point.accelerations = [0,0,0,0,0,0]
        point.effort = []
        point.time_from_start = rospy.Duration(0)
        traj.points = [point]

        pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
        pub.publish(traj)
        print('done')


        # req = controller_manager_msgs.srv.SwitchControllerRequest()
        # req.start_controllers = ['arm_position_controller', 'gripper_position_controller']
        # req.stop_controllers = ['arm_controller', 'gripper_controller']
        # req.strictness = req.STRICT

        # self.send_request(
        #     f'/controller_manager/switch_controller',
        #     controller_manager_msgs.srv.SwitchController,
        #     req
        # ) 

        # msg = std_

    
    def spawn_model(self, model_name, id, color, pose):
        mappings = self.object_db[id].copy()
        mappings['model_name'] = model_name
        mappings['color'] = f"{color.r} {color.g} {color.b} {color.a}"
        mappings = {key: str(val) for key, val in mappings.items()}
        xml = xacro.process_file(
            os.path.join(self.package_path, 'urdf/parts/object.xacro'),
            mappings=mappings
        ).toxml()

        req = gazebo_msgs.srv.SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = xml
        req.initial_pose = pose
        req.reference_frame = self.reference_frame

        self.send_request(
            '/gazebo/spawn_urdf_model',
            gazebo_msgs.srv.SpawnModel,
            req
        )        

    def send_request(self, service_name, service_type, req):
        try:
            rospy.wait_for_service(service_name)
            client = rospy.ServiceProxy(service_name, service_type)
            resp = client(req)

        except rospy.ServiceException as e:
            print("Gazebo Service call failed: %s"%e)

    def send_request(self, service_name, service_type, req):
        pub = rospy.Publisher('chatter', String, queue_size=1)
               
        
if __name__ == '__main__':

    objects = [
        {
            'id': 'cube',
            'model_name': 'cube1',
            'color' : ColorRGBA(255, 0, 0, 1),
            'pose': Pose(Point(0.3, 0.2, 0), Quaternion(0,0,0,1)),
        },
        {
            'id': 'cube',
            'model_name': 'cube2',
            'color' : ColorRGBA(0, 255, 0, 1),
            'pose': Pose(Point(0.4, 0.2, 0), Quaternion(0,0,0,1)),
        },
        {
            'id': 'cube',
            'model_name': 'cube3',
            'color' : ColorRGBA(0, 0, 255, 1),
            'pose': Pose(Point(0.5, 0.2, 0), Quaternion(0,0,0,1)),
        },
        {
            'id': 'plastic_bin',
            'model_name': 'plastic_bin1',
            'color' : ColorRGBA(0, 153, 0, 1),
            'pose': Pose(Point(0.4, -0.2, 0), Quaternion(0,0,0,1)),
        }
    ]

    rospy.init_node('scene_manager')
    scene_manager = SceneManager('homestri_robot_description', object_db)

    # for object in objects:
    #     scene_manager.spawn_model(**object)

    # rospy.sleep(3)

    # scene_manager.set_color('plastic_bin1', ColorRGBA(255, 255, 255, 1))
    # scene_manager.set_pose('plastic_bin1', Pose(Point(0.4, -0.4, 0), Quaternion(0,0,0,1)))

    scene_manager.reset_arm()


    rospy.spin()

    




        
