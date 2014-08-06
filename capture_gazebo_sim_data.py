#!/usr/bin/env python
import rospy
import rospkg
rospack = rospkg.RosPack()
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, DeleteModelResponse, GetModelState, GetModelStateRequest

import numpy as np
import os
from time import sleep
import matplotlib.pyplot as plt

import tf_conversions
import xyz_to_pixel_loc
import PyKDL
import math
import std_srvs.srv

import tf

GAZEBO_MODEL_PATH = os.environ["GAZEBO_MODEL_PATH"]
GRASPABLE_MODEL_PATH = rospack.get_path('graspable_objects')


class RGBDListener():

    def __init__(self,
                 depth_topic="/camera1/camera/depth/image_raw",
                 rgb_topic="/camera1/camera/rgb/image_raw"):

        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.rgbd_image = np.zeros((480, 640, 4))

    def depth_image_callback(self, data):
        depth_image_np = self.image2numpy(data)
        self.rgbd_image[:, :, 3] = depth_image_np

    def rgb_image_callback(self, data):
        rgbd_image_np = self.image2numpy(data)
        self.rgbd_image[:, :, 0:3] = rgbd_image_np

    #this method from:
    #https://github.com/rll/sushichallenge/blob/master/python/brett2/ros_utils.py
    def image2numpy(self, image):
        if image.encoding == 'rgb8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)[:, :, ::-1]
        if image.encoding == 'bgr8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        elif image.encoding == 'mono8':
            return np.fromstring(image.data, dtype=np.uint8).reshape(image.height, image.width)
        elif image.encoding == '32FC1':
            return np.fromstring(image.data, dtype=np.float32).reshape(image.height, image.width)
        else:
            raise Exception

    def listen(self):

        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber(self.depth_topic, Image, self.depth_image_callback, queue_size=1)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_image_callback, queue_size=1)


class GazeboKinectManager():
    def __init__(self, gazebo_namespace="/gazebo"):
        self.gazebo_namespace = gazebo_namespace
        self.rgbd_listener = RGBDListener()
        self.rgbd_listener.listen()
        self.camera_name = "camera1"

    def spawn_kinect(self):
        model_xml = rospy.get_param("robot_description")

        #f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi, math.pi), PyKDL.Vector(0, 0, 2))
        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi+math.pi/4.0, math.pi), PyKDL.Vector(0, 0, 2))
        f = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi/4.0, 0), PyKDL.Vector(0, 0, 2))
        model_pose = tf_conversions.posemath.toMsg(f)
        robot_namespace = self.camera_name
        gazebo_interface.spawn_urdf_model_client(model_name=self.camera_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

    def get_rgbd_image(self):
        return self.rgbd_listener.rgbd_image


class GazeboModelManager():

    def __init__(self,
                 gazebo_namespace="/gazebo",
                 models_dir=GAZEBO_MODEL_PATH):

        self.gazebo_namespace = gazebo_namespace
        self.models_dir = models_dir
        self.delete_model_service = rospy.ServiceProxy(gazebo_namespace + '/delete_model', DeleteModel)
        self.get_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)

    def remove_model(self, model_name="coke_can"):

        del_model_req = DeleteModelRequest(model_name)
        self.delete_model_service(del_model_req)

    def spawn_model(self, model_name="coke_can"):

        model_xml = open(self.models_dir + "/" + model_name + "/model.sdf").read()
        model_pose = Pose()
        model_pose.position.x = 2
        model_pose.position.y = 0.5
        model_pose.position.z = 1
        robot_namespace = model_name
        gazebo_interface.spawn_sdf_model_client(model_name=model_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

    def does_world_contain_model(self, model_name="coke_can"):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        resp = self.get_model_state_service(get_model_state_req)
        return resp.success

    def get_model_state(self,model_name="coke_can"):
        get_model_state_req = GetModelStateRequest()
        get_model_state_req.model_name = model_name
        return self.get_model_state_service(get_model_state_req)


def add_transform(pose_in_world_frame, frame_id, child_frame_id, transformer):
    transform_msg = tf.msg.geometry_msgs.msg.TransformStamped()
    transform_msg.transform.translation.x = pose_in_world_frame.position.x
    transform_msg.transform.translation.y = pose_in_world_frame.position.y
    transform_msg.transform.translation.z = pose_in_world_frame.position.z

    transform_msg.transform.rotation.x = pose_in_world_frame.orientation.x
    transform_msg.transform.rotation.y = pose_in_world_frame.orientation.y
    transform_msg.transform.rotation.z = pose_in_world_frame.orientation.z
    transform_msg.transform.rotation.w = pose_in_world_frame.orientation.w
    transform_msg.child_frame_id = child_frame_id
    transform_msg.header.frame_id = frame_id

    transformer.setTransform(transform_msg)


def transform_pose(pose, old_frame, new_frame, transformer):

    transform_msg = tf.msg.geometry_msgs.msg.PoseStamped()
    transform_msg.pose.position.x = pose.position.x
    transform_msg.pose.position.y = pose.position.y
    transform_msg.pose.position.z = pose.position.z

    transform_msg.pose.orientation.x = pose.orientation.x
    transform_msg.pose.orientation.y = pose.orientation.y
    transform_msg.pose.orientation.z = pose.orientation.z
    transform_msg.pose.orientation.w = pose.orientation.w

    transform_msg.header.frame_id = old_frame

    return transformer.transformPose(new_frame, transform_msg)


class Grasp():
    def __init__(self, pose=None, score=100):
        if not pose:
            f = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, 0, 0))
            pose = tf_conversions.posemath.toMsg(f)

        self.pose = pose
        self.score = score


def get_model_grasps(model_name):

    return [Grasp()]


if __name__ == '__main__':

    output_image_dir = "rgbd_images/"
    models_dir = GRASPABLE_MODEL_PATH

    kinect_manager = GazeboKinectManager()
    kinect_manager.spawn_kinect()

    pause_physics_service_proxy = rospy.ServiceProxy("/gazebo/pause_physics", std_srvs.srv.Empty)
    unpause_physics_service_proxy = rospy.ServiceProxy("/gazebo/unpause_physics", std_srvs.srv.Empty)
    #we don't need any physics right now
    pause_physics_service_proxy()

    model_manager = GazeboModelManager(models_dir=models_dir)

    for model_name in os.listdir(models_dir):

        if not os.path.isdir(models_dir + "/" + model_name):
            continue

        print model_name

        model_manager.spawn_model(model_name)

        #large models can take a moment to load
        while not model_manager.does_world_contain_model(model_name):
            sleep(0.5)

        #just to give the rgbd_listener time to receive msgs
        sleep(2)

        rgbd_image = np.copy(kinect_manager.get_rgbd_image())

        grasp_points = np.zeros((480, 640))

        transformer = tf.TransformerROS(True, rospy.Duration(10.0))

        camera_pose_in_world_frame = model_manager.get_model_state(kinect_manager.camera_name).pose
        add_transform(camera_pose_in_world_frame, "World", "Camera", transformer)

        model_pose_in_world_frame = model_manager.get_model_state(model_name).pose
        add_transform(model_pose_in_world_frame, "World", "Model", transformer)

        model_grasps = get_model_grasps(model_name)

        for model_grasp in model_grasps:

            add_transform(model_grasp.pose, "Model", "Grasp", transformer)

            #get grasp point in camera frame
            grasp_in_camera_frame = transform_pose(model_grasp.pose, "Grasp", "Camera", transformer)
            grasp_in_camera_frame = grasp_in_camera_frame.pose

            #this is the pixel location of the grasp point
            u, v = xyz_to_pixel_loc.xyz_to_uv((grasp_in_camera_frame.position.x,grasp_in_camera_frame.position.y,grasp_in_camera_frame.position.z))

            #rgbd_image[u-5:u+5, v-5:v+5, :] = 0
            grasp_points[u, v] = model_grasp.score

        plt.imsave(output_image_dir + model_name + '_out.png', grasp_points)
        plt.imsave(output_image_dir + model_name + '.png', rgbd_image)

        model_manager.remove_model(model_name)
