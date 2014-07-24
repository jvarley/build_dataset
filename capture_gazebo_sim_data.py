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

GAZEBO_MODEL_PATH = os.environ["GAZEBO_MODEL_PATH"]
GRASPABLE_OBJECT_PATH = rospack.get_path('graspable_objects')


class GazeboModelManager():

    def __init__(self,
                 gazebo_namespace="/gazebo",
                 model_path=GAZEBO_MODEL_PATH):

        self.gazebo_namespace = gazebo_namespace
        self.model_path = model_path
        self.delete_model_service = rospy.ServiceProxy(gazebo_namespace + '/delete_model', DeleteModel)
        self.get_model_state_service = rospy.ServiceProxy(gazebo_namespace + '/get_model_state', GetModelState)

    def remove_model(self, model_name="coke_can"):

        del_model_req = DeleteModelRequest(model_name)
        self.delete_model_service(del_model_req)

    def spawn_model(self, model_name="coke_can"):

        model_xml = open(self.model_path + "/" + model_name + "/model.sdf").read()
        model_pose = Pose()
        model_pose.position.x = 2.5
        model_pose.position.z = 1
        robot_namespace = model_name
        gazebo_interface.spawn_sdf_model_client(model_name=model_name,
                                                model_xml=model_xml,
                                                robot_namespace=robot_namespace,
                                                initial_pose=model_pose,
                                                reference_frame="world",
                                                gazebo_namespace=self.gazebo_namespace)

    def spawn_kinect(self):
        model_name = "camera1"
        model_xml = rospy.get_param("robot_description")
        model_pose = Pose()
        robot_namespace = model_name
        gazebo_interface.spawn_urdf_model_client(model_name=model_name,
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


if __name__ == '__main__':

    output_image_path = "rgbd_images/"
    model_path = GRASPABLE_OBJECT_PATH

    rgbd_listener = RGBDListener()
    rgbd_listener.listen()

    model_manager = GazeboModelManager(model_path=model_path)

    model_manager.spawn_kinect()

    for model_name in os.listdir(model_path):
        if not os.path.isdir(model_path + model_name):
            continue

        print model_name

        model_manager.spawn_model(model_name)

        #large models can take a moment to load
        while not model_manager.does_world_contain_model(model_name):
            sleep(0.5)

        #just to give the rgbd_listener time to receive msgs
        sleep(2)

        rgbd_image = np.copy(rgbd_listener.rgbd_image)

        plt.imsave(output_image_path + model_name + '.png', rgbd_image)

        model_manager.remove_model(model_name)
