#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
isaretler_file = "/home/abdullatif/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/"
def setParkSlots():
        NUMBER_OF_PARK_SLOT=23
        park_yeri_model = open(isaretler_file+'park_yeri/model.sdf', 'r')
        park_yeri_model = park_yeri_model.read()
        park_etmek_yasaktir_model = open(isaretler_file+'park_etmek_yasaktir/model.sdf', 'r')
        park_etmek_yasaktir_model = park_etmek_yasaktir_model.read()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = [rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel) for i in range(NUMBER_OF_PARK_SLOT)]
        model_pose = Pose()
        '''
        locations_x=[i for i in range(0,5)]
        locations_y=[i for i in range(0,5)]
        a=np.random.randint(1,5)
        limit_sign_pose.position.x = locations_x[a]
        limit_sign_pose.position.y = locations_y[a]
        '''
        model_pose.position.x=-76.5     #Manually taken
        model_pose.position.y=-62.61    #Manually taken
        model_pose.position.z=1.20      #Manually taken
        model_pose.orientation.x = 0
        model_pose.orientation.y = 0
        model_pose.orientation.z = 0
        model_pose.orientation.w = 0
        for i in range(NUMBER_OF_PARK_SLOT):
            a=np.random.randint(0,3)
            if a==1:
                spawn_model_prox[i]('park_yeri_'+str(i), park_yeri_model, "oto_name_space", model_pose, "world")
            else:
                spawn_model_prox[i]('park_etmek_yasaktir_'+str(i), park_etmek_yasaktir_model, "oto_name_space", model_pose, "world")

            model_pose.position.y += 2.4
        """
        parking_pose.position.z = 0.03
        parking_pose.orientation.x = 0
        parking_pose.orientation.y = 0
        parking_pose.orientation.z = -1
        parking_pose.orientation.w = 1
        """
def setOthers():
    NUMBER_OF_SIGN=6
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = [rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel) for i in range(NUMBER_OF_SIGN)]
    model_pose = Pose()
    model_pose.position.z=1      #Manually taken

    i=0

    model_pose.position.x=-38.5     #Manually taken
    model_pose.position.y=-65.5    #Manually taken
    sol_yon_model = open(isaretler_file+'sol_yon/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), sol_yon_model, "oto_name_space", model_pose, "world")
    i+=1

    model_pose.position.x=-29    #Manually taken
    model_pose.position.y=-35    #Manually taken
    azami_20_model = open(isaretler_file+'azami_20/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), azami_20_model, "oto_name_space", model_pose, "world")
    i+=1

    model_pose.position.x=-29     #Manually taken
    model_pose.position.y=21    #Manually taken
    hiz_son_20_model = open(isaretler_file+'hiz_son_20/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), hiz_son_20_model, "oto_name_space", model_pose, "world")
    i+=1

    model_pose.position.x=-3     #Manually taken
    model_pose.position.y=60    #Manually taken
    isaret_durak_model = open(isaretler_file+'durak/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), isaret_durak_model, "oto_name_space", model_pose, "world")
    i+=1

    model_pose.position.x=33.5     #Manually taken
    model_pose.position.y=60    #Manually taken
    isaret_giris_olamayan_yol_model = open(isaretler_file+'giris_olmayan_yol/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), isaret_giris_olamayan_yol_model, "oto_name_space", model_pose, "world")
    i+=1

    model_pose.position.x=20.3     #Manually taken
    model_pose.position.y=1.5    #Manually taken
    sag_yon_model = open(isaretler_file+'sag_yon/model.sdf', 'r').read()
    spawn_model_prox[i]('isaret'+str(i), sag_yon_model, "oto_name_space", model_pose, "world")
    i+=1

if __name__=="__main__":
    rospy.init_node('assign_signs')

    setParkSlots()
    setOthers()

