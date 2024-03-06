from ctypes import *
import datetime
import xvsdk
import numpy as np
import cv2
import keyboard
import time

count = 1
class ColorCameraResolution():
        RGB_1920x1080=0
        RGB_1280x720=1
        RGB_640x480=2
        RGB_320x240=3
        RGB_2560x1920=4
        RGB_3840x2160=5
fe_auto_Exposure=c_bool()
fe_gain=c_int()
fe_exposureTimeMs=c_int()
fe_auto_Exposure.value=False

xvsdk.init()
xvsdk.slam_start()
xvsdk.stereo_start()
xvsdk.imu_start()
xvsdk.rgb_start()
xvsdk.fe_april_tag_start(b"36h11", c_double(0.0639), c_double(50.))
xvsdk.rgb_april_tag_start(b"36h11", c_double(0.0639), c_double(50.))
xvsdk.sgbm_start()
'''
sn = xvsdk.xv_get_sn()
print("device sn = ", sn.value.decode('utf8'), "\n")

fe_trans_size, fe_ucm_size, fe_transform, fe_ucm = xvsdk.xv_get_fisheye_intrinsics()
for transform in fe_transform:
    print("fe R:[", transform.rotation[0], ",", transform.rotation[1], ",", transform.rotation[2], ",", transform.rotation[3], ",", transform.rotation[4], ",", transform.rotation[5], ",", transform.rotation[6], ",", transform.rotation[7], ",", transform.rotation[8], "]\n")
    print("fe T:[", transform.translation[0], ",", transform.translation[1], ",", transform.translation[2], "]\n")
for ucm in fe_ucm:
    print("fe UCM: { w=", ucm.w, ", h=", ucm.h, ", fx=", ucm.fx, ", fy=", ucm.fy, ", u0=", ucm.u0, ", v0=", ucm.v0, ", xi=", ucm.xi, "}\n")

rgb_trans_size, rgb_pdcm_size, rgb_transform, rgb_pdcm = xvsdk.xv_get_rgb_intrinsics()
for transform in rgb_transform:
    print("rgb R:[", transform.rotation[0], ",", transform.rotation[1], ",", transform.rotation[2], ",", transform.rotation[3], ",", transform.rotation[4], ",", transform.rotation[5], ",", transform.rotation[6], ",", transform.rotation[7], ",", transform.rotation[8], "]\n")
    print("rgb T:[", transform.translation[0], ",", transform.translation[1], ",", transform.translation[2], "]\n")
for pdcm in rgb_pdcm:
    print("rgb pdcm: { w=", pdcm.w, ", h=", pdcm.h, ", fx=", pdcm.fx, ", fy=", pdcm.fy, ", u0=", pdcm.u0, ", v0=", pdcm.v0, ", distor[0]=", pdcm.distor[0], ", distor[1]=", pdcm.distor[1], ", distor[2]=", pdcm.distor[2], ", distor[3]=", pdcm.distor[3], ", distor[4]=", pdcm.distor[4], "}\n")
'''
start = datetime.datetime.now().microsecond
print(start)
end = 0
index=0

xvsdk.xv_set_rgb_camera_resolution(ColorCameraResolution.RGB_1280x720)
xvsdk.xv_set_fe_autoExposure(fe_auto_Exposure)
# fe_gain=1
# fe_exposureTimeMs=1
fe_gain=15
fe_exposureTimeMs=65
# xvsdk.xv_set_fe_gain(fe_gain)
# xvsdk.xv_set_fe_exposureTimeMs(fe_exposureTimeMs)

dump_fe_files = False

xvsdk.xv_set_fe_gain_and_exposureTimeMs(fe_gain, fe_exposureTimeMs)
while count != 0:
    #if (end - start)%500 == 0:
  
        position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence = xvsdk.xv_get_6dof()
        print("6dof: position x =", position.x , ", y = ", position.y, ", z = ", position.z, ", pitch =", orientation.x * 180 / 3.14 , ", yaw = ", orientation.y * 180 / 3.14, ", roll = ", orientation.z * 180 / 3.14, ", quaternion[0] = ", quaternion.q0, ", quaternion[1] = ", quaternion.q1, ", quaternion[2] = ", quaternion.q2, ", quaternion[3] = ", quaternion.q3, ", edgeTimestamp = ", slam_edgeTimestamp.value, ", hostTimestamp = ", slam_hostTimestamp.value, ", confidence = ", slam_confidence.value, "\n")
        
        position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence = xvsdk.xv_get_6dof_prediction(c_double(0.005))
        print("6dof with prediction: position x =", position.x , ", y = ", position.y, ", z = ", position.z, ", pitch =", orientation.x * 180 / 3.14 , ", yaw = ", orientation.y * 180 / 3.14, ", roll = ", orientation.z * 180 / 3.14, ", quaternion[0] = ", quaternion.q0, ", quaternion[1] = ", quaternion.q1, ", quaternion[2] = ", quaternion.q2, ", quaternion[3] = ", quaternion.q3, ", edgeTimestamp = ", slam_edgeTimestamp.value, ", hostTimestamp = ", slam_hostTimestamp.value, ", confidence = ", slam_confidence.value, "\n")
        
        accel, gyro, imu_edgeTimestamp, imu_hostTimestamp = xvsdk.xslam_get_imu()
        print("imu: accel x =", accel.x , ", y = ", accel.y, ", z = ", accel.z, ", gyro x =", gyro.x , ", y = ", gyro.y, ", z = ", gyro.z, ", edgeTimestamp = ", imu_edgeTimestamp.value, ", hostTimestamp = ", imu_hostTimestamp.value, "\n")

        fe_width, fe_height, stereo_edgeTimestamp, stereo_hostTimestamp, fe_left_data, fe_right_data, fe_dataSize = xvsdk.xv_get_stereo()
        print("fisheye: width = ", fe_width.value, ", height = ", fe_height.value, ", edgeTimestamp = ", stereo_edgeTimestamp.value, ", hostTimestamp = ", stereo_hostTimestamp.value, ", datasize = ", fe_dataSize.value, "\n")
        if fe_dataSize.value > 0 :

            left_image = np.asfortranarray(fe_left_data)
            left_image = left_image[:fe_width.value*fe_height.value]
            left_image = left_image.reshape(fe_height.value, fe_width.value)
            left_image = cv2.cvtColor(left_image, cv2.COLOR_GRAY2BGR) 
            cv2.imshow('left', left_image)
            cv2.waitKey(1)
            right_image = np.asfortranarray(fe_right_data)
            right_image = right_image[:fe_width.value*fe_height.value]
            right_image = right_image.reshape(fe_height.value, fe_width.value)
            right_image = cv2.cvtColor(right_image, cv2.COLOR_GRAY2BGR) 
            cv2.imshow('right', right_image)
            cv2.waitKey(1)

            if dump_fe_files:
                dt_ms = datetime.datetime.now().strftime('%H%M%S-%f')
                dump_file_left = open(dt_ms + '-left.raw','wb')
                dump_file_left.write(bytearray(fe_left_data))
                dump_file_right = open(dt_ms + '-right.raw','wb')
                dump_file_right.write(bytearray(fe_right_data))

        rgb_width, rgb_height, rgb_edgeTimestamp, rgb_hostTimestamp, rgb_codec, rgb_data, rgb_dataSize = xvsdk.xv_get_rgb()
        print("rgb: width = ", rgb_width.value, ", height = ", rgb_height.value, ", edgeTimestamp = ", rgb_edgeTimestamp.value, ", hostTimestamp = ", rgb_hostTimestamp.value, ", codec = ", rgb_codec.value, ", datasize = ", rgb_dataSize.value, "\n")
        
        if rgb_dataSize.value > 0 :
            color_image = np.asfortranarray(rgb_data)
            nheight = int(rgb_height.value*3/2)
            color_image = color_image[:rgb_width.value*nheight]
            color_image = color_image.reshape(nheight, rgb_width.value)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_YUV2BGR_IYUV) 
            cv2.imshow('rgb', color_image)
            cv2.waitKey(1)
        
        position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence = xvsdk.xv_get_6dof_at_timestamp(c_double(rgb_hostTimestamp.value))
        print("6dof with with rgb timestamp x =", position.x , ", y = ", position.y, ", z = ", position.z, ", pitch =", orientation.x * 180 / 3.14 , ", yaw = ", orientation.y * 180 / 3.14, ", roll = ", orientation.z * 180 / 3.14, ", quaternion[0] = ", quaternion.q0, ", quaternion[1] = ", quaternion.q1, ", quaternion[2] = ", quaternion.q2, ", quaternion[3] = ", quaternion.q3, ", edgeTimestamp = ", slam_edgeTimestamp.value, ", hostTimestamp = ", slam_hostTimestamp.value, ", confidence = ", slam_confidence.value, "\n")
        
        position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence = xvsdk.xv_get_6dof_at_timestamp(c_double(stereo_hostTimestamp.value))
        print("6dof with with fisheye timestamp x =", position.x , ", y = ", position.y, ", z = ", position.z, ", pitch =", orientation.x * 180 / 3.14 , ", yaw = ", orientation.y * 180 / 3.14, ", roll = ", orientation.z * 180 / 3.14, ", quaternion[0] = ", quaternion.q0, ", quaternion[1] = ", quaternion.q1, ", quaternion[2] = ", quaternion.q2, ", quaternion[3] = ", quaternion.q3, ", edgeTimestamp = ", slam_edgeTimestamp.value, ", hostTimestamp = ", slam_hostTimestamp.value, ", confidence = ", slam_confidence.value, "\n")
        
        tags = xvsdk.xv_get_fe_april_tags()
        for tag in tags:
            print("Tag: tagid = ", tag.tagID, ", position x =", tag.position[0] , ", y = ", tag.position[1], ", z = ", tag.position[2], ", pitch =", tag.orientation[0] * 180 / 3.14 , ", yaw = ", tag.orientation[1] * 180 / 3.14, ", roll = ", tag.orientation[2] * 180 / 3.14, ", quaternion[0] = ", tag.quaternion[0], ", quaternion[1] = ", tag.quaternion[1], ", quaternion[2] = ", tag.quaternion[2], ", quaternion[3] = ", tag.quaternion[3], ", edgeTimestamp = ", tag.edgeTimestamp, ", hostTimestamp = ", tag.hostTimestamp, ", confidence = ", tag.confidence, "\n")
        print("start xv_get_fe_april_tags_withslam")
        tagSlamPoses = xvsdk.xv_get_fe_april_tags_withslam(b"36h11", c_double(0.0639))
        for tagPos in tagSlamPoses:
            print("TagPose with slam: tagid = ", tagPos.tagID, ", position x =", tagPos.position[0] , ", y = ", tagPos.position[1], ", z = ", tagPos.position[2], ", pitch =", tagPos.orientation[0] * 180 / 3.14 , ", yaw = ", tagPos.orientation[1] * 180 / 3.14, ", roll = ", tagPos.orientation[2] * 180 / 3.14, ", quaternion[0] = ", tagPos.quaternion[0], ", quaternion[1] = ", tagPos.quaternion[1], ", quaternion[2] = ", tagPos.quaternion[2], ", quaternion[3] = ", tagPos.quaternion[3], ", edgeTimestamp = ", tagPos.edgeTimestamp, ", hostTimestamp = ", tagPos.hostTimestamp, ", confidence = ", tagPos.confidence, "\n")
        print("start xv_get_fe_april_tags_withFE")
        tagFEPoses, tagFETrans = xvsdk.xv_get_fe_april_tags_withFE(b"36h11", c_double(0.0639))
        for tagPos in tagFEPoses:
            print("TagPose with FE: tagid = ", tagPos.tagID, ", position x =", tagPos.position[0] , ", y = ", tagPos.position[1], ", z = ", tagPos.position[2], ", pitch =", tagPos.orientation[0] * 180 / 3.14 , ", yaw = ", tagPos.orientation[1] * 180 / 3.14, ", roll = ", tagPos.orientation[2] * 180 / 3.14, ", quaternion[0] = ", tagPos.quaternion[0], ", quaternion[1] = ", tagPos.quaternion[1], ", quaternion[2] = ", tagPos.quaternion[2], ", quaternion[3] = ", tagPos.quaternion[3], ", edgeTimestamp = ", tagPos.edgeTimestamp, ", hostTimestamp = ", tagPos.hostTimestamp, ", confidence = ", tagPos.confidence, "\n")
        for tagTrans in tagFETrans:
            print("TagPose by transformed: tagid = ", tagTrans.tagID, ", position x =", tagTrans.position[0] , ", y = ", tagTrans.position[1], ", z = ", tagTrans.position[2], ", pitch =", tagTrans.orientation[0] * 180 / 3.14 , ", yaw = ", tagTrans.orientation[1] * 180 / 3.14, ", roll = ", tagTrans.orientation[2] * 180 / 3.14, ", quaternion[0] = ", tagTrans.quaternion[0], ", quaternion[1] = ", tagTrans.quaternion[1], ", quaternion[2] = ", tagTrans.quaternion[2], ", quaternion[3] = ", tagTrans.quaternion[3],  "\n")
        
        sgbm_width, sgbm_height, sgbm_edgeTimestamp, sgbm_hostTimestamp, sgbm_data, sgbm_dataSize = xvsdk.xv_get_sgbm()
        print("sgbm: width = ", sgbm_width.value, ", height = ", sgbm_height.value, ", edgeTimestamp = ", sgbm_edgeTimestamp.value, ", hostTimestamp = ", sgbm_hostTimestamp.value, ", datasize = ", sgbm_dataSize.value, "\n")
        
        if keyboard.is_pressed('d'):
            dump_fe_files = True
        if keyboard.is_pressed('s'):
            dump_fe_files = False

        rgbtags = xvsdk.xv_get_rgb_april_tags()
        for rgbtag in rgbtags:
            print("rgb Tag: tagid = ", rgbtag.tagID, ", position x =", rgbtag.position[0] , ", y = ", rgbtag.position[1], ", z = ", rgbtag.position[2], ", pitch =", rgbtag.orientation[0] * 180 / 3.14 , ", yaw = ", rgbtag.orientation[1] * 180 / 3.14, ", roll = ", rgbtag.orientation[2] * 180 / 3.14, ", quaternion[0] = ", rgbtag.quaternion[0], ", quaternion[1] = ", rgbtag.quaternion[1], ", quaternion[2] = ", rgbtag.quaternion[2], ", quaternion[3] = ", rgbtag.quaternion[3], ", edgeTimestamp = ", rgbtag.edgeTimestamp, ", hostTimestamp = ", rgbtag.hostTimestamp, ", confidence = ", rgbtag.confidence, "\n")
        
    
    #end = datetime.datetime.now().microsecond
