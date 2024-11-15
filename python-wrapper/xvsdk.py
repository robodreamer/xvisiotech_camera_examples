
from ctypes import *
dll = CDLL("./libxvisio-CInterface-wrapper.so")


class Vector3F(Structure):
        _fields_ = [('x', c_float),
                    ('y', c_float),
                    ('z', c_float)]

class Quaternion(Structure):
        _fields_ = [('q0', c_float),
                    ('q1', c_float),
                    ('q2', c_float),
                    ('q3', c_float)]

class Vector3D(Structure):
        _fields_ = [('x', c_double),
                    ('y', c_double),
                    ('z', c_double)]

class Transform_Matrix(Structure):
        _fields_ = [('rotation', c_double * 9),
                    ('translation', c_double * 3)]

class UnifiedCameraModel(Structure):
        _fields_ = [('w', c_int),
                    ('h', c_int),
                    ('fx', c_double),
                    ('fy', c_double),
                    ('u0', c_double),
                    ('v0', c_double),
                    ('xi', c_double)]

class PolynomialDistortionCameraModel(Structure):
        _fields_ = [('w', c_int),
                    ('h', c_int),
                    ('fx', c_double),
                    ('fy', c_double),
                    ('u0', c_double),
                    ('v0', c_double),
                    ('distor', c_double * 5)]

class TagData(Structure):
        _fields_ = [('tagID', c_int),
                    ('position', c_float * 3),
                    ('orientation', c_float * 3),
                    ('quaternion', c_float * 4),
                    ('edgeTimestamp', c_longlong),
                    ('hostTimestamp', c_double),
                    ('confidence', c_double)]

class TagTransData(Structure):
        _fields_ = [('tagID', c_int),
                    ('position', c_float * 3),
                    ('orientation', c_float * 3),
                    ('quaternion', c_float * 4)]


#slam
position = Vector3F()
orientation = Vector3F()
quaternion = Quaternion()
slam_edgeTimestamp = c_longlong()
slam_hostTimestamp = c_double()
slam_confidence = c_double()

#fisheye
fe_width = c_int()
fe_height = c_int()
fe_dataSize = c_int()
stereo_edgeTimestamp = c_longlong()
stereo_hostTimestamp = c_double()

#imu
accel = Vector3F()
gyro = Vector3F()
imu_edgeTimestamp = c_longlong()
imu_hostTimestamp = c_double()

#rgb
rgb_width = c_int()
rgb_height = c_int()
rgb_dataSize = c_int()
rgb_edgeTimestamp = c_longlong()
rgb_hostTimestamp = c_double()
rgb_codec = c_int()
rgb_transform = Transform_Matrix()

#tof
tof_width = c_int()
tof_height = c_int()
tof_dataSize = c_int()
tof_type = c_int()
tof_edgeTimestamp = c_longlong()
tof_hostTimestamp = c_double()
tof_transform = Transform_Matrix()

#sgbm
sgbm_width = c_int()
sgbm_height = c_int()
sgbm_dataSize = c_int()
sgbm_edgeTimestamp = c_longlong()
sgbm_hostTimestamp = c_double()

def init():
    return dll.xv_device_init()

def stop():
    dll.xv_device_uninit()

def slam_start():
    return dll.xv_start_slam()

def slam_stop():
    return dll.xv_stop_slam()

def slam_reset():
    return dll.xv_reset_slam()

def stereo_start():
    return dll.xv_start_stereo()

def imu_start():
    return dll.xv_start_imu()

def rgb_start():
    return dll.xv_start_rgb()

def tof_start():
    return dll.xv_start_tof()

def sgbm_start():
    return dll.xv_start_sgbm()

def fe_april_tag_start(tagFamily, size, refreshRate):
    return dll.xv_start_fe_tag_detector(tagFamily, size, refreshRate)

def rgb_april_tag_start(tagFamily, size, refreshRate):
    return dll.xv_get_rgb_tag_size(tagFamily, size, refreshRate)

def xv_get_6dof():
    dll.xv_get_6dof(byref(position), byref(orientation), byref(quaternion), byref(slam_edgeTimestamp),  byref(slam_hostTimestamp), byref(slam_confidence))
    return position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence

def xv_get_6dof_prediction(prediction):
    dll.xv_get_6dof_prediction(byref(position), byref(orientation), byref(quaternion), byref(slam_edgeTimestamp),  byref(slam_hostTimestamp), byref(slam_confidence), prediction)
    return position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence

def xv_get_6dof_at_timestamp(timestamp):
    dll.xv_get_6dof_at_timestamp(byref(position), byref(orientation), byref(quaternion), byref(slam_edgeTimestamp),  byref(slam_hostTimestamp), byref(slam_confidence), timestamp)
    return position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence

def xslam_get_imu():
    dll.xv_get_imu(byref(accel), byref(gyro), byref(imu_edgeTimestamp),  byref(imu_hostTimestamp))
    return accel, gyro, imu_edgeTimestamp, imu_hostTimestamp

def xv_get_stereo():
    dll.xv_get_stereo_info(byref(fe_width), byref(fe_height), byref(stereo_edgeTimestamp),  byref(stereo_hostTimestamp), byref(fe_dataSize))
    fe_left_data = (c_ubyte * fe_dataSize.value)()
    fe_right_data = (c_ubyte * fe_dataSize.value)()
    dll.xv_get_stereo_image(byref(fe_left_data), byref(fe_right_data))
    return fe_width, fe_height, stereo_edgeTimestamp, stereo_hostTimestamp, fe_left_data, fe_right_data, fe_dataSize

def xv_get_rgb():
    dll.xv_get_rgb_info(byref(rgb_width), byref(rgb_height),byref(rgb_edgeTimestamp),  byref(rgb_hostTimestamp), byref(rgb_dataSize), byref(rgb_codec))
    rgb_data = (c_ubyte * rgb_dataSize.value)()
    dll.xv_get_rgb_image(byref(rgb_data))
    return rgb_width, rgb_height, rgb_edgeTimestamp, rgb_hostTimestamp, rgb_codec, rgb_data, rgb_dataSize

def xv_get_tof():
    dll.xv_get_tof_info(byref(tof_width), byref(tof_height),byref(tof_edgeTimestamp),  byref(tof_hostTimestamp), byref(tof_dataSize), byref(tof_type))
    tof_data = (c_ubyte * tof_dataSize.value)()
    dll.xv_get_tof_image(byref(tof_data))
    return tof_width, tof_height, tof_edgeTimestamp, tof_hostTimestamp, tof_data, tof_dataSize, tof_type

def xv_get_sgbm():
    dll.xv_get_sgbm_info(byref(sgbm_width), byref(sgbm_height),byref(sgbm_edgeTimestamp),  byref(sgbm_hostTimestamp), byref(sgbm_dataSize))
    sgbm_data = (c_ubyte * sgbm_dataSize.value)()
    dll.xv_get_sgbm_image(byref(sgbm_data))
    return sgbm_width, sgbm_height, sgbm_edgeTimestamp, sgbm_hostTimestamp, sgbm_data, sgbm_dataSize

def xv_get_sn():
    sn = c_char_p()
    dll.xv_get_sn(byref(sn))
    return sn

def xv_get_fisheye_intrinsics():
    fe_trans_size = c_int()
    fe_ucm_size = c_int()
    dll.xv_get_fe_camera_intrinsics_param(byref(fe_trans_size), byref(fe_ucm_size))
    fe_transform = (Transform_Matrix * fe_trans_size.value)()
    fe_ucm = (UnifiedCameraModel * fe_ucm_size.value)()
    dll.xv_get_fe_camera_intrinsics(fe_transform, fe_ucm)
    return fe_trans_size, fe_ucm_size, fe_transform, fe_ucm

def xv_get_rgb_intrinsics():
    rgb_trans_size = c_int()
    rgb_pdcm_size = c_int()
    dll.xv_get_rgb_camera_intrinsics_param(byref(rgb_trans_size), byref(rgb_pdcm_size))
    rgb_transform = (Transform_Matrix * rgb_trans_size.value)()
    rgb_pdcm = (PolynomialDistortionCameraModel * rgb_pdcm_size.value)()
    dll.xv_get_rgb_camera_intrinsics(rgb_transform, rgb_pdcm)
    return rgb_trans_size, rgb_pdcm_size, rgb_transform, rgb_pdcm

def xv_get_tof_intrinsics():
    tof_trans_size = c_int()
    tof_pdcm_size = c_int()
    dll.xv_get_tof_camera_intrinsics_param(byref(tof_trans_size), byref(tof_pdcm_size))
    tof_transform = (Transform_Matrix * tof_trans_size.value)()
    tof_pdcm = (PolynomialDistortionCameraModel * tof_pdcm_size.value)()
    dll.xv_get_tof_camera_intrinsics(tof_transform, tof_pdcm)
    return tof_trans_size, tof_pdcm_size, tof_transform, tof_pdcm

def xv_set_rgb_camera_resolution(resolution):
    dll.xv_set_rgb_camera_resolution(resolution)

def xv_get_fe_april_tags():
    tagSize = c_int()
    dll.xv_get_fe_tag_size( byref(tagSize))
    tags = (TagData * tagSize.value)()
    dll.xv_get_fe_tag_detection(tags)
    return tags

def xv_get_fe_april_tags_withslam(tagFamily, size):
    tagSlamPosesSize = c_int()
    dll.xv_get_fe_tag_withslam_size(tagFamily, size, byref(tagSlamPosesSize))
    tagSlamPoses = (TagData * tagSlamPosesSize.value)()
    dll.xv_detect_fe_tags_withslam(tagSlamPoses)
    return tagSlamPoses

def xv_get_fe_april_tags_withFE(tagFamily, size):
    tagFEPosesSize = c_int()
    dll.xv_get_fe_tag_withFE_size(tagFamily, size, byref(tagFEPosesSize))
    tagFEPoses = (TagData * tagFEPosesSize.value)()
    tagFETrans = (TagTransData * tagFEPosesSize.value)()
    dll.xv_detect_fe_tags_withFE(tagFEPoses, tagFETrans)
    return tagFEPoses, tagFETrans

def xv_set_fe_autoExposure(isautoExposure):
    dll.xv_set_fe_autoExposure(isautoExposure)

def xv_set_fe_exposureTimeMs(exposureTimeMs):
    dll.xv_set_fe_exposureTimeMs(exposureTimeMs)

def xv_set_fe_gain(gain):
    dll.xv_set_fe_gain(gain)

def xv_set_fe_gain_and_exposureTimeMs(gain, exposureTimeMs):
    dll.xv_set_fe_gain_and_exposureTimeMs(gain, exposureTimeMs)

def xv_get_rgb_april_tags():
    tagSize = c_int()
    dll.xv_get_rgb_tag_size( byref(tagSize))
    tags = (TagData * tagSize.value)()
    dll.xv_get_rgb_tag_detection(tags)
    return tags
