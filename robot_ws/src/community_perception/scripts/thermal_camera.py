#!/usr/bin/env python3
"""Publish synchronized Seekware display and temperature images (SDK 3.4)."""
import ctypes as ct
from pathlib import Path
import platform

import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class Camera(ct.Structure):
    # ABI from third_party/seekware/include/seekware/seekware.h (struct sw).
    _fields_ = [
        ("model", ct.c_uint16), ("serial", ct.c_char * 13),
        ("model_number", ct.c_char * 17), ("manufacture_date", ct.c_char * 33),
        ("fw_major", ct.c_uint8), ("fw_minor", ct.c_uint8),
        ("fw_build_major", ct.c_uint8), ("fw_build_minor", ct.c_uint8),
        ("rows", ct.c_uint16), ("cols", ct.c_uint16),
        ("usb_handle", ct.c_void_p), ("usb_status", ct.c_void_p),
        ("retcode", ct.c_int), ("context", ct.c_void_p),
        ("raw_rows", ct.c_uint16), ("raw_cols", ct.c_uint16),
        ("usb_device", ct.c_void_p),
    ]


def load_sdk():
    package = Path(rospkg.RosPack().get_path("community_perception"))
    architecture = platform.machine()
    if architecture not in ("x86_64", "aarch64", "armv7l"):
        raise RuntimeError("SDK 不支持当前架构: " + architecture)
    library = package / "third_party/seekware/lib" / architecture / "libseekware.so.3.4"
    sdk = ct.CDLL(str(library))
    camera_ptr = ct.POINTER(Camera)
    signatures = {
        "Seekware_Find": [ct.POINTER(camera_ptr), ct.c_int, ct.POINTER(ct.c_int)],
        "Seekware_Open": [camera_ptr],
        "Seekware_Close": [camera_ptr],
        "Seekware_SetSetting": [camera_ptr, ct.c_int, ct.c_int],
        "Seekware_GetImage": [camera_ptr, ct.POINTER(ct.c_uint16),
                              ct.POINTER(ct.c_float), ct.POINTER(ct.c_uint32)],
        "Seekware_GetSpot": [camera_ptr, ct.POINTER(ct.c_float),
                             ct.POINTER(ct.c_float), ct.POINTER(ct.c_float)],
    }
    for name, arguments in signatures.items():
        function = getattr(sdk, name)
        function.argtypes = arguments
        function.restype = ct.c_int
    return sdk


def check(code, operation):
    if code != 0:
        raise RuntimeError("{} 失败，Seekware 错误码 {}".format(operation, code))


def capture():
    sdk = load_sdk()
    cameras = (ct.POINTER(Camera) * 5)()
    count = ct.c_int()
    check(sdk.Seekware_Find(cameras, len(cameras), ct.byref(count)), "查找相机")
    if count.value == 0:
        raise RuntimeError("未发现 Seek 相机，请检查 USB 连接、udev 权限及其他程序占用")
    camera = cameras[0]
    check(sdk.Seekware_Open(camera), "打开相机")
    try:
        # SETTING_TEMP_UNITS=1 / SW_TEMP_C=0; SETTING_TIMEOUT=2 (milliseconds).
        check(sdk.Seekware_SetSetting(camera, 1, 0), "设置摄氏度")
        check(sdk.Seekware_SetSetting(camera, 2, 1000), "设置采集超时")
        rows, cols = camera.contents.rows, camera.contents.cols
        if rows <= 0 or cols <= 0:
            raise RuntimeError("相机返回无效分辨率")
        temperatures = np.empty((rows, cols), dtype=np.float32)
        display = np.empty((rows, cols), dtype=np.uint32)
        spot, minimum, maximum = ct.c_float(), ct.c_float(), ct.c_float()
        bridge = CvBridge()
        image_pub = rospy.Publisher("~image_raw", Image, queue_size=1)
        temperature_pub = rospy.Publisher("~temperature", Image, queue_size=1)
        stats_pub = rospy.Publisher("~temperature_stats", Float32MultiArray, queue_size=1)
        frame_id = rospy.get_param("~frame_id", "thermal_camera_optical_frame")
        frequency = float(rospy.get_param("~publish_rate", 9.0))
        if frequency <= 0:
            raise ValueError("publish_rate 必须大于 0")
        rate = rospy.Rate(frequency)
        rospy.loginfo("Seek 相机已打开: %s, %dx%d", camera.contents.model_number.decode(), cols, rows)
        failures = 0
        while not rospy.is_shutdown():
            status = sdk.Seekware_GetImage(
                camera, None, temperatures.ctypes.data_as(ct.POINTER(ct.c_float)),
                display.ctypes.data_as(ct.POINTER(ct.c_uint32)))
            if status == 4:  # SW_RETCODE_NOFRAME: tolerate brief shutter/processing gaps.
                failures += 1
                if failures >= 3:
                    raise RuntimeError("连续三次采集无帧，请检查设备后重启节点")
                rospy.logwarn_throttle(2.0, "Seek 暂无图像")
                rate.sleep()
                continue
            check(status, "采集图像")
            failures = 0
            check(sdk.Seekware_GetSpot(camera, ct.byref(spot), ct.byref(minimum),
                                       ct.byref(maximum)), "读取温度统计")
            # SDK display pixels are ARGB32; explicitly extract BGR channels.
            bgr = np.stack((display & 255, (display >> 8) & 255,
                            (display >> 16) & 255), axis=-1).astype(np.uint8)
            stamp = rospy.Time.now()
            image = bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            thermal = bridge.cv2_to_imgmsg(temperatures, encoding="32FC1")
            for message in (image, thermal):
                message.header.stamp = stamp
                message.header.frame_id = frame_id
            image_pub.publish(image)
            temperature_pub.publish(thermal)
            stats_pub.publish(Float32MultiArray(data=[spot.value, minimum.value, maximum.value]))
            rospy.loginfo_throttle(2.0, "温度 ℃: 中心 %.1f / 最低 %.1f / 最高 %.1f" %
                                   (spot.value, minimum.value, maximum.value))
            rate.sleep()
    finally:
        code = sdk.Seekware_Close(camera)
        if code != 0:
            rospy.logwarn("关闭 Seek 相机返回错误码 %d", code)


if __name__ == "__main__":
    rospy.init_node("thermal_camera")
    try:
        capture()
    except rospy.ROSInterruptException:
        pass
    except (OSError, RuntimeError, ValueError) as exc:
        rospy.logfatal("红外采集停止: %s", exc)
        raise SystemExit(1)
