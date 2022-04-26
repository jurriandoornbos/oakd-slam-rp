
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>

static constexpr int fps = 60;
static constexpr int hz = 200;

std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution){
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution; 
    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto monoRight   = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
    auto stereo      = pipeline.create<dai::node::StereoDepth>();

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRGB = pipeline.create<dai::node::XLinkOut>();

    xoutRGB->setStreamName("rgb");

    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    

    // XLinkOut
    xoutLeft->setStreamName("left");

    int width, height;
    monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
    width  = 640;
    height = 400;

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->setFps(fps);
    monoRight->setFps(fps);
    // StereoDepth
    //stereo->initialConfig.setConfidenceThreshold(confidence);
    //stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    //stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->rectifiedLeft.link(xoutLeft->input);
    stereo->rectifiedRight.link(xoutRight->input);

    auto imuSensor = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();
    xoutImu->setStreamName("imu");
    
    // enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 100 hz rate
    imuSensor->enableIMUSensor({dai::IMUSensor::LINEAR_ACCELERATION, dai::IMUSensor::GYROSCOPE_CALIBRATED, dai::IMUSensor::ROTATION_VECTOR}, 200);
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imuSensor->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imuSensor->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imuSensor->out.link(xoutImu->input);




    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string tfPrefix, mode;
    std::string cameraParamUri;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence = 200;
    int monoWidth, monoHeight;
    int LRchecktresh = 5;
    std::string monoResolution = "400p";
    dai::Pipeline pipeline;

    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("tf_prefix",        tfPrefix);
    badParams += !pnh.getParam("mode",             mode);
    badParams += !pnh.getParam("lrcheck",          lrcheck);
    badParams += !pnh.getParam("extended",         extended);
    badParams += !pnh.getParam("subpixel",         subpixel);
    badParams += !pnh.getParam("confidence",       confidence);
    badParams += !pnh.getParam("LRchecktresh",     LRchecktresh);
    badParams += !pnh.getParam("monoResolution",   monoResolution);

    if (badParams > 0)
    {   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);

    dai::Device device(pipeline);
   

    auto leftQueue = device.getOutputQueue("rectified_left", 30, false);
    auto rightQueue = device.getOutputQueue("rectified_right", 30, false);
    auto rgbQueue = device.getOutputQueue("rgb", 30, false);
    

    auto imuQueue = device.getOutputQueue("imu", hz, false);

    auto calibrationHandler = device.readCalibration();
    std::cout << "USB SPEED: " << device.getUsbSpeed() << std::endl;

   
   dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 640, 400); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    pnh, 
                                                                                    std::string("left/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    leftCameraInfo,
                                                                                    "left");

    leftPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 640, 400); 

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     pnh, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right");

    rightPublish.addPublisherCallback();

    auto rgbCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 640, 400); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(rgbQueue,
                                                                                    pnh, 
                                                                                    std::string("color/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    rgbCameraInfo,
                                                                                    "color");
    rgbPublish.addPublisherCallback();

    dai::rosBridge::ImuConverter imuConverter(tfPrefix  + "_imu_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(imuQueue,
                                                                                    pnh,
                                                                                    std::string("imu"),
                                                                                    std::bind(static_cast<void(dai::rosBridge::ImuConverter::*)
                                                                                    (std::shared_ptr<dai::IMUData>, sensor_msgs::Imu&)
                                                                                    >(&dai::rosBridge::ImuConverter::toRosMsg),
                                                                                    &imuConverter,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2),
                                                                                    hz,
                                                                                    rightCameraInfo,
                                                                                    "imu");

    imuPublish.addPublisherCallback();  

    ros::spin();
    return 0;
}
