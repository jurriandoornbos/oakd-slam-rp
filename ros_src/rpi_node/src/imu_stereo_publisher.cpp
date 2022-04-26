#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <vision_msgs/Detection2DArray.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

#include <depthai_bridge/ImuConverter.hpp>

static constexpr int fps = 60;
static constexpr int hz = 200;

dai::Pipeline createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, std::string nnPath) {
    dai::Pipeline pipeline;

    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto monoRight   = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
    auto stereo      = pipeline.create<dai::node::StereoDepth>();

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRGB = pipeline.create<dai::node::XLinkOut>();

    xoutRGB->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    



    // XLinkOut Stereo
    xoutLeft->setStreamName("rectified_left");
    xoutRight->setStreamName("rectified_right");

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->setFps(fps);
    monoRight->setFps(fps);

    // int maxDisp = 96;
    // if (extended) maxDisp *= 2;
    // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    // StereoDepth
    //stereo->initialConfig.setConfidenceThreshold(250);
    //stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    //stereo->initialConfig.setLeftRightCheckThreshold(1);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->rectifiedLeft.link(xoutLeft->input);
    stereo->rectifiedRight.link(xoutRight->input);


        // Add IMU To the pipeline

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

    // add mobilenet to the pipeline
    //auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    //auto nnOut = pipeline.create<dai::node::XLinkOut>();

    //nnOut->setStreamName("detections");

    //detectionNetwork->setConfidenceThreshold(0.5f);
    //detectionNetwork->setBlobPath(nnPath);

    //camRgb->preview.link(detectionNetwork->input);

    //detectionNetwork->passthrough.link(xoutRGB->input);
    
    //detectionNetwork->out.link(nnOut->input);

    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName, mode;
    std::string cameraParamUri;
    std::string nnPath(BLOB_PATH);
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;

    badParams += !pnh.getParam("camera_name", deviceName);
    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("lrcheck",  lrcheck);
    badParams += !pnh.getParam("extended",  extended);
    badParams += !pnh.getParam("subpixel",  subpixel);
    

    if (badParams > 0)
    {   
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }


    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel,nnPath);

    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("rectified_left", 30, false);
    auto rightQueue = device.getOutputQueue("rectified_right", 30, false);
    auto rgbQueue = device.getOutputQueue("rgb", 30, false);
    
    //auto nNetDataQueue = device.getOutputQueue("detections", 30, false);


    auto imuQueue = device.getOutputQueue("imu", hz, false);

    auto calibrationHandler = device.readCalibration();

    // this part would be removed once we have calibration-api
    /*     
     std::string leftUri = cameraParamUri +"/" + "left.yaml";

     std::string rightUri = cameraParamUri + "/" + "right.yaml";

     std::string stereoUri = cameraParamUri + "/" + "right.yaml";
    */
    std::cout << "USB SPEED: " << device.getUsbSpeed() << std::endl;

    //dai::rosBridge::ImgDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", 300, 300, false);
    //dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueue,
    //                                                                                                     pnh, 
    //                                                                                                   std::string("color/mobilenet_detections"),
    //                                                                                                     std::bind(static_cast<void(dai::rosBridge::ImgDetectionConverter::*)(std::shared_ptr<dai::ImgDetections>, 
    //                                                                                                     vision_msgs::Detection2DArray&)>(&dai::rosBridge::ImgDetectionConverter::toRosMsg), 
    //                                                                                                     &detConverter,
    //                                                                                                     std::placeholders::_1, 
    //                                                                                                     std::placeholders::_2), 
    //                                                                                                     30);
    //
    //detectionPublish.startPublisherThread();


    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
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

    leftPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
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

    rightPublish.addPubisherCallback();

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
    rgbPublish.addPubisherCallback();

    dai::rosBridge::ImuConverter imuConverter(deviceName + "_imu_frame");
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

    imuPublish.addPubisherCallback();  

    ros::spin();
    

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

  
    return 0;
}
