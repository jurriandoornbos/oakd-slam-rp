#include "ros/ros.h"

#include <iostream>
#include <cstdio>
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

static constexpr int fps = 20;

static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;
static std::atomic<bool> downscaleColor{true};

dai::Pipeline createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel){
        // Create pipeline
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;

    // Define sources and outputs

    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    depthOut->setStreamName("depth");
    queueNames.push_back("depth");


    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->initialConfig.setConfidenceThreshold(100);
    stereo -> initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
        // LR-check is required for depth alignment
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    

    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);


    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName, mode;
    std::string cameraParamUri;
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


    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel);

    dai::Device device(pipeline);

    auto depthQueue = device.getOutputQueue("depth", fps, false);
  
    auto calibrationHandler = device.readCalibration();

    // this part would be removed once we have calibration-api
    /*     
     std::string leftUri = cameraParamUri +"/" + "left.yaml";

     std::string rightUri = cameraParamUri + "/" + "right.yaml";

     std::string stereoUri = cameraParamUri + "/" + "right.yaml";
    */
    std::cout << "USB SPEED: " << device.getUsbSpeed() << std::endl;

     

    dai::rosBridge::ImageConverter converter(deviceName + "_right_camera_optical_frame", true);
    auto rgbCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 680, 400); 

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &converter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     fps,
                                                                                     rgbCameraInfo,
                                                                                     "stereo");

    depthPublish.addPubisherCallback();
    ros::spin();
    

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    
    return 0;
}
