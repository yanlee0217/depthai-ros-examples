/**
 * This example shows usage of depth camera in crop mode with the possibility to move the crop.
 * Use 'WASD' in order to do it.
 */
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <memory>

#include <depthai_ros_msgs/srv/NormalizedImageCrop.hpp>
#include <sensor_msgs/msg/Image.hpp>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include "depthai/depthai.hpp"

std::shared_ptr<dai::DataInputQueue> configQueue;

void cropDepthImage(depthai_ros_msgs::srv::NormalizedImageCrop::Request request, depthai_ros_msgs::srv::NormalizedImageCrop::Response response){
    dai::ImageManipConfig cfg;
    cfg.setCropRect(request.topLeft.x, request.topLeft.y, request.bottomRight.x, request.bottomRight.y);
    configQueue->send(cfg);
    response->status = true;
    return;    
}

int main() {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("depth_crop_control");

    rclcpp::Service<depthai_ros_msgs::srv::NormalizedImageCrop>::SharedPtr service =
    node->create_service<depthai_ros_msgs::srv::NormalizedImageCrop>("crop_control_srv", &cropDepthImage);

    // For ROS1
    // ros::ServiceServer service = n.advertiseService("crop_control_srv", cropDepthImage);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb     = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc   = pipeline.create<dai::node::VideoEncoder>();
    auto stillEnc   = pipeline.create<dai::node::VideoEncoder>();
    auto ctrIn      = pipeline.create<dai::node::XLinkIn>();
    auto videoOut   = pipeline.create<dai::node::XLinkOut>();
    auto stillOut   = pipeline.create<dai::node::XLinkOut>();
    auto preview    = pipeline.create<dai::node::XLinkOut>();
    ctrIn->setStreamName("control");
    videoOut->setStreamName("video");
    stillOut->setStreamName("still");
    preview->setStreamName("preview");

    // Linking
    camRgb->video.link(videoEnc->input);
    camRgb->still.link(stillEnc->input);
    camRgb->preview.link(preview->input);
    ctrIn->out.link(camRgb->inputControl);
    videoEnc->bitstream.link(videoOut->input);
    stillEnc->bitstream.link(stillOut->input);
    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto depthQueue = device.getOutputQueue(xout->getStreamName(), 5, false);
    configQueue = device.getInputQueue(control->getCaptureStill());

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter depthConverter(cameraName + "_right_camera_optical_frame", true);
    // TODO(sachin): Modify the calibration based on crop from service
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight); 

    // ====== BridgePublisher Syntax (ROS2) ======
    // BridgePublisher(std::shared_ptr<dai::DataOutputQueue>    daiMessageQueue,
    //                 std::shared_ptr<rclcpp::Node>            node,
    //                 std::string                              rosTopic,
    //                 ConvertFunc                              converter,
    //                 size_t                                   qosHistoryDepth,
    //                 ImageMsgs::CameraInfo                    cameraInfoData,
    //                 std::string);                            cameraName


    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                    node, 
                                                                                    std::string("stereo/depth"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &depthConverter, // since the converter has the same frame name
                                                                                                    // and image type is also same we can reuse it
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    rightCameraInfo,
                                                                                    "stereo");
    depthPublish.addPublisherCallback();  //dai creates its own publisher thread
    rclcpp::spin(node);

    return 0;
}
