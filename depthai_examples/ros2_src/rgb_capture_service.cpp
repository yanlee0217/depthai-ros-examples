
#include "rclcpp/rclcpp.hpp"
#include <ros/console.h>

#include <iostream>
#include <cstdio>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

// std::shared_ptr<dai::DataInputQueue> Queue;
void still_cb(const ros::TimerEvent&){
    ROS_DEBUG("Order 66 callback.")
    //
}
dai::Pipeline createPipeline(){
    dai::Pipeline pipeline;
    // Initial Pipeline initialisation
    auto cam   = pipeline.create<dai::node::ColorCamera>();
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    cam->setPreviewSize(544,306);
    cam->setPreviewKeepAspectRaio(False);
    // Streams
    auto previewCam    = pipeline.create<dai::node::XLinkOut>();
    auto stillMjpegOut = pipeline.create<dai::node::XLinkOut>();
    auto controlIn     = pipeline.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control")  
    stillMjpegOut->setStreamName("still")
    previewCam->setStreamName("preview")  
    // Linking
    cam->still.link(stillMjpegOut->input);
    cam->preview.link(preivewCam->input);
    controlIn->out.link(cam->inputControl);
    return pipeline;
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("OAK-D");

    // OAK-D camera setup. Total of 2 output queues
    dai::Pipeline pipeline = createPipeline();
    dai::Device device(pipeline);
    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", 30, false);
    std::shared_ptr<dai::DataOutputQueue> stillQueue   = device.getOutputQueue("still", 5, false);
   
    /*Control queue stuffs
    std::shared_ptr<dai::DataOutputQueue> stillQueue   = device.getInputQueue("control");
    dai::CameraControl ctrl;
    ctrl.setCaptureStill(true);
    controlQueue->send(ctrl);    
    */

    // Setup node tf and params
    std::string tfPrefix;
    std::string cameraParamUri = "package://depthai_examples/params/camera";
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    
    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    // ====== BridgePublisher Syntax (ROS2) ======
    // BridgePublisher(std::shared_ptr<dai::DataOutputQueue>    daiMessageQueue,
    //                 std::shared_ptr<rclcpp::Node>            node,
    //                 std::string                              rosTopic,
    //                 ConvertFunc                              converter,
    //                 size_t                                   qosHistoryDepth,
    //                 ImageMsgs::CameraInfo                    cameraInfoData,
    //                 std::string);                            cameraName
    // rclcpp::TimerBase::SharedPtr timer;
    // timer = node->create_wall_timer(2s, still_cb);
    
    
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "rgb", false); 
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> previewPub (previewQueue,
                                                                                  node, 
                                                                                  std::string("preview/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter,
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  color_uri,
                                                                                  "preview");
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> capturePub(stillQueue,
                                                                                  node, 
                                                                                  std::string("still/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter,
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  color_uri,
                                                                                  "still");
    previewPub.addPublisherCallback();
    stillPub.addPublisherCallback();
    rclcpp::spin(node);
    return 0;
}

