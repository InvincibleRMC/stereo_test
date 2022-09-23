// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo where each image
// message consists of a left and right image concatenated to form a single
// double-wide image.  This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right images.
//
// This is a modified version of public domain code posted by PeteBlackerThe3rd
// in response to my question on rclcpp Answers:
// https://answers.rclcpp.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
//
// -- dbc

//#include <rclcpp2/rclcpp.h>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

//#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/msg/camera_info.hpp>


// If non-zero, outputWidth and outputHeight set the size of the output images.
// If zero, the outputWidth is set to 1/2 the width of the input image, and
// outputHeight is the same as the height of the input image.
int outputWidth, outputHeight;

// Input image subscriber.
image_transport::Subscriber imageSub;

// Left and right image publishers.
image_transport::Publisher leftImagePublisher, rightImagePublisher;

// Left and right camera info publishers and messages.
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> leftCameraInfoPublisher, rightCameraInfoPublisher;
sensor_msgs::msg::CameraInfo leftCameraInfoMsg, rightCameraInfoMsg;

// Camera info managers.
camera_info_manager::CameraInfoManager *left_cinfo_;
camera_info_manager::CameraInfoManager *right_cinfo_;


// Image capture callback.
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    // Get double camera image.
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat image = cvImg->image;

    // If there are any subscribers to either output topic then publish images
    // on them.
    if (leftImagePublisher.getNumSubscribers() > 0 ||
        rightImagePublisher.getNumSubscribers() > 0)
    {
        // Define the relevant rectangles to crop.
        cv::Rect leftROI, rightROI;
        leftROI.y = rightROI.y = 0;
        leftROI.width = rightROI.width = image.cols / 2;
        leftROI.height = rightROI.height = image.rows;
        leftROI.x = 0;
        rightROI.x = image.cols / 2;

        // Crop images.
        cv::Mat leftImage = cv::Mat(image, leftROI);
        cv::Mat rightImage = cv::Mat(image, rightROI);

        // Apply scaling, if specified.
        bool use_scaled;
        cv::Mat leftScaled, rightScaled;
        if (use_scaled = (outputWidth > 0 && outputHeight > 0))
        {
            cv::Size sz = cv::Size(outputWidth, outputHeight);
            cv::resize(leftImage, leftScaled, sz);
            cv::resize(rightImage, rightScaled, sz);
        }

        // Publish.
        cv_bridge::CvImage cvImage;
        //Used to be a pointer
        //sensor_msgs::msg::Image::SharedPtr& img;
        cvImage.encoding = msg->encoding;
        cvImage.header.frame_id = msg->header.frame_id;
        cvImage.header.stamp = msg->header.stamp;
        if (leftImagePublisher.getNumSubscribers() > 0
            || (*leftCameraInfoPublisher).get_subscription_count() > 0)
        {
            cvImage.image = use_scaled ? leftScaled : leftImage;
            auto img = cvImage.toImageMsg();
            leftImagePublisher.publish(img);
            leftCameraInfoMsg.header.stamp = img->header.stamp;
            leftCameraInfoMsg.header.frame_id = img->header.frame_id;
            (*leftCameraInfoPublisher).publish(leftCameraInfoMsg);
        }
        if (rightImagePublisher.getNumSubscribers() > 0
            || (*rightCameraInfoPublisher).get_subscription_count() > 0)
        {
            cvImage.image = use_scaled ? rightScaled : rightImage;
            auto img = cvImage.toImageMsg();
            rightImagePublisher.publish(img);
            rightCameraInfoMsg.header.stamp = img->header.stamp;
            rightCameraInfoMsg.header.frame_id = img->header.frame_id;
            (*rightCameraInfoPublisher).publish(rightCameraInfoMsg);
        }
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_main_ = rclcpp::Node::make_shared("stereo_splitter_main");
    auto node_left_ = node_main_->create_sub_node("left");
    auto node_right_ = node_main_->create_sub_node("right");
    

    //rclcpp::spin(node_main_);

    //auto node_main_ = rclcpp::Node("sxs_stereo");
    //auto node_main__left = rclcpp::Node::image_transport(node_main_,"left");
   // auto node_main__right = rclcpp::Node::image_transport(node_main_,"right");
    //rclcpp::NodeHandle node_main_("sxs_stereo");
    //rclcpp::NodeHandle node_main__left(node_main_, "left");
    //rclcpp::NodeHandle node_main__right(node_main_, "right");

    RCLCPP_INFO(node_main_->get_logger(),"made nodes");

    image_transport::ImageTransport it(node_main_);

    RCLCPP_INFO(node_main_->get_logger(),"image transport");

    // Allocate and initialize camera info managers.
    //new camera_info_manager::CameraInfoManager::CameraInfoManager(node_left_,"left camera","");
    camera_info_manager::CameraInfoManager left_cinfo_(node_left_.get());
    camera_info_manager::CameraInfoManager right_cinfo_(node_right_.get());
    RCLCPP_INFO(node_main_->get_logger(),"camera managers");
    left_cinfo_.loadCameraInfo("");
    right_cinfo_.loadCameraInfo("");

    RCLCPP_INFO(node_main_->get_logger(),"camera managers");

    // Pre-fill camera_info messages.
    leftCameraInfoMsg = left_cinfo_.getCameraInfo();
    rightCameraInfoMsg = right_cinfo_.getCameraInfo();

    // Load node settings.
    std::string inputImageTopic, leftOutputImageTopic, rightOutputImageTopic,
        leftCameraInfoTopic, rightCameraInfoTopic;

    //TODO: add namespace thingy for /stereo
    
    // should use declare_parameters probably
    inputImageTopic = (*node_main_).declare_parameter("input_image_topic","input_image_topic_not_set");
    leftOutputImageTopic = (*node_main_).declare_parameter("left_output_image_topic","/stereo/left/image_raw");
    rightOutputImageTopic = (*node_main_).declare_parameter("right_output_image_topic","/stereo/right/image_raw");
    leftCameraInfoTopic = (*node_main_).declare_parameter("left_camera_info_topic","/stereo/left/camera_info");
    rightCameraInfoTopic = (*node_main_).declare_parameter("right_camera_info_topic","/stereo/right/camera_info");
    outputWidth = (*node_main_).declare_parameter("output_width",0);
    outputHeight = (*node_main_).declare_parameter("output_height",0);

    RCLCPP_INFO(node_main_->get_logger(),"declared params");

    RCLCPP_INFO(node_main_->get_logger(),"input topic to stereo splitter=%s\n", inputImageTopic.c_str());

    // Register publishers and subscriber.
    leftCameraInfoPublisher = node_left_->create_publisher<sensor_msgs::msg::CameraInfo>(leftCameraInfoTopic.c_str(), rclcpp::SensorDataQoS());
    rightCameraInfoPublisher = node_right_->create_publisher<sensor_msgs::msg::CameraInfo>(rightCameraInfoTopic.c_str(), rclcpp::SensorDataQoS());
    imageSub = it.subscribe(inputImageTopic.c_str(), 2, imageCallback);
    leftImagePublisher = it.advertise(leftOutputImageTopic.c_str(), 1);
    rightImagePublisher = it.advertise(rightOutputImageTopic.c_str(), 1);

    RCLCPP_INFO(node_main_->get_logger(),"pubs and subs");

    // Run node until cancelled.
    rclcpp::spin(node_main_);

    // De-allocate CameraInfoManagers.
    left_cinfo_.~CameraInfoManager();
    right_cinfo_.~CameraInfoManager();
}