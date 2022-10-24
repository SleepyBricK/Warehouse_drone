#include <iostream>
#include <queue>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/utility.hpp>


constexpr float CONFIDENCE_THRESHOLD = 0.2;
constexpr float NMS_THRESHOLD = 0.4;

// std::string yolo_weights = "/home/clover/weights/yolov4-tiny.weights";
// std::string yolo_config = "/home/clover/weights/yolov4-tiny.cfg";



const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};

std::string yolo_weights, yolo_config;
cv::dnn::Net net; std::vector<std::string> output_names;

bool detected = false;
std::vector<int> indices;
std::vector<cv::Rect> boxes;
std::vector<float> scores;

cv::Mat frame, blob, labeled_frame, det_frame;
std::vector<cv::Mat> detections;
auto dnn_start = std::chrono::steady_clock::now();
auto dnn_end = std::chrono::steady_clock::now();


image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
cv_bridge::CvImagePtr cv_ptr;

std::string topic_in, topic_out;

bool streaming_started = false;

void detect(){
    std::cout << "Detection thread started!\n";
    while(true){
        if (det_frame.empty()) continue;

        //cv::resize(det_frame, blob, cv::Size(416, 416), cv::Scalar(), true, false, CV_32F);
        cv::dnn::blobFromImage(det_frame, blob, 0.00392, cv::Size(416, 416), cv::Scalar(), true, false, CV_32F);

        net.setInput(blob);

        auto _dnn_start = std::chrono::steady_clock::now();
        net.forward(detections, output_names);
        auto _dnn_end = std::chrono::steady_clock::now();

        dnn_start = _dnn_start; dnn_end = _dnn_end;

        std::vector<int> _indices;
        std::vector<cv::Rect> _boxes;
        std::vector<float> _scores;

        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * det_frame.cols;
                auto y = output.at<float>(i, 1) * det_frame.rows;
                auto width = output.at<float>(i, 2) * det_frame.cols;
                auto height = output.at<float>(i, 3) * det_frame.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);

                auto confidence = *output.ptr<float>(i, 5);

                /*
                std::cout << "x=" << x << "; y=" << y << "; width=" << width << "; height=" << height
                << "; conf=" << confidence
                << "; cols=" <<  det_frame.cols
                << "; rows=" <<  det_frame.rows
                << std::endl;
                */

                if (confidence >= CONFIDENCE_THRESHOLD)
                {
                    std::cout << "FOUND HUMAN!!!\n";
                    _boxes.push_back(rect);
                    _scores.push_back(confidence);
                }
            }
        }


        cv::dnn::NMSBoxes(_boxes, _scores, 0.0, NMS_THRESHOLD, _indices);
        boxes = _boxes;
        scores = _scores;
        indices = _indices;
        detected = true;

    }

}

void imageCb(const sensor_msgs::ImageConstPtr& msg){
    //std::cout << "Caught frame!\n";
    try {
        cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    } catch ( cv_bridge::Exception& e ) {
        ROS_ERROR(" cv_bridge exception: %s", e.what() );
        return;
    }


    frame = cv_ptr->image;
    if(!streaming_started){
        streaming_started = true;
    }


}


int main( int argc, char** argv ) {
    std::cout << "Started!\n";

    cv::CommandLineParser parser(argc, argv, "{weights|<none>|}{config|<none>|}{topic_in|<none>|}{topic_out|<none>|}");
    
    yolo_weights = parser.get<std::string>("weights");
    yolo_config = parser.get<std::string>("config");

    topic_in = parser.get<std::string>("topic_in");
    topic_out = parser.get<std::string>("topic_out");

    std::cout << "CMD ARGUMENTS:" << std::endl;
    std::cout << "YOLO weights: " << yolo_weights << std::endl;
    std::cout << "YOLO config: " << yolo_config << std::endl;
    std::cout << "Topic in: " << topic_in << std::endl;
    std::cout << "Topic out: " << topic_out << std::endl;

    net = cv::dnn::readNetFromDarknet(yolo_config, yolo_weights);
    output_names = net.getUnconnectedOutLayersNames();

    ros::init( argc, argv, "image_converter" );
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_sub = it.subscribe(topic_in, 1, &imageCb);
    image_pub = it.advertise(topic_out, 1);

    std::thread detection_thread (detect);

    
    std::cout << "Started loop!\n";
    while(true){
        if (!ros::ok()) return 1;
        ros::spinOnce();
        if (frame.empty() && streaming_started) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        } else if (frame.empty()){
            //std::cout << "Frame is empty!\n";
            continue;
        }
        det_frame = frame.clone();

        auto total_start = std::chrono::steady_clock::now();
        labeled_frame = frame.clone();
        for (size_t i = 0; i < indices.size(); ++i)
        {
            auto color = colors[0];

            auto idx = indices[i];
            const auto& rect = boxes[idx];
            cv::rectangle(labeled_frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);

            std::ostringstream label_ss;
            label_ss << "Person: " << std::fixed << std::setprecision(2) << scores[idx];
            auto label = label_ss.str();

            int baseline;
            auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
            cv::rectangle(labeled_frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
            cv::putText(labeled_frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
        }
        auto total_end = std::chrono::steady_clock::now();

        if(detected){
            float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
            float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
            std::ostringstream stats_ss;
            stats_ss << std::fixed << std::setprecision(2);
            stats_ss << "Inference FPS: " << inference_fps << ", Total FPS: " << total_fps;
            auto stats = stats_ss.str();

            int baseline;
            auto stats_bg_sz = cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
            cv::rectangle(labeled_frame, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(labeled_frame, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
        }
        
        sensor_msgs::ImagePtr msg;
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", labeled_frame).toImageMsg();
        image_pub.publish(msg);
    }
    //ros::spin();
    ros::shutdown();
    return 0;
}