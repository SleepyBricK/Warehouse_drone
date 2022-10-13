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


constexpr float CONFIDENCE_THRESHOLD = 0.2;
constexpr float NMS_THRESHOLD = 0.4;

std::string source = "/home/kostya/Downloads/_import_6174f7175a78a2.67231114.mov";
std::string yolo_weights = "/home/kostya/yolov4/yolov4-tiny.weights";
std::string yolo_config = "/home/kostya/yolov4/yolov4-tiny.cfg";

// colors for bounding boxes
const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};


auto net = cv::dnn::readNetFromDarknet(yolo_config, yolo_weights);
auto output_names = net.getUnconnectedOutLayersNames();

bool detected = false;
std::vector<int> indices;
std::vector<cv::Rect> boxes;
std::vector<float> scores;

cv::Mat frame, blob; cv::VideoCapture cap; std::vector<cv::Mat> detections;
auto dnn_start = std::chrono::steady_clock::now();
auto dnn_end = std::chrono::steady_clock::now();

void detect(){
    while(true){
        if (frame.empty()) continue;

        cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(416, 416), cv::Scalar(), true, false, CV_32F);
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
                auto x = output.at<float>(i, 0) * frame.cols;
                auto y = output.at<float>(i, 1) * frame.rows;
                auto width = output.at<float>(i, 2) * frame.cols;
                auto height = output.at<float>(i, 3) * frame.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);

                auto confidence = *output.ptr<float>(i, 5);
                if (confidence >= CONFIDENCE_THRESHOLD)
                {
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

int main(int, char**)
{
    cap.open(source);
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    std::thread detection_thread (detect);

    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing\nPress any key to terminate\n";
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        auto total_start = std::chrono::steady_clock::now();

        for (size_t i = 0; i < indices.size(); ++i)
        {
            auto color = colors[0];

            auto idx = indices[i];
            const auto& rect = boxes[idx];
            cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);

            std::ostringstream label_ss;
            label_ss << "Person: " << std::fixed << std::setprecision(2) << scores[idx];
            auto label = label_ss.str();

            int baseline;
            auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
            cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
            cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
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
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(frame, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
        }
        //cv::namedWindow("output");
        cv::imshow("output", frame);

        if (cv::waitKey(1) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

