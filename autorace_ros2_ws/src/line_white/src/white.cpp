#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <coss_msgs/msg/coss.hpp>

class WhiteLineDetector {
public:
    WhiteLineDetector() : white_pixel_threshold(20000) { // 기본 값 설정 대회 : 25000 숙소 노랑 : 20000
        // ROS 초기화 및 토픽 설정
        ros::NodeHandle nh;
        cam_sub = nh.subscribe("/usb_cam/image_rect_color", 10, &WhiteLineDetector::camCallback, this);
        white_line_pub = nh.advertise<coss_msgs::msg::Coss>("/stopline_detected", 10);
        
        // ROS 파라미터에서 사용자 설정 픽셀 기준 가져오기 (없으면 기본값 사용)
        nh.param("white_pixel_threshold", white_pixel_threshold, white_pixel_threshold);
    }

    // 카메라 콜백 함수
    void camCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img = cv_ptr->image;
            detectWhiteLine();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not decode image: %s", e.what());
        }
    }

    // 하얀색 선 감지 함수
    void detectWhiteLine() {
        if (img.empty()) return;

        cv::Mat img_hsv, white_mask;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // 방 피팅 HSV 범위 설정 노랑
        // cv::inRange(img_hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), white_mask);

        // 대회 HSV 범위 설정
        cv::inRange(img_hsv, cv::Scalar(0, 0, 80), cv::Scalar(180, 105, 230), white_mask);

        // 하얀색 HSV 범위 설정
        // cv::inRange(img_hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 70, 230), white_mask);

        // 지하1층 하얀색 값
        //cv::inRange(img_hsv, cv::Scalar(0, 0, 210), cv::Scalar(150, 85, 250), white_mask);
        // 밝은 하얀색 HSV 범위
        //cv::inRange(img_hsv, cv::Scalar(0, 0, 230), cv::Scalar(180, 30, 255), bright_white_mask);
        // 일반 하얀색 HSV 범위
        //cv::inRange(img_hsv, cv::Scalar(0, 0, 210), cv::Scalar(180, 50, 255), white_mask);
        // 어두운 하얀색 HSV 범위
        //cv::inRange(img_hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 70, 230), dark_white_mask);

        // ROI 설정 (이미지 하단 % 사용)
        int roi_start_row = img.rows * (0.7) ;
        cv::Mat roi = white_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());

        int white_pixel_count = cv::countNonZero(roi);
        ROS_INFO("White pixel count: %d", white_pixel_count);

        // 하얀색 픽셀 감지 여부에 따라 상태 업데이트
        coss_msgs::msg::Coss coss_msg;
        if (white_pixel_count > white_pixel_threshold) { // launch
            ROS_INFO("White line detected");
            white_detected = true;
        } else {
            white_detected = false;
        }

        // 퍼블리시
        coss_msg.state2 = white_detected;
        white_line_pub.publish(coss_msg);

        // 디버깅을 위해 이미지 시각화 (옵션)
        cv::imshow("White Line Detection", roi);
        cv::waitKey(1);
    }

private:
    ros::Subscriber cam_sub;
    ros::Publisher white_line_pub;
    cv::Mat img;
    bool white_detected = false; // 상태 저장 변수
    int white_pixel_threshold;   // 하얀색 픽셀 수 기준
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "white_line_detector");

    WhiteLineDetector white_line_detector;
    ros::spin();

    return 0;
}
