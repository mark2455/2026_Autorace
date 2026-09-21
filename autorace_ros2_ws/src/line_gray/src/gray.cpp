#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <coss_msgs/msg/coss.hpp>

class GrayLineDetector {
public:
    GrayLineDetector() : gray_pixel_threshold(40000) { // 기본 값 설정
        // ROS 초기화 및 토픽 설정
        ros::NodeHandle nh;
        cam_sub = nh.subscribe("/usb_cam/image_rect_color", 10, &GrayLineDetector::camCallback, this);
        gray_line_pub = nh.advertise<coss_msgs::msg::Coss>("/gray_detected", 1);
        
        // ROS 파라미터에서 사용자 설정 픽셀 기준 가져오기 (없으면 기본값 사용)
        nh.param("gray_pixel_threshold", gray_pixel_threshold, gray_pixel_threshold);
    }

    // 카메라 콜백 함수
    void camCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img = cv_ptr->image;
            detectGrayLine();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not decode image: %s", e.what());
        }
    }

    // 회색 선 감지 함수
    void detectGrayLine() {
        if (img.empty()) return;

        cv::Mat img_hsv, gray_mask;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // 회색 HSV 범위 설정
        cv::inRange(img_hsv, cv::Scalar(0, 0, 80), cv::Scalar(180, 105, 230), gray_mask);

        // ROI 설정 (이미지 하단 % 사용)
        int roi_start_row = img.rows * (0.7) ;
        cv::Mat roi = gray_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());

        int gray_pixel_count = cv::countNonZero(roi);
        ROS_INFO("gray pixel count: %d", gray_pixel_count);

        // 회색 픽셀 감지 여부에 따라 상태 업데이트
        coss_msgs::msg::Coss coss_msg;
        if (gray_pixel_count > gray_pixel_threshold) { // launch
            ROS_INFO("Gray line detected");
            gray_detected = true;
            
        } else {
            gray_detected = false;
        }

        // 퍼블리시
        coss_msg.gray_flag = gray_detected;
        gray_line_pub.publish(coss_msg);

        // 디버깅을 위해 이미지 시각화 (옵션)
        cv::imshow("Gray Line Detection", roi);
        cv::waitKey(1);
    }

private:
    ros::Subscriber cam_sub;
    ros::Publisher gray_line_pub;
    cv::Mat img;
    bool gray_detected = false; // 상태 저장 변수
    int gray_count;
    int gray_pixel_threshold;   // 하얀색 픽셀 수 기준
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gray_line_detector");

    GrayLineDetector gray_line_detector;
    ros::spin();

    return 0;
}
