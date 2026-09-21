#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

class LaneDetectorROS
{
public:
    LaneDetectorROS() : nh_("~")
    {
        sub_ = nh_.subscribe("/usb_cam/image_proc", 1, &LaneDetectorROS::cameraCB, this);
        pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/cam_data", 1);
    }

    void cameraCB(const sensor_msgs::CompressedImage::ConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat src = cv_ptr->image;
            cv::resize(src, src, cv::Size(640, 360));

            cv::Mat dst;
            cv::Canny(src, dst, 50, 200, 3);

            cv::Mat cdst;
            cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
            cv::Mat cdstP = cdst.clone();

            std::vector<cv::Vec2f> lines;
            cv::HoughLines(dst, lines, 1, CV_PI / 180, 150, 0, 0);

            if (!lines.empty())
            {
                for (size_t i = 0; i < lines.size(); ++i)
                {
                    float rho = lines[i][0];
                    float theta = lines[i][1];
                    double a = cos(theta), b = sin(theta);
                    double x0 = a * rho, y0 = b * rho;
                    cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
                    cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
                    cv::line(cdst, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
                }
            }

            std::vector<cv::Vec4i> linesP;
            cv::HoughLinesP(dst, linesP, 1, CV_PI / 180, 50, 50, 10);

            if (!linesP.empty())
            {
                for (size_t i = 0; i < linesP.size(); ++i)
                {
                    cv::Vec4i l = linesP[i];
                    cv::line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
                }
            }

            sensor_msgs::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            cv::imencode(".jpg", cdst, compressed_msg.data);
            pub_.publish(compressed_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node");
    LaneDetectorROS lane_detector;
    ros::spin();
    return 0;
}
