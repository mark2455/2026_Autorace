#include "lane_detector/InToOutLaneDetector.h"

using namespace std;
using namespace cv;


InToOutLaneDetector::InToOutLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count)
  : LaneDetector(width, height, steer_max_angle, detect_line_count)
{}

//void InToOutLaneDetector::setHsvHBinThres(const int h_bin_thres) { hsv_h_bin_thres_ = h_bin_thres; }  //========================수정 해야함
//void InToOutLaneDetector::setHsvSBinThres(const int s_bin_thres) { hsv_s_bin_thres_ = s_bin_thres; } 
//void InToOutLaneDetector::setHsvVBinThres(const int v_bin_thres) { hsv_v_bin_thres_ = v_bin_thres; } 

void InToOutLaneDetector::setLeftDetectOffset(const int offset) { left_detect_offset_ = offset; }
void InToOutLaneDetector::setRightDetectOffset(const int offset) { right_detect_offset_ = offset; }

//int InToOutLaneDetector::getHsvHBinThres() const { return hsv_h_bin_thres_; }  //========================수정 해야함
//int InToOutLaneDetector::getHsvSBinThres() const { return hsv_s_bin_thres_; }
//int InToOutLaneDetector::getHsvVBinThres() const { return hsv_v_bin_thres_; }

int InToOutLaneDetector::getRightDetectOffset() const { return right_detect_offset_; }
int InToOutLaneDetector::getLeftDetectOffset() const { return left_detect_offset_; }

cv::Mat InToOutLaneDetector::getRoiGrayBinImg() const { return roi_bin_img_from_gray_; }

//cv::Mat InToOutLaneDetector::getRoiHsvHBinImg() const { return roi_bin_img_from_hsv_h_; } 
//cv::Mat InToOutLaneDetector::getRoiHsvSBinImg() const { return roi_bin_img_from_hsv_s_; }      //========================수정 해야함
//cv::Mat InToOutLaneDetector::getRoiHsvVBinImg() const { return roi_bin_img_from_hsv_v_; } 


/*cv::Scalar lower_white = cv::Scalar(0,0,200);
	cv::Scalar upper_white = cv::Scalar(180,255,255);
	cv::inRange(roi_hsv_img, lower_white, upper_white, roi_hsv_img);*/


void InToOutLaneDetector::cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size)    //========================수정 해야함
{
  Mat yellow_mask, yellow_image;
  //지하1층 차선 값 최소값, 최대값.
  Scalar lower_yellow=Scalar(0, 0, 190);
  Scalar upper_yellow=Scalar(150, 85, 255);
  // 그냥 노란색 차선 값 최소값, 최대값.
  // Scalar lower_yellow=Scalar(20, 100, 100);
  // Scalar upper_yellow=Scalar(30, 255, 255);
  // 밝은 노란색 차선 값 최소값, 최대값.
  // Scalar lower_yellow=Scalar(20, 150, 150);
  // Scalar upper_yellow=Scalar(30, 255, 255);
  // 어두운 노란색 차선 값 최소값, 최대값.
  // Scalar lower_yellow=Scalar(15, 100, 100);
  // Scalar upper_yellow=Scalar(30, 200, 200);

  Mat roi_hsv_img;
  Mat roi_gray_img;
  Mat final_gray;

  // 1. convert roi to hsv
  Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));


  //Mat roi_color_img = resized_img_(Rect((0,360),(1280,360),1280, 720));
  cvtColor(roi_color_img, roi_hsv_img, COLOR_BGR2HSV);

  inRange(roi_hsv_img, lower_yellow, upper_yellow, yellow_mask);  

  cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
  
  threshold(roi_gray_img, final_gray, 180, 255, THRESH_BINARY);

  //imshow("gray", roi_gray_img);

  //imshow("yellow_mask", yellow_mask);

  // imshow("roi_gray_img", final_gray);





  Mat intersection = Mat::zeros(final_gray.size(), final_gray.type());

    // 5. Find the intersection manually using a loop
    for (int i = 0; i < final_gray.rows; ++i) 
    {
        for (int j = 0; j < final_gray.cols; ++j) 
        {
            if (final_gray.at<uchar>(i, j) == 255 && yellow_mask.at<uchar>(i, j) == 255) 
            {
                intersection.at<uchar>(i, j) = 255;
            }
        }
    }


 
  Mat new_color;
  Mat new_color2;
  cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
  cv::Mat opened_image2;
  cv::morphologyEx(intersection, opened_image2, cv::BORDER_CONSTANT, kernel2);
  new_color2 = opened_image2.clone();




    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));


    // 열림(Opening) 연산 수행
    cv::Mat opened_image;
    cv::morphologyEx(new_color2, opened_image, cv::MORPH_CLOSE, kernel);

    // 필터링할 이미지 정의 (이 예제에서는 모폴로지 연산 결과를 사용)
    new_color = opened_image.clone();  // 복사하여 새로운 이미지 생성
    // 필터링을 위한 코드를 여기에 추가
    // 예를 들어, 특정 임계값 이상인 픽셀을 남기고 나머지를 제거하는 경우:
    // cv::threshold(opened_image, new_color, 0, 255, cv::THRESH_BINARY);//(128,255)


  // Canny(intersection, intersection, 0, 360);

  Canny(new_color, new_color, 0, 360);
  // imshow("new_color", new_color);
  // imshow("new_color", new_color2);




  Mat final = Mat::zeros(new_color.size(), new_color.type());

    // 5. Find the intersection manually using a loop
    for (int i = 0; i < new_color.rows; ++i) 
    {
        for (int j = 0; j < new_color.cols; ++j) 
        {
            if (new_color.at<uchar>(i, j) == 255 && intersection.at<uchar>(i, j) == 255) 
            {
                final.at<uchar>(i, j) = 255;
            }
        }
    }





    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(30, 20));


    // 열림(Opening) 연산 수행
    cv::Mat opened_image3;
    cv::morphologyEx(final, opened_image3, cv::MORPH_CLOSE, kernel3);
    Mat final_final;
    final_final = opened_image3.clone();









  // imshow("new_color", new_color);
  // waitKey(1);

  

  //4. add these two binary images
  roi_binary_img_ =  intersection;

  // Mat resized_img;
  // resize(roi_binary_img_, resized_img, Size(1280,720));

  
  imshow("intersection", roi_binary_img_);
  waitKey(1);
}

void InToOutLaneDetector::resetLeftPoint(const int index)
 
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  last_left_point_arr_[index].x = line_point_detector_arr_[index].find_L0_x_in2out(roi_binary_img_, detect_y_offset_arr_[index], last_left_point_arr_[index].x, left_detect_offset_);
  last_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void InToOutLaneDetector::resetRightPoint(const int index)
 
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  last_right_point_arr_[index].x = line_point_detector_arr_[index].find_R0_x_in2out(roi_binary_img_, detect_y_offset_arr_[index], last_right_point_arr_[index].x, right_detect_offset_);
  last_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void InToOutLaneDetector::updateNextPoint(const int index)
 
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  cur_right_point_arr_[index].x = line_point_detector_arr_[index].find_RN_x_in2out(roi_binary_img_, last_right_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD, right_detect_offset_);
	cur_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_right_point_arr_[index].x = cur_right_point_arr_[index].x;

	cur_left_point_arr_[index].x = line_point_detector_arr_[index].find_LN_x_in2out(roi_binary_img_, last_left_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD, left_detect_offset_);
	cur_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_left_point_arr_[index].x = cur_left_point_arr_[index].x;
}

void InToOutLaneDetector::showImg() const
{
  LaneDetector::showImg();
  //imshow("binary from gray", roi_bin_img_from_gray_);
	//imshow("binary from hsv s", roi_bin_img_from_hsv_s_);
	// waitKey(3);
}
