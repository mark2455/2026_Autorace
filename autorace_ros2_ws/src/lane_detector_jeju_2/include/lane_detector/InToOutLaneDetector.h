#ifndef INTOOUTLANEDETECTOR_H
#define INTOOUTLANEDETECTOR_H

#include "LaneDetector.h"

class InToOutLaneDetector: public LaneDetector
{
public:
  InToOutLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count);

  //void setHsvHBinThres(const int h_bin_thres);
  //void setHsvSBinThres(const int s_bin_thres);
  //void setHsvVBinThres(const int v_bin_thres);
  
  void setLeftDetectOffset(const int offset);
  void setRightDetectOffset(const int offset);

  //int getHsvHBinThres() const;
  //int getHsvSBinThres() const;
  //int getHsvVBinThres() const;
  
  int getLeftDetectOffset() const;
  int getRightDetectOffset() const;

  cv::Mat getRoiGrayBinImg() const;
  //cv::Mat getRoiHsvHBinImg() const;
  //cv::Mat getRoiHsvSBinImg() const;
  //cv::Mat getRoiHsvVBinImg() const;

	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	virtual void resetLeftPoint(const int index);
	virtual void resetRightPoint(const int index);

	virtual void updateNextPoint(const int index);

  virtual void showImg() const;

protected:
  cv::Mat roi_bin_img_from_gray_;
  cv::Mat roi_bin_img_from_hsv_h_;
  cv::Mat roi_bin_img_from_hsv_s_;
  cv::Mat roi_bin_img_from_hsv_v_;

  int hsv_h_bin_thres_;
  int hsv_s_bin_thres_;
  int hsv_v_bin_thres_;

  int left_detect_offset_;
  int right_detect_offset_;
};

#endif
