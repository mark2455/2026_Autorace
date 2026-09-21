#ifndef OUTTOINLANEDETECTOR_H
#define OUTTOINLANEDETECTOR_H

#include "LaneDetector.h"

class OutToInLaneDetector: public LaneDetector
{
public:
  OutToInLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count);

	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	virtual void resetLeftPoint(const int index);
	virtual void resetRightPoint(const int index);

	virtual void updateNextPoint(const int index);
};

#endif
