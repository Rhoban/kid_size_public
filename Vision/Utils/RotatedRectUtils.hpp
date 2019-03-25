/*************************************************************************
 *  File Name	: 'RotatedRectUtils.hpp'
 *  Author	    : Remi FABRE
 *  Contact      : remi.fabre@labri.fr
 *  Created	    : dimanche, juin 28 2015
 *  Licence	    : http://creativecommons.org/licenses/by-nc-sa/3.0/
 *
 *  Notes:
 *************************************************************************/

#pragma once

#include <opencv2/core/core.hpp>

void drawRotatedRectangle(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 1,
                          int lineType = 8);

/// Makes call easier
void drawRotatedRectangles(cv::Mat& img, const std::vector<cv::RotatedRect>& rects, const cv::Scalar& color,
                           int thickness = 1, int lineType = 8);

// Return the smallest side of the rotated_rect
double getSmallSide(const cv::RotatedRect& rotated_rect);

// Return the biggest side of the rotated_rect
double getBigSide(const cv::RotatedRect& rotated_rect);

/// Return smallSide / bigSide, result is in [0,1]
/// if bigSide is smaller or equal to 0, result is -1
double getAspectRatio(const cv::RotatedRect& rotated_rect);
