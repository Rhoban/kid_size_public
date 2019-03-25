//*****************************************************************************
//
// File Name	: 'decomp_homography.hpp'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen.000@gmail.com
// Created	: mercredi, juin 21 2017
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	notes
//
//*****************************************************************************

#if !defined(DECOMP_HOMOGRAPHY_HPP)
#define DECOMP_HOMOGRAPHY_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
namespace cv
{
int decomposeHomographyMat(cv::InputArray _H, cv::InputArray _K, cv::OutputArrayOfArrays& _rotations,
                           cv::OutputArrayOfArrays& _translations, cv::OutputArrayOfArrays& _normals);
}
#endif
