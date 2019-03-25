#ifndef BLOB_UTILS_HPP
#define BLOB_UTILS_HPP

#include <opencv2/core/core.hpp>

/* This function add the blobs on a binary image to the second parameter
 * blobs is not cleared by this function.
 * In the binary image, there must be only two possible values:
 * - 0 background
 * - wishedValue :
 * Based on :
 * - http://nghiaho.com/uploads/code/opencv_connected_component/blob.cpp
 */
void addBlobs(const cv::Mat& binaryImg, std::vector<std::vector<cv::Point2i>>& blobs, bool gpuOn, float fgValue = 255,
              cv::Mat* output = NULL);

void addBlobsRects(const cv::Mat& binary, std::vector<cv::Rect>& rects, bool gpuOn, float fgValue = 255,
                   cv::Mat* output = NULL);

/* TODO */
void colorBlobs(cv::Mat& output, std::vector<std::vector<cv::Point2i>>& blobs, bool randomColor = true,
                cv::Scalar color = cv::Scalar(255, 255, 255));

void FindBlobsGraph(const cv::Mat& binary, const cv::Mat& whiteImg, std::vector<std::pair<int, cv::Rect>>& blobRect,
                    cv::Mat& adjMat, int maxDist = 1);

void paintBlob(cv::Mat& blobPainted, std::vector<cv::Point2i> blob, unsigned char value = 255);
#endif  // BLOB_UTILS_HPP
