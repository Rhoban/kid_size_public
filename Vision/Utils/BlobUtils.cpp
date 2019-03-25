#include "BlobUtils.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "rhoban_utils/timing/benchmark.h"

using rhoban_utils::Benchmark;
using namespace std;
using namespace cv;

// Function from :
// - http://nghiaho.com/uploads/code/opencv_connected_component/blob.cpp
void addBlobs(const Mat& binary, vector<vector<Point2i>>& blobs, bool gpuOn, float fgValue, Mat* output)
{
  blobs.clear();

  // Fill the label_image with the blobs
  // 0                - background
  // fgValue          - unlabelled foreground
  // [fgValue+1, ...] - labelled foreground
  bool toBeReleased = false;
  if (output == NULL)
  {
    toBeReleased = true;
    output = new cv::Mat();
  }
  if (gpuOn)
  {
    Benchmark::open("(OpenCL) Addblob: binary_convert");
    cv::UMat gpuIn, gpuOut;
    binary.copyTo(gpuIn);
    gpuIn.convertTo(gpuOut, CV_32FC1);
    gpuOut.copyTo(*output);
    Benchmark::close("(OpenCL) Addblob: binary_convert");
  }
  else
  {
    Benchmark::open("(OpenCL) Addblob: binary_convert");
    binary.convertTo(*output, CV_32FC1);
    Benchmark::close("(OpenCL) Addblob: binary_convert");
  }

  // starts at fgValue+1 because fgValue is used already
  int label_count = fgValue + 1;

  Benchmark::open("Addblob: loops");
  for (int y = 0; y < output->rows; y++)
  {
    for (int x = 0; x < output->cols; x++)
    {
      float checker = output->at<float>(y, x);
      if (checker != fgValue)
      {
        continue;
      }

      cv::Rect rect;
      Benchmark::open("Addblob: Floodfill");
      cv::floodFill(*output, cv::Point(x, y), label_count, &rect, 0, 0, 8);
      Benchmark::close("Addblob: Floodfill");

      std::vector<cv::Point2i> blob;

      for (int i = rect.y; i < (rect.y + rect.height); i++)
      {
        for (int j = rect.x; j < (rect.x + rect.width); j++)
        {
          float check = output->at<float>(i, j);
          if (check != label_count)
          {
            continue;
          }

          blob.push_back(cv::Point2i(j, i));
        }
      }

      blobs.push_back(blob);

      label_count++;
    }
  }
  Benchmark::close("Addblob: loops");
  if (toBeReleased)
  {
    delete output;
  }
}

void addBlobsRects(const Mat& binary, std::vector<cv::Rect>& rects, bool gpuOn, float fgValue, cv::Mat* output)
{
  rects.clear();

  // Fill the label_image with the blobs
  // 0                - background
  // fgValue          - unlabelled foreground
  // [fgValue+1, ...] - labelled foreground
  bool toBeReleased = false;
  if (output == NULL)
  {
    toBeReleased = true;
    output = new cv::Mat();
  }
  if (gpuOn)
  {
    Benchmark::open("(OpenCL) Addblob: binary_convert");
    cv::UMat gpuIn, gpuOut;
    binary.copyTo(gpuIn);
    gpuIn.convertTo(gpuOut, CV_32FC1);
    gpuOut.copyTo(*output);
    Benchmark::close("(OpenCL) Addblob: binary_convert");
  }
  else
  {
    Benchmark::open("(OpenCL) Addblob: binary_convert");
    binary.convertTo(*output, CV_32FC1);
    Benchmark::close("(OpenCL) Addblob: binary_convert");
  }

  // starts at fgValue+1 because fgValue is used already
  int label_count = fgValue + 1;

  Benchmark::open("AddblobRects: loops");
  for (int y = 0; y < output->rows; y++)
  {
    for (int x = 0; x < output->cols; x++)
    {
      float checker = output->at<float>(y, x);
      if (checker != fgValue)
      {
        continue;
      }

      cv::Rect rect;
      Benchmark::open("Addblob: Floodfill");
      cv::floodFill(*output, cv::Point(x, y), label_count, &rect, 0, 0, 8);
      Benchmark::close("Addblob: Floodfill");

      rects.push_back(rect);
      label_count++;
    }
  }
  Benchmark::close("AddblobRects: loops");
  if (toBeReleased)
  {
    delete output;
  }
}

void colorBlobs(Mat& output, vector<vector<Point2i>>& blobs, bool randomColor, Scalar color)
{
  // If no color specified, use random color
  unsigned char r(255), g(255), b(255);
  if (!randomColor)
  {
    r = color[0];
    g = color[1];
    b = color[2];
  }
  for (size_t i = 0; i < blobs.size(); i++)
  {
    if (randomColor)
    {
      r = 255 * (rand() / (1.0 + RAND_MAX));
      g = 255 * (rand() / (1.0 + RAND_MAX));
      b = 255 * (rand() / (1.0 + RAND_MAX));
    }

    for (size_t j = 0; j < blobs[i].size(); j++)
    {
      int x = blobs[i][j].x;
      int y = blobs[i][j].y;

      output.at<cv::Vec3b>(y, x)[0] = b;
      output.at<cv::Vec3b>(y, x)[1] = g;
      output.at<cv::Vec3b>(y, x)[2] = r;
    }
  }
}

void FindBlobsGraph(const cv::Mat& binary, const cv::Mat& whiteImg, std::vector<std::pair<int, cv::Rect>>& blobRect,
                    const std::vector<std::pair<int, cv::Rect>>& whiteRect, cv::Mat& adjMat, int maxDist)
{
  /*
    Compute the connectivity between the "binary" blobs and the "whiteImg" blobs
    Returns a matrix of size (nb whiteImg labels+2)x(nb binary labels+2). It is
    +2 because labels begins at 2...
    binary and whiteImg should be the same size!!

    binary: thresholded binary image
    whiteImg: thresholded binary image
    blobRect: vector of pairs of labels and rectanges (will be filled with
    "binary's" labels)
    whiteRect: vector of pairs of labels and rectanges of the "whiteImg" mat
    adjMat: Matrix of connectivity, will be filled
    maxDist: optionnal, maximum pixel distance for connectivity computation
  */

  // Fill the label_image with the blobs
  // 0  - background
  // 1  - unlabelled foreground
  // 2+ - labelled foreground

  if (binary.size() != whiteImg.size())
    throw std::logic_error("FindBlobsGraph: binary and whiteImg not the same size");

  if (whiteImg.type() != CV_32SC1)
    whiteImg.convertTo(whiteImg, CV_32SC1);

  cv::Mat label_image;
  binary.convertTo(label_image, CV_32SC1);
  // label_image=binary;
  int label_count = 2;  // starts at 2 because 0,1 are used already

  // TODO keep the rectangles
  // First pass, label pixels
  for (int y = 0; y < label_image.rows; y++)
  {
    int* row = (int*)label_image.ptr(y);
    for (int x = 0; x < label_image.cols; x++)
    {
      // std::cout<<"pix: "<<row[x]<<std::endl;
      if (row[x] != 1)
      {
        continue;
      }
      cv::Rect rect;
      cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

      blobRect.push_back(std::pair<int, cv::Rect>(label_count, rect));
      label_count++;
    }
  }

  // whiteRect.size() should be the number of different white blobs
  adjMat = cv::Mat(label_count, whiteRect.size() + 2, CV_8U, cv::Scalar::all(0));

  // Second pass, create the graph
  for (int y = 0; y < label_image.rows; y++)
  {
    int* row = (int*)label_image.ptr(y);
    // uchar *whiterow = (uchar*)whiteImg.ptr(y);
    int* whiterow = (int*)whiteImg.ptr(y);

    for (int x = 0; x < label_image.cols; x++)
    {
      if (whiterow[x] == 0)
      {  // Do we have white here?
        continue;
      }

      // Else, we look at the neighbors
      // Only 4 connectivity... should be enough

      // WEST
      if (x > 0)
      {
        if (row[x - 1] > 1)  // labeled
        {
          // DEBUG
          // std::cout<<"DEBUG WEST: "<<x<<" "<<y<<" "<<(x-1)<<" "<<row[x-1]<<"
          // "<<(int)whiterow[x]<<std::endl;
          // adjMat.at<uchar>(whiterow[x],row[x-1])=1;
          adjMat.at<uchar>(row[x - 1], whiterow[x]) = 1;  // symmetric, probably useless
        }
        else
        {
          if (maxDist > 1)  // we look a little farther
          {
            int dist = 2;
            bool done = false;
            while ((x - dist) > 0 && dist <= maxDist && !done)
            {
              if (row[x - dist] > 1)  // labeled
              {
                // adjMat.at<uchar>(whiterow[x],row[x-dist])=1;
                adjMat.at<uchar>(row[x - dist], whiterow[x]) = 1;  // symmetric, probably useless
                done = true;
              }
              dist++;
            }
          }
        }
      }

      // NORTH
      if (y > 0)
      {
        int* nrow = (int*)label_image.ptr(y - 1);
        if (nrow[x] > 1)  // labeled
        {
          // DEBUG
          // std::cout<<"DEBUG NORTH: "<<x<<" "<<y<<" "<<(x)<<" "<<nrow[x]<<"
          // "<<(int)whiterow[x]<<std::endl;
          // adjMat.at<uchar>(whiterow[x],nrow[x])=1;
          adjMat.at<uchar>(nrow[x], whiterow[x]) = 1;  // symmetric, probably useless
        }
        else
        {
          if (maxDist > 1)  // we look a little farther
          {
            int dist = 2;
            bool done = false;
            while ((y - dist) > 0 && dist <= maxDist && !done)
            {
              nrow = (int*)label_image.ptr(y - dist);
              if (nrow[x] > 1)  // labeled
              {
                // adjMat.at<uchar>(whiterow[x],nrow[x])=1;
                adjMat.at<uchar>(nrow[x], whiterow[x]) = 1;  // symmetric, probably useless
                done = true;
              }
              dist++;
            }
          }
        }
      }

      /////////
      // EAST
      if (x < (label_image.cols - 1))
      {
        if (row[x + 1] > 1)  // labeled
        {
          // DEBUG
          // std::cout<<"DEBUG EST: "<<x<<" "<<y<<" "<<(x+1)<<" "<<row[x+1]<<"
          // "<<(int)whiterow[x]<<std::endl;
          // adjMat.at<uchar>(0,row[x+1])=whiterow[x];
          adjMat.at<uchar>(row[x + 1], whiterow[x]) = 1;  // symmetric, probably useless
        }
        else
        {
          if (maxDist > 1)  // we look a little farther
          {
            int dist = 2;
            bool done = false;
            while ((x + dist) < (label_image.cols - 1) && dist <= maxDist && !done)
            {
              if (row[x + dist] > 1)  // labeled
              {
                // adjMat.at<uchar>(0,row[x+dist])=whiterow[x];
                adjMat.at<uchar>(row[x + dist], whiterow[x]) = 1;  // symmetric, probably useless
                done = true;
              }
              dist++;
            }
          }
        }
      }

      // SOUTH
      if (y < (label_image.rows - 1))
      {
        int* nrow = (int*)label_image.ptr(y + 1);
        if (nrow[x] > 1)  // labeled
        {
          // DEBUG
          // std::cout<<"DEBUG SOUTH: "<<x<<" "<<y<<" "<<(x)<<" "<<nrow[x+1]<<"
          // "<<(int)whiterow[x]<<std::endl;
          // adjMat.at<uchar>(whiterow[x],nrow[x])=1;
          adjMat.at<uchar>(nrow[x], whiterow[x]) = 1;  // symmetric, probably useless
        }
        else
        {
          if (maxDist > 1)  // we look a little farther
          {
            int dist = 2;
            bool done = false;
            while ((y + dist) < (label_image.rows - 1) && dist <= maxDist && !done)
            {
              nrow = (int*)label_image.ptr(y + dist);
              if (nrow[x] > 1)  // labeled
              {
                // adjMat.at<uchar>(whiterow[x],nrow[x])=1;
                adjMat.at<uchar>(nrow[x], whiterow[x]) = 1;  // symmetric, probably useless
                done = true;
              }
              dist++;
            }
          }
        }
      }
    }
  }

  // std::cout<<"DEBUG MAT:"<<std::endl;
  // std::cout<<adjMat<<std::endl;
}

void FindBlobsLabels(const cv::Mat& binary, cv::Mat& label_image, std::vector<std::pair<int, cv::Rect>>& blobs)
{
  /*
    input a binary image and labels it into label_image with the label and the
    rectangle into blobs
  */

  // Fill the label_image with the blobs
  // 0  - background
  // 1  - unlabelled foreground
  // 2+ - labelled foreground

  // cv::Mat label_image;
  binary.convertTo(label_image, CV_32SC1);
  // label_image=binary;
  int label_count = 2;  // starts at 2 because 0,1 are used already

  for (int y = 0; y < label_image.rows; y++)
  {
    int* row = (int*)label_image.ptr(y);
    for (int x = 0; x < label_image.cols; x++)
    {
      // std::cout<<"pix: "<<row[x]<<std::endl;
      if (row[x] != 1)
      {
        continue;
      }

      cv::Rect rect;
      cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

      blobs.push_back(std::pair<int, cv::Rect>(label_count, rect));
      label_count++;
    }
  }
}

// Oh
void paintBlob(cv::Mat& blobPainted, std::vector<cv::Point2i> blob, unsigned char value)
{
  for (auto point : blob)
  {
    blobPainted.at<uchar>(point.y, point.x) = value;
  }
}
