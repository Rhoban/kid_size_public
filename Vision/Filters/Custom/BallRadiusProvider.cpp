#include "Filters/Custom/BallRadiusProvider.hpp"

#include "CameraState/CameraState.hpp"

namespace Vision {
namespace Filters {

std::string BallRadiusProvider::getClassName() const {
  return "BallRadiusProvider";
}

int BallRadiusProvider::expectedDependencies() const {
  return 1;
}

void BallRadiusProvider::setParameters() {
  nbCols = ParamInt(4, 2, 200);
  nbRows = ParamInt(4, 2, 200);
  params()->define<ParamInt>("nbCols", &nbCols);
  params()->define<ParamInt>("nbRows", &nbRows);
}

void BallRadiusProvider::process() {
  cv::Size size = getDependency().getImg()->size();
  // 1: Compute key columns and key rows
  std::vector<int> key_cols, key_rows;
  // 1.a: always select extreme pixels
  key_cols.push_back(0);
  key_rows.push_back(0);
  // 1.b: add intermediary values
  double step_x = size.width/(double)nbCols;
  double step_y = size.height/(double)nbRows;
  for (int col = 1; col < nbCols-1; col++) {
    key_cols.push_back((int)(col * step_x));
  }
  for (int row = 1; row < nbRows-1; row++) {
    key_rows.push_back((int)(row * step_y));
  }
  // 1.c: always use last pixel
  key_cols.push_back(size.width-1);
  key_rows.push_back(size.height-1);

  // 2: create image
  cv::Mat tmp_img(size, CV_32FC1);

  // 3: Place values at key points
  for (int col : key_cols) {
    for (int row : key_rows) {
      ///TODO: replace the current method to get something more appropriate
      float factor = 100;//Since current method returns a int for the expected radius, use a factor
      int radiusMin, radiusMax;
      getCS().ballInfoFromPixel(cv::Point2f(factor*col, factor*row),
                                factor * size.width, factor*size.height,
                                &radiusMin, &radiusMax);
      float radius = (radiusMin + radiusMax) / 2 / factor;
      tmp_img.at<float>(row, col) = radius;
    }
  }

  // 4: Interpolate on key columns
  for (int col : key_cols) {
    for (int row_idx = 0; row_idx < nbRows -1; row_idx++) {
      int start_row = key_rows[row_idx];
      int end_row = key_rows[row_idx+1];
      double start_val = tmp_img.at<float>(start_row, col);
      double end_val   = tmp_img.at<float>(end_row  , col);
      int dist = end_row - start_row;
      double diff = end_val - start_val;
      double slope = diff / dist;
      for (int row = start_row+1; row < end_row; row++) {
        double val = start_val + slope * (row - start_row);
        tmp_img.at<float>(row, col) = val;
      }
    }
  }

  // 5: Interpolate between key rows
  for (int row = 0; row < size.height; row++) {
    for (int col_idx = 0; col_idx < nbCols -1; col_idx++) {
      int start_col = key_cols[col_idx];
      int end_col = key_cols[col_idx+1];
      int dist = end_col - start_col;
      double start_val = tmp_img.at<float>(row, start_col);
      double end_val   = tmp_img.at<float>(row, end_col  );
      double diff = end_val - start_val;
      double slope = diff / dist;
      for (int col = start_col+1; col < end_col; col++) {
        double val = start_val + slope * (col - start_col);
        tmp_img.at<float>(row, col) = val;
      }
    }
  }

  // 6: Affect img
  img() = tmp_img;
}
}
}
