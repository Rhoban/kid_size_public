#include "Filters/Custom/BallRadiusProvider.hpp"

#include "CameraState/CameraState.hpp"

namespace Vision
{
namespace Filters
{
std::string BallRadiusProvider::getClassName() const
{
  return "BallRadiusProvider";
}

int BallRadiusProvider::expectedDependencies() const
{
  return 1;
}

void BallRadiusProvider::setParameters()
{
  nbCols = ParamInt(4, 2, 200);
  nbRows = ParamInt(4, 2, 200);
  params()->define<ParamInt>("nbCols", &nbCols);
  params()->define<ParamInt>("nbRows", &nbRows);
}

void BallRadiusProvider::process()
{
  cv::Size size = getDependency().getImg()->size();
  // 1: Compute key columns and key rows
  std::vector<int> key_cols, key_rows;
  // 1.a: always select extreme pixels
  key_cols.push_back(0);
  key_rows.push_back(0);
  // 1.b: add intermediary values
  double step_x = size.width / (double)nbCols;
  double step_y = size.height / (double)nbRows;
  for (int col = 1; col < nbCols - 1; col++)
  {
    key_cols.push_back((int)(col * step_x));
  }
  for (int row = 1; row < nbRows - 1; row++)
  {
    key_rows.push_back((int)(row * step_y));
  }
  // 1.c: always use last pixel
  key_cols.push_back(size.width - 1);
  key_rows.push_back(size.height - 1);

  // 2: create image
  cv::Mat tmp_img(size, CV_32FC1);

  // 3: Place values at key points
  for (int col : key_cols)
  {
    for (int row : key_rows)
    {
      double ballRadius = getCS().computeBallRadiusFromPixel(cv::Point2f(col, row));
      // For points above horizon, set ballRadius to 0
      if (ballRadius < 0)
        ballRadius = 0;
      tmp_img.at<float>(row, col) = ballRadius;
    }
  }

  // 4: Interpolate on key columns
  // Note: forbidding interpolation when there are 0 values because their meaning is also that we failed to compute ball
  // radius, therefore it might provide degenerate result
  for (int col : key_cols)
  {
    for (int row_idx = 0; row_idx < nbRows - 1; row_idx++)
    {
      int start_row = key_rows[row_idx];
      int end_row = key_rows[row_idx + 1];
      double start_val = tmp_img.at<float>(start_row, col);
      double end_val = tmp_img.at<float>(end_row, col);
      int dist = end_row - start_row;
      double diff = end_val - start_val;
      double slope = diff / dist;
      bool interpolation_allowed = start_val > 0 && end_val > 0;
      for (int row = start_row + 1; row < end_row; row++)
      {
        double val = 0;
        if (interpolation_allowed)
        {
          val = start_val + slope * (row - start_row);
        }
        tmp_img.at<float>(row, col) = val;
      }
    }
  }

  // 5: Interpolate between key rows
  // Note: forbidding interpolation when there are 0 values because their meaning is also that we failed to compute ball
  // radius, therefore it might provide degenerate result
  for (int row = 0; row < size.height; row++)
  {
    for (int col_idx = 0; col_idx < nbCols - 1; col_idx++)
    {
      int start_col = key_cols[col_idx];
      int end_col = key_cols[col_idx + 1];
      int dist = end_col - start_col;
      double start_val = tmp_img.at<float>(row, start_col);
      double end_val = tmp_img.at<float>(row, end_col);
      double diff = end_val - start_val;
      double slope = diff / dist;
      bool interpolation_allowed = start_val > 0 && end_val > 0;
      for (int col = start_col + 1; col < end_col; col++)
      {
        double val = 0;
        if (interpolation_allowed)
        {
          val = start_val + slope * (col - start_col);
        }
        tmp_img.at<float>(row, col) = val;
      }
    }
  }

  // 6: Affect img
  img() = tmp_img;
}
}  // namespace Filters
}  // namespace Vision
