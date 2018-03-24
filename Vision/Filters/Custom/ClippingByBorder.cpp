#include "Filters/Custom/ClippingByBorder.hpp"
#include "Filters/Custom/FieldBorder.hpp"

namespace Vision {
namespace Filters {

ClippingByBorder::ClippingByBorder() : Filter("ClippingByBorder") {}

ClippingByBorder::~ClippingByBorder() {}
  
void ClippingByBorder::setParameters() {
  debug_output = ParamInt(0, 0, 1);
  params()->define<ParamInt>("debug_output", &debug_output);
  vshift_ratio = ParamFloat(0.05, 0, 1);
  params()->define<ParamFloat>("vshift_ratio", &vshift_ratio);
}

void ClippingByBorder::process() {
  
  std::string fieldBorderName = _dependencies[0]; 
  // cv::Mat fieldBorder = (getDependency(fieldBorderName).getImg())->clone();
  // int down_scale = 4;
  int row_nb = (getDependency(fieldBorderName).getImg())->rows / 4;
  int col_nb = (getDependency(fieldBorderName).getImg())->cols / 4;
  img() = cv::Mat(row_nb, col_nb, 0);
  img().setTo(0);

  int vshift = (int) (vshift_ratio * row_nb);
  
  if (FieldBorder::current_loc_data == NULL) {
    if (debug_output) printf("ClippingByBorder : I have no data\n");
    img().setTo(255);
    return;
  }

  if (FieldBorder::get_score() > FieldBorder::current_loc_data->max_obs_score) {
    if (debug_output) printf("ClippingByBorder : bad score\n");
    img().setTo(255);
    return;
  }
   
  std::vector<std::pair<cv::Point2f, cv::Point2f > > pix_lines =
    FieldBorder::current_loc_data->getPixLines();
  std::vector<std::pair<double, double> > line_eq;
  for (int i=0; i<(int)pix_lines.size(); i++) {
    cv::Point2f A = pix_lines[i].first;
    cv::Point2f B = pix_lines[i].second;
    double ABx = B.x - A.x;
    if (ABx == 0) continue;
    double a = (B.y - A.y) / ABx;
    double b = A.y - a * A.x;
    line_eq.push_back(std::pair<double,double>(a,b));
  }

  for (int x=0; x<col_nb; x++) {
    int y_max = 0;
    for (int l=0; l<(int) line_eq.size(); l++) {
      int y = (int) (row_nb * (line_eq[l].first * (((double) x)/col_nb) + line_eq[l].second));
      if (y_max < y) y_max = y;
    }
    for (int y=max(0,y_max-vshift); y<row_nb; y++)
      img().at<uchar>(y,x) = 255;
  }
}

}
}
