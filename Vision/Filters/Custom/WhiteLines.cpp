#include "Filters/Custom/WhiteLines.hpp"
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace Vision {
namespace Filters {

WhiteLines::WhiteLines() : Filter("WhiteLines") {}

WhiteLines::~WhiteLines() {}
  
void WhiteLines::setParameters() {
}

void WhiteLines::process() {
  std::string sourceName = _dependencies[0]; 
  cv::Mat source = (getDependency(sourceName).getImg())->clone();
  std::string clippingName = _dependencies[1]; 
  cv::Mat clipping = (getDependency(clippingName).getImg())->clone();
  std::string whitelinecolorName = _dependencies[2]; 
  cv::Mat whitelinecolor = (getDependency(whitelinecolorName).getImg())->clone();
  int row_nb = source.rows;
  int col_nb = source.cols;
  img() = cv::Mat(row_nb, col_nb, 0);
  img().setTo(0);

  int clipping_scale = source.rows / clipping.rows;
  int clipping_security = row_nb / 20;

  // TODO: faire un vrai masque
  for (int x=0; x<col_nb; x++) {
    for (int y=0; y<row_nb; y++) {        
      if (clipping.at<uchar>(y/clipping_scale, x/clipping_scale) > 0 &&
          clipping.at<uchar>(y/clipping_scale - clipping_security/clipping_scale, x/clipping_scale) > 0 &&
          whitelinecolor.at<uchar>(y,x) > 0) {
        img().at<uchar>(y,x) = 255;
      }
    }
  }


  // cv::blur( img(), img(), cv::Size(5,5) );
  /*
  cv::Canny( img(), img(),
             100, // lowThreshold
             200, // highThreshold 
             3 // kernel_size
             );
  */

  // int shape = 2;
  // cv::Mat kernel = getStructuringElement(shape, cv::Size(5,5));
  // erode(img(), img(), kernel);
  // dilate(img(), img(), kernel);
  
  return;
  
  vector<cv::Vec2f> brut_lines;
  // Le nombre de point à considérer au minimum dans une ligne
  int hough_threshold = 50; // un 1/5 de la largeur d'image
  // Paramétrage de la discrétisation de Hough
  int pix_precision = 1; // en pixel
  double angle_precision = 1; // en degré
  do {
    hough_threshold += 10;
    cv::HoughLines(img(), brut_lines, pix_precision,
                   angle_precision*CV_PI/180, hough_threshold, 0, 0);
  } while (brut_lines.size() > 20);
  printf("%d brut lines\n", (int)brut_lines.size());
  
  bool draw_brut_line = true;
  if (draw_brut_line) {
    for( size_t i = 0; i < brut_lines.size(); i++ ) {
      float rho = brut_lines[i][0], theta = brut_lines[i][1];
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      cv::Point pt1, pt2;
      pt1.x = cvRound(x0 + 1000*(-b)); pt1.y = cvRound(y0 + 1000*a);
      pt2.x = cvRound(x0 - 1000*(-b)); pt2.y = cvRound(y0 - 1000*a);
      line( img(), pt1, pt2, cv::Scalar::all(255), 1, CV_AA);
    }
  }

  
}



}
}
