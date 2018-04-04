#pragma once

#include "Filters/Filter.hpp"
#include "Filters/Custom/FieldBorderData.hpp"

namespace Vision {
namespace Filters {

/**
 * Recognize Border of the Field and compute the clipping
 */
class FieldBorder : public Filter {
  int col_nb, row_nb; // the size of the image
  double gd_ratio;    // ratio entre les inputs (TMP)
  int comb_dx;        // spacing between teeth of the comb
  int corr_comb_size; // comb size after scaling factor
  int l_line, r_line;
  int best_K;
  
public:
  FieldBorder();
  ~FieldBorder();

  virtual std::string getClassName() const override { return "FieldBorder"; }
  virtual int expectedDependencies() const override { return 4; }

  FieldBorderData loc_data;
  void update_loc_data(std::vector<cv::Vec2f> line_pair_eq,
                       std::vector<std::pair<int, int> > line_pair_domain,
          	       std::vector<double> line_quality,
		       double X,
		       bool final_compute);
  void compute_marks(cv::Mat & green_density,
                     std::vector<cv::Point> & marks,
                     std::vector<double> & threshold);
  void line_detection(std::vector<cv::Point> & marks,
                      std::vector<cv::Vec2f> & brut_lines,
                      std::vector<cv::Vec2f> & line_eq,
                      std::vector<std::pair<int, int> > & line_x_domain,
                      /* paramètres de Hough */
                      int hough_threshold,
                      int pix_precision,
                      int angle_precision);
  
  void filter_bad_marks(std::vector<cv::Point> & marks,
                        std::vector<cv::Point> & bad_marks,
                        std::vector<cv::Vec2f> & line_eq,
                        int pix_precision);

  void compute_precision_scores(std::vector<cv::Point> & marks,
                                std::vector<cv::Vec2f> & line_eq,
                                std::vector<std::pair<int, int> > & line_x_domain);

  void compute_geometric_score(std::map< std::pair<int,int>, std::pair<double,double> >
                                 & line_pairs_geom_scores,
                               std::map< std::pair<int,int>, int > & line_pairs_X_inter,
                               std::vector<cv::Vec2f> & line_eq,
                               std::vector<std::pair<int, int> > & line_x_domain);

  void compute_separation_score(cv::Mat & green_density,
                                std::vector<double> & threshold,
                                std::vector<cv::Vec2f> & line_eq,
                                std::vector<std::pair<int, int> > & line_x_domain);

  void compute_border(std::vector<cv::Point> & marks,
                      std::vector<cv::Vec2f> & line_eq,
                      std::vector<std::pair<int, int> > & line_x_domain,
                      std::map< std::pair<int,int>, int > & line_pairs_X_inter,
                      std::map< std::pair<int,int>, std::pair<double,double> >
                        & line_pairs_geom_scores);

  void debug_tag(cv::Mat * green,
                 cv::Mat * green_density,
                 std::vector<cv::Point> & marks,
                 std::vector<cv::Vec2f> & brut_lines,
                 std::vector<cv::Point> & bad_marks,
                 std::vector<cv::Vec2f> & line_eq);

  void init_params(cv::Mat & green, cv::Mat & green_density);
  void epilogue(std::vector<cv::Vec2f> & line_eq,
                std::vector<std::pair<int, int> > & line_x_domain,
                std::map< std::pair<int,int>, int > & line_pairs_X_inter);
    
public:
  static FieldBorderData * current_loc_data;
  static double best_score;
  static double get_score();
  
protected:
  /**
   * @Inherit
   */
  virtual void process() override;
  virtual void setParameters() override;

private:
  /* enable (1) or disable (0) the filter */
  ParamInt enable;
  /* output debug image tag level */
  ParamInt tag_level;
  /* the size of the side of the (squared) kernel
     used to compute the density */
  ParamInt density_kernel_size;
  /* the number of teeth of the comb */
  ParamInt comb_size;
  /* the maximum score to admit the line as observation */
  ParamFloat max_obs_score;
  /* scale factor of input image */
  ParamInt scale_factor;
  /* 0 or 1 : activate the loc */
  ParamInt loc_active;
  /* 0 or 1 : activate debug info */
  ParamInt debug_output;
  /* 0.0 or 3.0 :  */
  ParamFloat max_dist_corner;
  /* 0.0 or 360.0 :  */
  ParamFloat tolerance_angle_corner;
  /* 0.0 or 360.0 :  */
  ParamFloat tolerance_angle_line;
  /* Minimal segment length  */
  ParamFloat minimal_segment_length;
  /* Minimal segment length  */
  ParamInt with_black;

  // calcul des potentiels
  ParamFloat potential_pos50, potential_angle50, potential_exp, potential_error;
  
private:
  static float * line_fine_score;
  static float * line_fine_score_square;
  static float * line_fine_score_cum;
  static float * line_fine_score_square_cum;
  static float * line_sep_cum_score;
  static int line_nb;

  static bool allocate_scores(int line_nb, int comb_size);
  static void clear_score_mem();
  static double dist_point_line(cv::Vec2f line_eq, cv::Point2f M);
  static double dist_point_line(cv::Vec2f line_eq, cv::Point M);
};
}
}
