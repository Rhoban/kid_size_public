#include "Filters/Custom/FieldBorder.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "Filters/Custom/FieldBorderData.hpp"
#include "CameraState/CameraState.hpp"
#include "rhoban_utils/timing/benchmark.h"
#include "Localisation/Field/ArenaCornerObservation.hpp"
#include <algorithm>


#define FBPRINT_DEBUG(str...) if (debug_output) { printf("[CLIPPING] "); printf(str); }

using ::rhoban_utils::Benchmark;
using namespace rhoban_utils;

namespace Vision {
namespace Filters {

#define lfscore(l,k) line_fine_score[l*corr_comb_size+k]
#define lfscore_sq(l,k) line_fine_score_square[l*corr_comb_size+k]
#define lfscore_cum(l,k) line_fine_score_cum[l*corr_comb_size+k]
#define lfscore_sq_cum(l,k) line_fine_score_square_cum[l*corr_comb_size+k]
#define lsscore_cum(l,k) line_sep_cum_score[l*corr_comb_size+k]
  
FieldBorder::FieldBorder() :
  Filter("FieldBorder"), col_nb(1), row_nb(1), gd_ratio(1), comb_dx(1), corr_comb_size(1),
  l_line(-1), r_line(-1), best_K(-1)
{}

void FieldBorder::setParameters() {
  enable = ParamInt(1, 0, 1);
  params()->define<ParamInt>("enable", &enable);
  tag_level = ParamInt(1, 0, 5);
  params()->define<ParamInt>("tag_level", &tag_level);
  density_kernel_size = ParamInt(30, 2, 100);
  params()->define<ParamInt>("density_kernel_size", &density_kernel_size);
  comb_size = ParamInt(100, 10, 640);
  params()->define<ParamInt>("comb_size", &comb_size);
  max_obs_score = ParamFloat(20.0, 0.0, 100.0);
  params()->define<ParamFloat>("max_obs_score", &max_obs_score);
  loc_data.max_obs_score = max_obs_score;
  scale_factor = ParamInt(4, 1, 16);
  params()->define<ParamInt>("scale_factor", &scale_factor);
  loc_active = ParamInt(1, 0, 1);
  params()->define<ParamInt>("loc_active", &loc_active);
  debug_output = ParamInt(1, 0, 1);
  params()->define<ParamInt>("debug_output", &debug_output);
  max_dist_corner = ParamFloat(10.0, 0.0, 30.0);
  params()->define<ParamFloat>("max_dist_corner", &max_dist_corner);
  tolerance_angle_corner = ParamFloat(15.0, 0.0, 360.0);
  params()->define<ParamFloat>("tolerance_angle_corner", &tolerance_angle_corner);
  tolerance_angle_line = ParamFloat(10.0, 0.0, 360.0);
  params()->define<ParamFloat>("tolerance_angle_line", &tolerance_angle_line);
  minimal_segment_length = ParamFloat(0.30, 0.0, 10.0);
  params()->define<ParamFloat>("minimal_segment_length", &minimal_segment_length);
  with_black = ParamInt(1, 0, 1);
  params()->define<ParamInt>("with_black", &with_black);


  potential_pos50 = ParamFloat(25.0, 0.0, 500.0);
  params()->define<ParamFloat>("potential_pos50", &potential_pos50);
  potential_angle50 = ParamFloat(30.0, 0.0, 500.0);
  params()->define<ParamFloat>("potential_angle50", &potential_angle50);
  potential_exp = ParamFloat(1.0, 0.0, 500.0);
  params()->define<ParamFloat>("potential_exp", &potential_exp);
  potential_error = ParamFloat(0.3, 0.0, 500.0);
  params()->define<ParamFloat>("potential_error", &potential_error);
}

void FieldBorder::process() {
  if (!enable) {
    loc_data.reset();
    return;
  }
  
  std::string greenName = _dependencies[0];
  std::string greenDensityName = _dependencies[1];
  std::string sourceName = _dependencies[2];
  std::string greenDensityWithoutBlackName = _dependencies[3];
  
  // le canal vert selectionné par la couleur
  cv::Mat green = (getDependency(greenName).getImg())->clone();
  // smooth à base d'image intégrale
  cv::Mat green_density;
  if( with_black == 1 ){
	green_density = (getDependency(greenDensityWithoutBlackName).getImg())->clone();
  }else{
	green_density = (getDependency(greenDensityName).getImg())->clone();
  }
  // image source (juste pour le débug)
  if (tag_level > 0) {
    cv::Mat source = (getDependency(sourceName).getImg())->clone();
    img() = source.clone();
  }

  init_params(green, green_density);
  
  // Les marques détectent la transition du haut vers le
  // le vert le long des dents du peigne (comb_size est le nombre de dents du peigne)
  // Note: une dent peut contenir plusieurs marques a priori (ca peut-être des tâches de
  // vert dans le décors, etc. On les filtre plus tard (on stocke donc une liste de point
  // brute, et non pas un tableau indicé par les dents).
  std::vector<cv::Point> marks;
  // pour chaque dent du peigne, on évalue le seuil de passage entre non-vert et vert
  // l'évaluation se fait en regardant la colonne et en regardant si on peut séparer
  // les valeurs de densité en 2 paquets.
  std::vector<double> threshold;
  for (int x = 0; x < col_nb; x+= comb_dx) threshold.push_back(0);

  Benchmark::open("calcul des marques");
  compute_marks(green_density, marks, threshold);
  Benchmark::close("calcul des marques");

  vector<cv::Vec2f> brut_lines; // sortie de l'algorithme de Hough
  // On calcule les équations des droites sous la forme y=ax+b
  // (les droites verticales sont bannies pour le moment...)
  std::vector<cv::Vec2f> line_eq;
  // [min_x, max_x] for each line to stay in the image 
  std::vector<std::pair<int, int> > line_x_domain;
  // Le nombre de point à considérer au minimum dans une ligne
  int hough_threshold = corr_comb_size / 5; // un 1/5 de la largeur d'image
  // Paramétrage de la discrétisation de Hough
  int pix_precision = row_nb / 40; // en pixel
  int angle_precision = 1; // en degré
  Benchmark::open("Hough");
  line_detection(marks, brut_lines, line_eq, line_x_domain,
                 hough_threshold, pix_precision, angle_precision);
  Benchmark::close("Hough");
  
  // Elimination des marques inconsistantes, i.e. qui n'appartiennent
  // sur aucune ligne de Hough. C'est mieux pour calculer les scores de précision
  std::vector<cv::Point> bad_marks;
  filter_bad_marks(marks, bad_marks, line_eq, pix_precision);

  // Matériel pour calculer les moyennes et écart-types globaux et locaux de score
  // TODO: à documenter
  compute_precision_scores(marks, line_eq, line_x_domain);

  // Pre-calcul des scores géométriques
  std::map< std::pair<int,int>, std::pair<double,double> >
    line_pairs_geom_scores; // (idx,idx) -> (score, confidence)
  std::map< std::pair<int,int>, int > line_pairs_X_inter;
  Benchmark::open("geom scores");
  // TODO: à documenter
  compute_geometric_score(line_pairs_geom_scores, line_pairs_X_inter, line_eq, line_x_domain);
  Benchmark::close("geom scores");
    
  Benchmark::open("separation scores");
  compute_separation_score(green_density, threshold, line_eq, line_x_domain);
  Benchmark::close("separation scores");
      
  // On optimise les couples de demi-droite sur la base d'un score calculé à partir
  // - l'écart type à la plus proche marque
  // - le caractère séparant de la demi-droite: est-ce qu'elle sépare le vert du non vert ?
  // - la géométrie, i.e. l'angle corrigé entre les deux droites.

  Benchmark::open("optimisation des lignes");
  compute_border(marks, line_eq, line_x_domain, line_pairs_X_inter, line_pairs_geom_scores);
  Benchmark::close("optimisation des lignes");

  // this is the end ...
  epilogue(line_eq, line_x_domain, line_pairs_X_inter);
  debug_tag(&green, &green_density, marks, brut_lines, bad_marks, line_eq);
}

void FieldBorder::init_params(cv::Mat & green, cv::Mat & green_density) {
  current_loc_data = &loc_data;
  loc_data.max_obs_score = max_obs_score;
  loc_data.loc_active = loc_active;
  loc_data.debug_output = debug_output;
  loc_data.max_dist_corner = max_dist_corner;
  loc_data.tolerance_angle_line = tolerance_angle_line;
  loc_data.tolerance_angle_corner = tolerance_angle_corner;
  Vision::Localisation::ArenaCornerObservation::potential_pos50 = potential_pos50;
  Vision::Localisation::ArenaCornerObservation::potential_angle50 = potential_angle50;
  Vision::Localisation::ArenaCornerObservation::potential_exp = potential_exp;
  Vision::Localisation::ArenaCornerObservation::pError = potential_error;
  corr_comb_size = comb_size / scale_factor;
  col_nb = green.cols / scale_factor;
  row_nb = green.rows / scale_factor;
  if (col_nb == 0 || row_nb == 0 || corr_comb_size == 0) return;
  gd_ratio = ((double) green_density.rows) / row_nb;
  comb_dx = col_nb / corr_comb_size; // espacement entre les dents du peigne  
}
  
void FieldBorder::epilogue(std::vector<cv::Vec2f> & line_eq,
                           std::vector<std::pair<int, int> > & line_x_domain,
                           std::map< std::pair<int,int>, int > & line_pairs_X_inter) {
  loc_data.reset();
  if (best_score >= 0) {
    std::vector<cv::Vec2f> line_pair_eq = { line_eq[l_line] } ;
    std::vector<std::pair<int, int> > line_pair_domain = { line_x_domain[l_line] };
    if (r_line >= 0) {
      line_pair_eq.push_back(line_eq[r_line]);
      line_pair_domain.push_back(line_x_domain[r_line]);
    } else { FBPRINT_DEBUG("Only one line !\n"); }
    std::vector<double> line_quality =
      { best_score, best_score }; // TODO: distinguer les scores
    // On prend le point d'intersection s'il y en a un
    int X = (int) (best_K * comb_dx);
    if (r_line >= 0) {
      try {
        X = line_pairs_X_inter.at(std::pair<int,int>(l_line, r_line));
      } catch ( const std::exception & e ) {}
    } else {
      X = line_pair_domain[0].second;
    }
    update_loc_data(line_pair_eq,
                    line_pair_domain,
                    line_quality,
                    X,
                    true);
  }  
}

  
void FieldBorder::debug_tag(cv::Mat * green,
                            cv::Mat * green_density,
                            std::vector<cv::Point> & marks,
                            std::vector<cv::Vec2f> & brut_lines,
                            std::vector<cv::Point> & bad_marks,
                            std::vector<cv::Vec2f> & line_eq) {

  if (tag_level == 0) return;
  if (tag_level >= 3) {
    for( size_t i = 0; i < brut_lines.size(); i++ ) {
      double rho = brut_lines[i][0], theta = brut_lines[i][1];
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      cv::Point pt1, pt2;
      pt1.x = cvRound(x0 + 1000*(-b)); pt1.y = cvRound(y0 + 1000*a);
      pt2.x = cvRound(x0 - 1000*(-b)); pt2.y = cvRound(y0 - 1000*a);
      line( img(), scale_factor*pt1, scale_factor*pt2,
            cv::Scalar::all(255), 1, CV_AA);
    }
  }

  if (tag_level >= 2) {
    for (int k=0; k<(int)marks.size(); k++)
      cv::circle(img(), scale_factor * marks[k], 5, cv::Scalar::all(255));
    for (int k=0; k<(int)bad_marks.size(); k++)
      cv::circle(img(), scale_factor * bad_marks[k], 7, cv::Scalar(0,0,255));
  }

  if (tag_level >=4) {
    if (green != NULL) {
      for (int x=0; x<scale_factor*col_nb; x++) {
        for (int y=0; y<scale_factor*row_nb; y++) {
          if (green->at<uchar>(y,x) == 0) {
            for (int c=0; c<3; c++)
              img().at<cv::Vec3b>(y,x)[c] = 0;
          }
        }
      }
    }
  } 
  
  std::stringstream stream;
  stream << "score : " << best_score;
  std::string text = stream.str();
  int fontFace = cv::FONT_HERSHEY_PLAIN;
  double fontScale = 2;
  int thickness = 3;
  cv::Point textOrg(10, 20);
  cv::putText(img(), text, textOrg, fontFace,
	      fontScale, cv::Scalar::all(255), thickness,8);

  if (loc_data.is_obs_valid()) {
    FBPRINT_DEBUG("Observation is valid, ready to be used !\n");
  }
  else {
    FBPRINT_DEBUG("Observation not valid\n");
  }
}
  
void FieldBorder::update_loc_data(std::vector<cv::Vec2f> line_pair_eq,
                                  std::vector<std::pair<int, int> > line_pair_domain,
				  std::vector<double> line_quality,
				  double X,
				  bool final_compute) {

  Benchmark::open("update_loc_data");

  loc_data.reset();
  if (line_pair_eq.size()==0) return;
  std::vector<cv::Point2f> corner_approx;
  int X0 = line_pair_domain[0].first;
  int Y0 = line_pair_eq[0][0] * X0 + line_pair_eq[0][1];
  cv::Point U(X0, Y0);
  cv::Point V(X, line_pair_eq[0][0] * X + line_pair_eq[0][1]);
  if (final_compute && tag_level > 0)
    line( img(), scale_factor*U, scale_factor*V, cv::Scalar::all(255), 8, CV_AA);

  cv::Point2f Uf(((float) U.x) / col_nb, ((float) U.y) / row_nb);
  cv::Point2f Vf(((float) V.x) / col_nb, ((float) V.y) / row_nb);
  loc_data.pushPixLine(line_quality[0],
		       std::pair<cv::Point2f,cv::Point2f>(Uf,Vf));
  loc_data.line_scores.push_back(line_quality[0]); //TODO utiliser pour le potentiel?
  corner_approx.push_back(Vf);
  
  if (line_pair_eq.size() >= 2) {
    int X1 = line_pair_domain[1].second;
    int Y1 = line_pair_eq[1][0] * X1 + line_pair_eq[1][1];    
    U = cv::Point(X, line_pair_eq[1][0] * X + line_pair_eq[1][1]);
    V = cv::Point(X1, Y1);  
    if (final_compute && tag_level > 0)
      line( img(), scale_factor*U, scale_factor*V, cv::Scalar::all(255), 8, CV_AA);

    cv::Point2f Uf(((float) U.x) / col_nb, ((float) U.y) / row_nb);
    cv::Point2f Vf(((float) V.x) / col_nb, ((float) V.y) / row_nb);
    loc_data.pushPixLine(line_quality[1],
			 std::pair<cv::Point2f,cv::Point2f>(Uf, Vf));
    loc_data.line_scores.push_back(line_quality[1]); 
    corner_approx.push_back(Uf);
  }

  if (corner_approx.size() == 2 /* ICI: pas terrible ce test ... */) {
    cv::Point2f C((corner_approx[0].x + corner_approx[1].x) / 2,
		  (corner_approx[0].y + corner_approx[1].y) / 2);
    loc_data.addPixCorner(C);
  }

  loc_data.computeTransformations(&getCS(), final_compute);

  Benchmark::close("update_loc_data");
}

void
FieldBorder::compute_border(std::vector<cv::Point> & marks,
                            std::vector<cv::Vec2f> & line_eq,
                            std::vector<std::pair<int, int> > & line_x_domain,
                            std::map< std::pair<int,int>, int > & line_pairs_X_inter,
                            std::map< std::pair<int,int>, std::pair<double,double> >
                              & line_pairs_geom_scores) {
  
  l_line = -1;
  r_line = -1;
  best_K = -1;
  best_score = -1;
  float best_precision_score = 0,
    best_separation_score = 0, best_geom_score = 0, best_orphan_mark_score = 0; 
  float sc_mean=0,sc_sq_mean=0;
  for (int ll=0; ll<(int) line_eq.size(); ll++) {
    for (int rl=0; rl<(int) line_eq.size(); rl++) {
      int K = -1;
      try {
        int X_inter = line_pairs_X_inter.at(std::pair<int,int>(ll,rl));
        K = X_inter / comb_dx;
      } catch ( const std::exception & e ) {} 
      if (K < 0) {
        if (ll==rl) K = corr_comb_size-2; // cas d'une seule droite
        else continue; // cas dégénérés (e.g. intersection en dehors de l'image)
      }
      if (K <= 0) K = 1;
      if (K >= corr_comb_size-1) K = corr_comb_size-2;
      // on interdit les angles obtus
      if (line_eq[ll][0] > line_eq[rl][0]) continue;

      // restriction aux domaines des droite
      int min_x = line_x_domain[ll].first;
      int min_K = max(1, min_x / comb_dx); 
      int max_x = line_x_domain[rl].second;
      int max_K = min(corr_comb_size-2, max_x / comb_dx); 
      if (K < min_K) K = min_K;
      if (K > max_K) K = max_K;
      
      // écart type de la demi droite de gauche
      if ((K-(min_K-1)+1) != 0) { 
        sc_mean = (lfscore_cum(ll,K) - lfscore_cum(ll, min_K-1)) / (K-(min_K-1)+1);
        sc_sq_mean = (lfscore_sq_cum(ll,K) - lfscore_sq_cum(ll, min_K-1)) / (K-(min_K-1)+1);
      }
      float sq = sc_sq_mean - sc_mean*sc_mean;
      if (sq < 0) sq = 0;
      float lscore = sqrt(sq);

      // écart type de la demi droite de droite
      if ((max_K+1-K) != 0) {
        sc_mean = (lfscore_cum(rl,max_K+1)-lfscore_cum(rl,K)) / (max_K+1-K); 
        sc_sq_mean = (lfscore_sq_cum(rl,max_K+1)-lfscore_sq_cum(rl,K)) / (max_K+1-K);
      }
      sq = sc_sq_mean - sc_mean*sc_mean;
      if (sq < 0) sq = 0;
      float rscore = sqrt(sq);
      float precision_score =
        ((K-(min_K-1)) * lscore) + (max_K+1 - K) * rscore;
      if ((max_K-min_K+2) != 0)
        precision_score /= max_K-min_K+2;
      
      // Critère 2 pour le score : les lignes doivent séparer
      // l'image intégrale
      float separation_score =
          lsscore_cum(ll,K)
        + lsscore_cum(rl, corr_comb_size-1) - lsscore_cum(rl, K); 

      // Score géométrique
      float geo_score = -1;
      float geo_weight = 0.0;
      try {
        auto S = line_pairs_geom_scores.at(std::pair<int,int>(ll,rl));
        geo_score = S.first;
        geo_weight = S.second;
      } catch ( const std::exception & e ) {}

      // on regarde les marques orphelines
      double orphan_marks_score = 0.0;
      std::vector<double> orphan_marks_scores;
      double marks_avoiding_ratio = 0.05; // ratio de mauvaise marques à ne pas considérer
      if (marks.size() > 0) {
        for (int m=0; m<(int)marks.size(); m++) {
          int pow_factor = 3;
          try {
            int X_inter = line_pairs_X_inter.at(std::pair<int,int>(ll,rl));
            if (marks[m].x < X_inter)
              orphan_marks_scores.push_back(FieldBorder::dist_point_line(line_eq[ll], marks[m]));
            else
              orphan_marks_scores.push_back(FieldBorder::dist_point_line(line_eq[rl], marks[m]));
          } catch ( const std::exception & e ) {
            orphan_marks_scores.push_back(min(FieldBorder::dist_point_line(line_eq[ll], marks[m]),
                                              FieldBorder::dist_point_line(line_eq[rl], marks[m])));
          }
        }
        // On elimine les (marks_avoiding_ratio%) plus mauvaise marques (supposées être du bruit)
        // (percentille), et on calcule la moyenne
        std::sort(orphan_marks_scores.begin(), orphan_marks_scores.end());
        int N = (int) (orphan_marks_scores.size() * (1.0-marks_avoiding_ratio));
        for (int n=0; n<N; n++)
          orphan_marks_score += orphan_marks_scores[n];
        if (N>0) orphan_marks_score /= N;
      }

      
      /* paramétrage du score */
      float pre_weight = 0.5;
      float sep_weight = 2.0;
      /* geo_weight est dynamique, definit avant */
      float orphan_weight = 5.0;
      
      float score =
        ((pre_weight * precision_score) +
         (sep_weight * separation_score) +
         (geo_weight * geo_score) +
         (orphan_weight * orphan_marks_score)) / (pre_weight + sep_weight + geo_weight + orphan_weight);
	
      if (best_score < 0 || best_score > score) {
        best_score = score;
        best_separation_score = separation_score;
        best_precision_score = precision_score;
        best_geom_score = geo_score;
        best_K = K;
        best_orphan_mark_score = orphan_marks_score;
        l_line = ll;
        if (best_K == corr_comb_size-2)
          r_line = -1;
        else
          r_line = rl;
      }
    }
  }

  FBPRINT_DEBUG("<< global score : %f >>\n", best_score);
  FBPRINT_DEBUG("- precision score  : %f\n", best_precision_score);
  FBPRINT_DEBUG("- separation score : %f\n", best_separation_score);
  FBPRINT_DEBUG("- orphan marks     : %f\n", best_orphan_mark_score);
  if (best_geom_score > 0) {
    FBPRINT_DEBUG("- geometric score  : %f\n", best_geom_score);
  } else {
    FBPRINT_DEBUG("- no geometric score\n");
  }

}
  
void
FieldBorder::compute_separation_score(cv::Mat & green_density,
                                      std::vector<double> & threshold,
                                      std::vector<cv::Vec2f> & line_eq,
                                      std::vector<std::pair<int, int> > & line_x_domain) {
  
  float sep_penalty = 0.1; // la penalité est très arbitraire...
  int delta_y = 1; 
  for (int l=0; l<(int) line_eq.size(); l++) {
    int min_x = line_x_domain[l].first;
    int max_x = line_x_domain[l].second;
    for (int k=0; k<corr_comb_size; k++) {
      int Xk = k*comb_dx;
      if (Xk <= min_x) {
        lsscore_cum(l,k) = 0;
        continue;
      }
      if (Xk >= max_x) {
        if (k==0)
          lsscore_cum(l,k) = 0;
        else
          lsscore_cum(l,k) = lsscore_cum(l,k-1);
        continue;
      }
      int Yk = (int) (line_eq[l][0] * Xk + line_eq[l][1]);
      float separation_score = 0.0;
      float Yu = Yk-delta_y;
      bool found = false;
      while (Yu > 0 && !found) {
        if (0 <= Yu && Yu < row_nb) { 
          float up_v = green_density.at<uchar>((int) (gd_ratio*Yu), (int) (gd_ratio*Xk));
          if (up_v > threshold[k]) separation_score += sep_penalty;
          else found = true;
        }
        Yu -= delta_y;
      }
      
      float Yb = Yk+delta_y;
      found = false;
      while (Yb < row_nb && !found) {
        if (0 <= Yb && Yb < row_nb) { 
          float bot_v = green_density.at<uchar>((int) (gd_ratio*Yb), (int) (gd_ratio*Xk));
          if (bot_v < threshold[k]) separation_score += sep_penalty;
          else found = true;
        }
        Yb += delta_y;
      }
      
      separation_score = separation_score * scale_factor;
      if (k==0)
        lsscore_cum(l,k) = separation_score;
      else
        lsscore_cum(l,k) = lsscore_cum(l,k-1) + separation_score;
    }
  }
}
  
void
FieldBorder::compute_geometric_score(std::map< std::pair<int,int>, std::pair<double,double> >
                                       & line_pairs_geom_scores,
                                     std::map< std::pair<int,int>, int > & line_pairs_X_inter,
                                     std::vector<cv::Vec2f> & line_eq,
                                     std::vector<std::pair<int, int> > & line_x_domain) {
  
  for (int ll=0; ll<(int) line_eq.size(); ll++) {
    for (int rl=0; rl<(int) line_eq.size(); rl++) {
      if (ll == rl) {
        // geometricaly, taking just one line is good.
        line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
          std::pair<double,double>(0.0, 1.0);
        continue;
      }
      double diff_a = line_eq[ll][0] - line_eq[rl][0];
      if (diff_a == 0) {
        // parallèles ou confondues
        line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
          std::pair<double,double>(1000, 1.0);
        continue;
      }
      int X_inter = (int) ((line_eq[rl][1] - line_eq[ll][1]) / diff_a);
      line_pairs_X_inter[std::pair<int,int>(ll,rl)] = X_inter;
      if (X_inter < 0 || X_inter >= col_nb) {
        // intersection en dehors de l'image,
        // on disqualifie la solution
        line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
          std::pair<double,double>(1000, 1.0);
        continue;
      }

      double pix_angle =
        atan2(line_eq[ll][0],1.0) - atan2(line_eq[rl][0],1.0);
      if (rad2deg(pix_angle) < 20.0 ) {
        // param: angle en pix minimum pour considérer
        // le critère géométrique (qui est très couteux).
        continue;
      }
      
      std::vector<cv::Vec2f> line_pair_eq = { line_eq[ll], line_eq[rl] };
      std::vector<std::pair<int, int> >
        line_pair_domain = { line_x_domain[ll], line_x_domain[rl] };
      std::vector<double> line_quality = { -1, -1 };
      update_loc_data(line_pair_eq, line_pair_domain, line_quality, X_inter, false);
      float geo_score = -1;
      float geo_weight = 0.0;
      if (loc_data.hasCorner || loc_data.hasSegment) {
        float a = loc_data.getCornerAngle(); // in [0, PI] (in particular, positive)
        geo_score = min(fabs(a), (float) min(fabs(a-M_PI/2), fabs(a-M_PI)));
        geo_score *= 180 / M_PI; // in degree
        geo_score = 10 * pow(geo_score / 10, 3.0); // On creuse le score en dessous de 10 degres 
        float maxD = 10.0; // Note: parametre
        float D = min(loc_data.getRobotCornerDist(), maxD);
        geo_weight = pow((maxD-D) / maxD, 2.0); // x^2 pour creuser la courbe

        line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
          std::pair<double,double>(geo_score, geo_weight);
        // Faire un cas particulier des lignes à très haut score (ligne blanche,
        // ou coin ?        
      } else {
        // solution mitigée...
        line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
          std::pair<double,double>(10, 1.0);
        continue;
        // Note: ou bien (mais quand la geométrie ne marche pas, ca pénalise terriblement) :
        // line_pairs_geom_scores[std::pair<int,int>(ll,rl)] =
        //   std::pair<double,double>(1000, 1.0);
      }
    }
  }
}
  
void
FieldBorder::compute_precision_scores(std::vector<cv::Point> & marks,
                                      std::vector<cv::Vec2f> & line_eq,
                                      std::vector<std::pair<int, int> > & line_x_domain) {
    
  allocate_scores(line_eq.size(), corr_comb_size);
  std::vector<float> line_score_sum;
  for (int l=0; l<(int) line_eq.size(); l++)
    line_score_sum.push_back(0);
  std::vector<float> line_score_sq_sum;
  for (int l=0; l<(int) line_eq.size(); l++)
    line_score_sq_sum.push_back(0);

  for (int l=0; l<(int) line_eq.size(); l++) { // pour chaque ligne
    for (int k=0; k<corr_comb_size; k++) { // pour chaque colonne du peigne
      // on calcule la distance à la marque la plus proche de la colonne considérée
      // c'est le premier élément du score
      float min_d = -1;
      for (int m=0; m<(int)marks.size(); m++) {
        int X = k*comb_dx;
        auto domain = line_x_domain[l];
        if (X < domain.first || X > domain.second) continue;
	if (marks[m].x != X) continue;
	float d =
	  fabs(line_eq[l][0] * marks[m].x + line_eq[l][1]
	       - marks[m].y);
	if (min_d < 0 || min_d > d) min_d = d;
      }

      if (min_d < 0) {
	/* Note:
         * option 1: s'il n'y a pas de marque sur la colonne, 
	 * on attribue alors la distance au point le plus proche hors image */
	/* float ys = line_eq[l][0]*k*comb_dx + line_eq[l][1];
	   min_d = min(ys, row_nb - ys); */
        /* option 2: */
        min_d = 0; /* si on a pas de marque (c'est le brouillard), 
                      ou bien que l'on est en dehors du domaine */
      }
      
      lfscore(l,k) = min_d;
      line_score_sum[l] += min_d;
      lfscore_sq(l,k) = min_d * min_d;
      line_score_sq_sum[l] += min_d * min_d;
      lfscore_cum(l,k) =
	(k==0) ? lfscore(l,k) : lfscore_cum(l,k-1) + lfscore(l,k);
      lfscore_sq_cum(l,k) =
	(k==0) ? lfscore_sq(l,k) : lfscore_sq_cum(l,k-1) + lfscore_sq(l,k);
    }
  }
}
  
void
FieldBorder::compute_marks(cv::Mat & green_density,
                           std::vector<cv::Point> & marks,
                           std::vector<double> & threshold) {
  for (int x = 0; x < col_nb; x+= comb_dx) { // on boucle sur les dents du peigne

    // On compte les valeurs de la colonne considérée
    int values[256];
    for (int v=0; v<256; v++) values[v]=0;
    for (int y=0; y<row_nb; y++)
      values[green_density.at<uchar>((int) (gd_ratio*y), (int) (gd_ratio*x))]++;
    // Calcul de la moyenne des valeurs
    float sum = 0;
    int size = 0;
    for (int i=0; i<255; i++)
      sum += i*values[i];
    sum /= row_nb;
    
    // On détermine 2 paquets sur les valeurs des densités (vert et non vert) (variation de k-mean)
    // Note: on calcule les déviations standarts des 2 paquets pour voir s'ils sont bien distincts
    /*
    int m1 = (int) (sum * 0.80),
        m2 = (int) (sum * 1.20); // Initialisation autour de la moyenne
    */
    int m1 = 0,
        m2 = (int) sum;
    if (m1 > 255) m1 = 255;
    if (m2 > 255) m2 = 255;
    int turn_nb = 10; // Note: Nombre de tours très arbitraire
    float sum_sq = 0, stddev1 = 0, stddev2 = 0;
    bool separated = true;
    int I = -1, previous_I = -1;
    while (turn_nb > 0) {
      if (m1 > m2) {
        int c = m1;
        m1 = m2; m2 = c;
      }
      I=(m1+m2)/2; // I est le séparateur des deux paquets
      if (I == previous_I) break; // l'algo a convergé
      previous_I = I;
      // dans la suite, on calcule la moyenne et l'écart-type des deux paquets
      // [0, I[ et [I, 255]
      sum = sum_sq = 0;
      size = 0;
      for (int i=0; i<=I; i++) {
	sum_sq += i*i*values[i];
	sum += i*values[i];
	size += values[i];
      }
      if (size > 0) {
	m1 = sum / size;
	float V = sum_sq / size - m1*m1;
	if (V<0) V=0;
	stddev1 = sqrt(V);
      } else {
	separated = false;
	break;
      }
      sum = 0;
      size = 0;
      for (int i=I+1; i<=255; i++) {
	sum_sq += i*i*values[i];
	sum += i*values[i];
	size += values[i];
      }
      if (size > 0) {
	m2 = sum / size;
      	float V = sum_sq / size - m2*m2;
	if (V<0) V=0;
	stddev2 = sqrt(V);
      }
      else {
	separated = false;
	break;
      }
      turn_nb--;
    }

    I = (m1+m2)/2;
    // On corrige le I en fonction de la répartition
    if (stddev1 != 0 && stddev2 != 0) {
        int I1, I2;
        float a = 0.1;
        do {
          I1 = (int) (m1 + a * stddev1);
          I2 = (int) (m2 - a * stddev2);
          a += 0.1;
        } while (I1 < I2);
        I = (I1+I2)/2;
    }
    
    threshold[x/comb_dx] = I;
    // on verifie qu'on a bien 2 clusters
    separated = separated && ((m1 + stddev1) < (m2 - stddev2)); 
    
    // Creation des marques par détection de transition par seuillage
    // mais on ne le fait que si on a réussit à identifier 2 paquets
    // de valeurs
    if (separated) {
      // ici, on descend la dents du peigne jusqu'au seuil, on place alors une marque.
      // si on a traversé "suffisamment" de vert, on ne pose plus de marque car on
      // suppose qu'on est dans le vert (pour ne pas marquer les lignes blanches)
      bool in_green = false;
      int green_count = 0;
      for (int y=0; y<row_nb; y++) {
	bool mark = false;
	if (in_green && green_density.at<uchar>((int) (gd_ratio*y), (int) (gd_ratio*x)) < threshold[x/comb_dx]) {
	  in_green = false;
	}
	else
	  if (!in_green && green_density.at<uchar>((int) (gd_ratio*y), (int) (gd_ratio*x)) >= threshold[x/comb_dx]) {
	    in_green = true;
	    if (y > 0)
	      // on ne marque pas la première ligne,
	      // ca veut dire qu'on est déjà dans le vert
	      mark = true;
	  }
	if (mark)
	  marks.push_back(cv::Point(x,y));

	// On ne marque pas si on a déja passé beaucoup de vert
	// car on suppose qu'on est dans le terrain
	// On fixe à 10% de la hauteur de l'image
	// (ca elimine certaine ligne blanche
	if (in_green) green_count++;
	if (green_count > row_nb * 0.10) break;
      }
    }
  }
}

void
FieldBorder::line_detection(std::vector<cv::Point> & marks,
                            std::vector<cv::Vec2f> & brut_lines,
                            std::vector<cv::Vec2f> & line_eq,
                            std::vector<std::pair<int, int> > & line_x_domain,
                            int hough_threshold,
                            int pix_precision,
                            int angle_precision) {
  // Détection des lignes avec Hough pour matcher les marques identifiées précédemment
  cv::Mat marks_img;
  marks_img.create(row_nb, col_nb, 0);
  marks_img.setTo(0);

  // On place les marques 
  for (int k=0; k<(int)marks.size(); k++)
    marks_img.at<uchar>(marks[k].y,marks[k].x) = 255;

  do {
    brut_lines.clear();
    cv::HoughLines(marks_img, brut_lines, pix_precision,
                   angle_precision*CV_PI/180, hough_threshold, 0, 0);
    hough_threshold += 5;
  } while (brut_lines.size() > 50); // Param: nbre de ligne maximum pour maitriser le temps
  FBPRINT_DEBUG("brut line nb : %d\n", (int) brut_lines.size());
  
  if (brut_lines.size() == 0) {
    loc_data.reset();
    return;
  }
    
  // affichage des lignes et calcul des equations y=ax+b
  for( size_t i = 0; i < brut_lines.size(); i++ ) {
    float rho = brut_lines[i][0], theta = brut_lines[i][1];
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    // Calcul des équations de forme y=ax+b
    if (b != 0) { // Note: On ne considère pas les lignes verticales, plus tard peut-être...
      cv::Point A,B;
      A.x = 0; A.y = y0 + (x0/b)*a;
      B.x = col_nb; B.y = y0 + ((x0-col_nb)/b)*a;
      cv::Vec2f eq(((float) (B.y - A.y))/(B.x - A.x), A.y);
      line_eq.push_back(eq);
      // calcul du domaine de définition de la droite
      // TODO: calcul direct pour gagner du temps de calcul
      int x_min=-1, y;
      do {
	x_min++;
	y = eq[0]*x_min + eq[1];
      } while (y < 0 || y > row_nb);
      int x_max=col_nb;
      do {
	x_max--;
	y = eq[0]*x_max + eq[1];
      } while (y < 0 || y > row_nb);
      line_x_domain.push_back(std::pair<int,int>(x_min, x_max));
    } 
  }
}

void
FieldBorder::filter_bad_marks(std::vector<cv::Point> & marks,
                              std::vector<cv::Point> & bad_marks,
                              std::vector<cv::Vec2f> & line_eq,
                              int pix_precision) {
  
  int idx = marks.size()-1;
  while (idx > 0) {
    // 1. la distance minimale à l'une des droites de Hough
    double min_D = -1;
    for (int l=0; l<(int)line_eq.size(); l++) {
      double D = FieldBorder::dist_point_line(line_eq[l],
                                              cv::Point2f(marks[idx].x,
                                                          marks[idx].y));
      if (min_D < 0 || min_D > D) min_D = D;
    }
      // 2. si cette distance est supérieure à pix_precision, on l'élimine
    if (min_D > 2 * pix_precision) {
      bad_marks.push_back(marks[idx]);
      marks.erase(marks.begin() + idx);
    }
    idx--;
  }
}
  
float * FieldBorder::line_fine_score = NULL;
float * FieldBorder::line_fine_score_square = NULL;
float * FieldBorder::line_fine_score_cum = NULL;
float * FieldBorder::line_fine_score_square_cum = NULL;
float * FieldBorder::line_sep_cum_score = NULL;
int FieldBorder::line_nb = 0;

bool FieldBorder::allocate_scores(int _line_nb, int _comb_size) {
  if (_line_nb > line_nb) {
    _line_nb += 10; /* marge de sécurité, pour ne pas réallouer tout le temps */
    clear_score_mem();
    line_fine_score =
      (float*) malloc(sizeof(float)*_line_nb*_comb_size);
    line_fine_score_square =
      (float*) malloc(sizeof(float)*_line_nb*_comb_size);
    line_fine_score_cum =
      (float*) malloc(sizeof(float)*_line_nb*_comb_size);
    line_fine_score_square_cum =
      (float*) malloc(sizeof(float)*_line_nb*_comb_size);
    line_sep_cum_score =
      (float*) malloc(sizeof(float)*_line_nb*_comb_size);

    line_nb = _line_nb;
  }
  
  if (line_fine_score == NULL ||
      line_fine_score_square == NULL ||
      line_fine_score_cum == NULL ||
      line_fine_score_square_cum == NULL ||
      line_sep_cum_score == NULL) {
    line_nb = 0;
    return false;
  }

  return true;
}

void FieldBorder::clear_score_mem() {
  if (line_fine_score != NULL) delete line_fine_score;
  if (line_fine_score_square != NULL) delete line_fine_score_square;
  if (line_fine_score_cum != NULL) delete line_fine_score_cum;
  if (line_fine_score_square_cum != NULL) delete line_fine_score_square_cum;
  if (line_sep_cum_score != NULL) delete line_sep_cum_score;
  line_fine_score = NULL;
  line_fine_score_square = NULL;
  line_fine_score_cum = NULL;
  line_fine_score_square_cum = NULL;
  line_sep_cum_score = NULL;
}

FieldBorder::~FieldBorder() {
  FieldBorder::clear_score_mem();
}
  
FieldBorderData * FieldBorder::current_loc_data = NULL;

double FieldBorder::dist_point_line(cv::Vec2f line_eq, cv::Point M) {
  return FieldBorder::dist_point_line(line_eq, cv::Point2f(M.x,M.y));
}

double FieldBorder::dist_point_line(cv::Vec2f line_eq, cv::Point2f M) {
  double a = line_eq[0], b = line_eq[1];
  double t = (a * M.x + b - M.y) / (1 + a*a);
  cv::Point2f H = M + t * cv::Point2f(-a,1);
  return cv::norm(H-M);
}

double FieldBorder::best_score = -1;
double FieldBorder::get_score() {
  if (best_score < 0) return 1000;
  return best_score;
}

#undef lfscore
#undef lfscore_sq
#undef lfscore_cum
#undef lfscore_sq_cum
#undef lsscore_cum

}
}
