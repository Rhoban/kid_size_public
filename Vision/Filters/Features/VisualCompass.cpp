#include <stdexcept>
#include "Filters/Features/VisualCompass.hpp"

#include <opencv2/legacy/legacy.hpp>
#include "rhoban_utils/timing/time_stamp.h"
#include "Utils/RotatedRectUtils.hpp"
#include "rhoban_utils/logging/logger.h"
#include "CameraState/CameraState.hpp"
#include <vector>
#include "rhoban_utils/timing/benchmark.h"
#include "services/ModelService.h"
#include <cmath>
#include "rhoban_utils/logging/logger.h"
#include "Filters/Features/homography_decomp.hpp"
#include <cvdrawingutils.h>
#include <opencv2/calib3d/calib3d.hpp>
// #include <iterator>
#include "Filters/Features/precomp_import.hpp"
#include <algorithm>

// FIXME: this should be elsewhere
#define PIXEL_SIZE 0.00375  // in mm (camera BFLY-PGE-12A2C-CS sensor Aptina AR0134)
#define NATIVE_WIDTH 1280
#define NATIVE_HEIGHT 960

using ::rhoban_utils::Benchmark;
using namespace aruco;
using namespace rhoban_utils;
static rhoban_utils::Logger out("VisualCompass");

namespace Vision {
namespace Filters {

VisualCompass::VisualCompass() : CompassProvider("VisualCompass"), active_field(-1) {
  CamParam.readFromXMLFile("camera_calib.yml");
  cameraparamsdone = false;
  prev_maskBelow=0;
  prev_maskAbove=1;
}

void VisualCompass::initSIFT() {
  Benchmark::open("VisualCompass create SIFT");
  // initialize algoritm
  // detector = cv::FeatureDetector::create("SIFT");
  // descriptor_extractor = cv::DescriptorExtractor::create("SIFT");

  detector = new cv::SiftFeatureDetector();
  descriptor_extractor = new cv::SiftDescriptorExtractor();

  // Matcher
  // descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING); //Brute force
  descriptorMatcher = new cv::FlannBasedMatcher(new cv::flann::KDTreeIndexParams(5),
                                                new cv::flann::SearchParams(50));  // FLANN KNN FIXME params
  // cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(5),new cv::flann::SearchParams(50));
  // descriptorMatcher = new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(12,20,2)); //FLANN KNN FIXME params
  // //seems to be table_number, key_size, multi_probe_level

  Benchmark::close("VisualCompass create SIFT");

  // load the pano
  Benchmark::open("VisualCompass init pano SIFT");

  // Compute the features of the pano
  detector->detect(field, keypoints_pano,field_mask);
  descriptor_extractor->compute(field, keypoints_pano, descriptors_pano);

  Benchmark::close("VisualCompass init pano SIFT");
}

void VisualCompass::setParameters() {
  debugLevel = ParamInt(0, 0, 2);
  params()->define<ParamInt>("debugLevel", &debugLevel);

  nndrRatio = ParamFloat(0.81, 0, 1);// 0.85
  params()->define<ParamFloat>(
      "nndrRatio", &nndrRatio);  // Matching. Distance ratio for correct matching. Higher values=less restrictive match
  
  knn_K = ParamInt(2, 0, 10);
  params()->define<ParamInt>("knn_K", &knn_K);  // number of neigbors

  panoScale = ParamFloat(1, 0.1, 2);
  params()->define<ParamFloat>("panoScale", &panoScale);  // scale ratio for the pano img

  fieldNumber = ParamInt(0, 0, 10);
  params()->define<ParamInt>("fieldNumber", &fieldNumber);  // number of the field

  enabled = ParamInt(0, 0, 1);
  params()->define<ParamInt>("enabled", &enabled);  // process the filter?


  maskBelow=ParamFloat(0, 0, 1); //% of the image
  params()->define<ParamFloat>("maskBelow", &maskBelow);

  maskAbove=ParamFloat(1, 0, 1); //% of the image
  params()->define<ParamFloat>("maskAbove", &maskAbove);

}

void VisualCompass::fromJson(const Json::Value & v, const std::string & dir_name)
{
  CompassProvider::fromJson(v, dir_name);
  updateField();
}

void VisualCompass::updateField()
{

  switch (fieldNumber) {
    case 0: //A
      out.log("Opening panoramic image: pano.jpg");
      field = cv::imread("../common/pano.jpg", 0);
      out.log("Opening panoramic image mask: pano_mask.jpg");
      field_mask=cv::imread("../common/pano_mask.jpg", 0);
      break;
    case 1: //C
      out.log("Opening panoramic image: pano.jpg");
      field = cv::imread("../common/pano.jpg", 0);
      out.log("Opening panoramic image mask: pano_mask.jpg");
      field_mask=cv::imread("../common/pano_mask.jpg", 0);
      break;
    case 2: //E
      out.log("Opening panoramic image: pano.jpg");
      field = cv::imread("../common/pano.jpg", 0);
      out.log("Opening panoramic image mask: pano_mask.jpg");
      field_mask=cv::imread("../common/pano_mask.jpg", 0);
      break;
    default: //ENSEIRB!!!!!
      out.log("Opening panoramic image: pano.jpg");
      field = cv::imread("../common/pano.jpg", 0);
      out.log("Opening panoramic image mask: pano_mask.jpg");
      field_mask=cv::imread("../common/pano_mask.jpg", 0);
      break;
  }

  out.log("field before resize: %d * %d", field.cols, field.rows);
  out.log("field_mask before resize: %d * %d", field_mask.cols, field_mask.rows);
  cv::resize(field, field, cv::Size(), panoScale, panoScale);  // depends on the size of the pano.
  cv::resize(field_mask, field_mask, cv::Size(), panoScale, panoScale);  // depends on the size of the pano.
  out.log("field after resize: %d * %d", field.cols, field.rows);
  out.log("field_mask after resize: %d * %d", field_mask.cols, field_mask.rows);
  // Ideally the panorama should be done with the Robot's camera. Otherwise, errors may be introduced.
}

void VisualCompass::getPoints(const std::vector<cv::KeyPoint>& kpts_train, const std::vector<cv::KeyPoint>& kpts_query,
                              std::vector<cv::Point2f>& pts_train, std::vector<cv::Point2f>& pts_query,
                              std::vector<cv::DMatch>& good_matches) {
  pts_train.clear();
  pts_query.clear();
  for (size_t k = 0; k < good_matches.size(); k++) {
    const cv::DMatch& match = good_matches[k];
    pts_query.push_back(kpts_query[match.queryIdx].pt);
    pts_train.push_back(kpts_train[match.trainIdx].pt);
  }
}

void VisualCompass::matchSIFT(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& good_matches) {
  cv::Mat descriptors;

  // detection
  Benchmark::open("VisualCompass SIFT detect");

  detector->detect(img(), keypoints, img_mask);

  Benchmark::close("VisualCompass SIFT detect");

  // Compute features
  Benchmark::open("VisualCompass SIFT compute");

  descriptor_extractor->compute(img(), keypoints, descriptors);

  Benchmark::close("VisualCompass SIFT compute");

  // TODO debug level
  Benchmark::open("VisualCompass SIFT draw");
  cv::drawKeypoints(img(), keypoints, img()); //Useless
  Benchmark::close("VisualCompass SIFT draw");

  // std::vector<cv::DMatch> matches;
  std::vector<std::vector<cv::DMatch> > matches;

  cv::Mat results, dists;

  Benchmark::open("VisualCompass SIFT match");

  // flannIndex->knnSearch(descriptors, results, dists, knn_K, cv::flann::SearchParams() );

  // Don't know the difference...
  descriptorMatcher->knnMatch(descriptors, descriptors_pano, matches, 2);
  if (debugLevel >= 1) {
    out.log("Matches: %d", matches.size());
  }

  Benchmark::close("VisualCompass SIFT match");

  // std::cerr<<"DEBUG "<<matches.size()<<std::endl;
  for (int k = 0; k < (int)matches.size(); k++) {
    if ((matches[k][0].distance < nndrRatio * (matches[k][1].distance)))  // &&
      //  ((int)matches[k].size() <= 2 && (int)matches[k].size()>0) ) //TODO param
    {
      // take the first result only if its distance is smaller than 0.6*second_best_dist
      // that means this descriptor is ignored if the second distance is bigger or of similar
      good_matches.push_back(matches[k][0]);
    }
  }

  /*
  //crash, for unknown reasons...
  std::vector<cv::Point2f>pts_train;
  std::vector<cv::Point2f>pts_query;
  getPoints(keypoints_pano,keypoints, pts_train, pts_query,good_matches);
  cv::Mat E, R, t, mask;
  E = findEssentialMat(pts_query, pts_train, CamParam.CameraMatrix, cv::RANSAC, 0.999, 1.0, mask);
  recoverPose(E, pts_query, pts_train, R, t, CamParam.CameraMatrix, mask);
  */

    if (debugLevel >= 1) {
      out.log("Good matches: %d", good_matches.size());
    }

    // cv::Mat img_goodmatches;

    if (good_matches.size() > 0) {
      if (debugLevel >= 2) {
	
        cv::drawMatches(img(), keypoints, field, keypoints_pano, good_matches, img_debug, cv::Scalar(0, 0, 255),
                        cv::Scalar(255, 0, 0), std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::resize(img_debug, img_debug, cv::Size(), 0.5, 0.5);
	cv::resize(img_debug, img_debug, cv::Size(), 1.2, 1.2);


        // cv::imshow("VisualCompass SIFT debug goodmatches", img_goodmatches);
        // cv::imshow("VisualCompass SIFT debug", img_debug);
	// cv::waitKey(1);
        //////
      }
    }
}

void VisualCompass::process() {
  // First clean memory
  clearCompassData();

  bool shouldUpdate=false;
  //First update the field
  if(active_field != fieldNumber)
  {
    updateField();
    active_field=fieldNumber;
    shouldUpdate=true;
  }

  if (shouldUpdate) {
    initSIFT();
  }


  if (enabled) {
    const cv::Mat& srcImg = *(getDependency().getImg());
    // TODO: clone only when debugLevel > 0
    img() = srcImg.clone();



    // DEBUG
    // cv::cvtColor(img(),img(),CV_YCrCb2BGR);
    // img()=  cv::imread("test.jpg",0);


    if (!cameraparamsdone) {
      width = srcImg.cols;
      height = srcImg.rows;
      pano_width = field.cols;
      pano_height = field.rows;

      //init image mask
      img_mask=cv::Mat(height,width,CV_8UC1, cv::Scalar(255));



      // std::cout<<"WIDTH: "<<width<<" "<<pano_width<<" HEIGHT: "<<height<<" "<<pano_height<<std::endl;

      double appertureW = NATIVE_WIDTH / width * PIXEL_SIZE;
      double appertureH = NATIVE_HEIGHT / height * PIXEL_SIZE;
      double fovx, fovy, focalLength, aspectRatio;
      cv::Point2d principalPoint;

      cv::calibrationMatrixValues(CamParam.CameraMatrix, cv::Size(width, height), appertureW, appertureH, fovx, fovy,
                                  focalLength, principalPoint, aspectRatio);
      hfov_rad = deg2rad(fovx);
      // std::cerr<<"DEBUG CAMERA: "<<fovx<<" "<<fovy<<" "<<focalLength<<" "<<principalPoint<<"
      // "<<aspectRatio<<std::endl;

      // std::cerr<<"DEBUG MATRIX: "<<CamParam.CameraMatrix.at< float >(0, 0)<<" "<<CamParam.CameraMatrix.at< float >(1,
      // 1)<<std::endl;

      pixeltoangle_pano = (2.0 * M_PI) / pano_width;
      angletopixel_pano = pano_width / (2.0 * M_PI);
      pixeltoangle = hfov_rad / width;  // FIXME, we should have this value somewhere...
      cameraparamsdone = true;
    }


    if((prev_maskBelow != maskBelow) || (prev_maskAbove != maskAbove)) //mask has changed
    {
      //start black
      img_mask=cv::Mat(height,width,CV_8UC1, cv::Scalar(0));
      //draw white
      cv::Point pt1(0,maskBelow*height);
      cv::Point pt2(width,maskAbove*height);
      cv::rectangle(img_mask,pt1,pt2,cv::Scalar(255),CV_FILLED);

    }


    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::DMatch> good_matches;

    matchSIFT(keypoints, good_matches);

    if (good_matches.size() > 0) {
      Benchmark::open("VisualCompass compute angle");
      computeAngle(keypoints, good_matches);
      Benchmark::close("VisualCompass compute angle");

      
      // cv::imshow("VisualCompass SIFT debug", img());
      // cv::waitKey(1);      
    } else {
      // humm
    }
  }
}

void VisualCompass::cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose) {
  pose = cv::Mat::eye(3, 4, CV_32FC1);  // 3x4 matrix, the camera pose
  float norm1 = (float)norm(H.col(0));
  float norm2 = (float)norm(H.col(1));
  float tnorm = (norm1 + norm2) / 2.0f;  // Normalization value

  cv::Mat p1 = H.col(0);     // Pointer to first column of H
  cv::Mat p2 = pose.col(0);  // Pointer to first column of pose (empty)

  cv::normalize(p1, p2);  // Normalize the rotation, and copies the column to pose

  p1 = H.col(1);     // Pointer to second column of H
  p2 = pose.col(1);  // Pointer to second column of pose (empty)

  cv::normalize(p1, p2);  // Normalize the rotation and copies the column to pose

  p1 = pose.col(0);
  p2 = pose.col(1);

  cv::Mat p3 = p1.cross(p2);  // Computes the cross-product of p1 and p2
  cv::Mat c2 = pose.col(2);   // Pointer to third column of pose
  p3.copyTo(c2);              // Third column is the crossproduct of columns one and two

  pose.col(3) = H.col(2) / tnorm;  // vector t [R|t] is the last column of pose
}

void VisualCompass::sanitizeCoordinates(const std::vector<cv::Point2f>& in, std::vector<cv::Point2f>& out) {
  // not sure...
}

void VisualCompass::computeAngle(std::vector<cv::KeyPoint> keypoints, const std::vector<cv::DMatch>& good_matches) {

  cv::Mat outangle;

  if (debugLevel >= 2) {
    outangle = field.clone();
    cv::cvtColor(outangle, outangle, CV_GRAY2RGB);
  }

  isHomographyGood = false;
  cv::Mat output_mask;
  if (good_matches.size() > 8)  // seems like we need at least 8
  {
    // Homography
    // Here, homography is only used to filter out the outliers

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (size_t i = 0; i < good_matches.size(); i++) {
      //-- Get the keypoints from the good matches
      obj.push_back(keypoints[good_matches[i].queryIdx].pt);
      scene.push_back(keypoints_pano[good_matches[i].trainIdx].pt);
    }

    // TODO fix the angular coordinate (take into account the full loop of the panorama)
    // std::vector<cv::Point2f> scene_fix;
    // sanitizeCoordinates(scene,scene_fix);

    Benchmark::open("VisualCompass homography");

    cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 5, output_mask);
    // cv::Mat H = cv::findHomography( obj, scene, CV_LMEDS,5 ); //LMEDS, not good

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point(0, 0);
    obj_corners[1] = cv::Point(width - 1, 0);
    obj_corners[2] = cv::Point(width - 1, height - 1);
    obj_corners[3] = cv::Point(0, height - 1);
    std::vector<cv::Point2f> scene_corners(4);

    cv::perspectiveTransform(obj_corners, scene_corners, H);
    Benchmark::close("VisualCompass homography");

    if (debugLevel >= 2) {
      // cv::perspectiveTransform( obj_corners, scene_corners, H);
      cv::line(outangle, scene_corners[0], scene_corners[1], cv::Scalar(255, 255, 255), 4);
      cv::line(outangle, scene_corners[1], scene_corners[2], cv::Scalar(255, 255, 255), 4);
      cv::line(outangle, scene_corners[2], scene_corners[3], cv::Scalar(255, 255, 255), 4);
      cv::line(outangle, scene_corners[3], scene_corners[0], cv::Scalar(255, 255, 255), 4);
    }

    // Convexity test for the found homography polygon
    // Compute the z component of the cross product for each pair of edges
    float sign[4];
    for (int i = 0; i < 4; i++) {
      float dx1 = scene_corners[(i + 1) % 4].x - scene_corners[i % 4].x;
      float dy1 = scene_corners[(i + 1) % 4].y - scene_corners[i % 4].y;

      float dx2 = scene_corners[(i + 2) % 4].x - scene_corners[(i + 1) % 4].x;
      float dy2 = scene_corners[(i + 2) % 4].y - scene_corners[(i + 1) % 4].y;

      float zproduct = dx1 * dy2 - dy1 * dx2;
      sign[i] = zproduct;
      // std::cerr<<"\t "<<zproduct<<std::endl;
    }

    // if the 4 values are of the same sign, the polygon is convex, which is a pretty good indication that the
    // homography is not too bad...
    if ((sign[0] > 0 && sign[1] > 0 && sign[2] > 0 && sign[3] > 0) ||
        (sign[0] < 0 && sign[1] < 0 && sign[2] < 0 && sign[3] < 0)) {
      // We can reasonably trust the homography
      if (debugLevel >= 1) {
        out.log("VisualCompass: homography polygon is convex");
      }
      isHomographyGood = true;

      // Huumm? Doesn't work?
      // cv::Mat pose;
      // cameraPoseFromHomography(H,pose);
      // std::cerr<<"POSE: "<<pose<<std::endl;

      /*
      //Overkill but...
      std::vector<cv::Mat> rotations;
      std::vector<cv::Mat> translations;
      std::vector<cv::Mat> normals;
      int nbsol=cv::decomposeHomographyMat(H,CamParam.CameraMatrix,rotations,translations,normals);

      //DEBUG
      // std::cerr<<"VisualCompass: "<<H<<" Camera pose "<<pose<<std::endl;
      std::cerr<<"VisualCompass decomp: "<<nbsol<<std::endl;
      std::cerr<<"VisualCompass rotations:"<<std::endl;
      for(int it=0;it<rotations.size();it++)
      std::cerr<<"\t"<<rotations[it]<<std::endl;
      std::cerr<<std::endl;

      // for (auto R_ : rotations) {
      //   cv::Mat1d rvec;
      //   cv::Rodrigues(R_, rvec);
      //   std::cout << rvec*180/CV_PI << std::endl << std::endl;
      // }


      std::cerr<<"VisualCompass translations:"<<std::endl;
      for(int it=0;it<translations.size();it++)
      std::cerr<<"\t"<<translations[it]<<std::endl;
      std::cerr<<std::endl;




      std::cerr<<"VisualCompass normals:"<<std::endl;
      for(int it=0;it<normals.size();it++)
      std::cerr<<"\t"<<normals[it]<<std::endl;
      std::cerr<<std::endl;
      */

      /*
        for(size_t it=0;it<4;it++)
        {
        //   cv::Mat1d rvec;
        //   cv::Rodrigues(rotations[it], rvec);
        //   std::cerr<<"Rvec: "<<it<<" "<<rvec<<std::endl;
        //   CvDrawingUtils::draw3dAxis(outangle,CamParam,rvec,translations[it],0.25);



        //   // try{CvDrawingUtils::draw3dAxis(outangle,CamParam,rvec,translations[it],0.25);}
        //   // catch(...){std::cerr<<"DRAW failed"<<it<<std::endl;}
        // }
        // */

      /*
        cv::Mat1d rvec;
        cv::Rodrigues(rotations[0], rvec);
        // std::cerr<<"Rvec: "<<it<<" "<<rvec<<std::endl;
        CvDrawingUtils::draw3dAxis(outangle,CamParam,rvec,translations[0],0.25);
      */

      // CvDrawingUtils::draw3dAxis(outangle,CamParam,rotations[0],translations[0],0.25);
      // CvDrawingUtils::draw3dAxis(outangle,CamParam,rotations[1],translations[1],0.25);
      // CvDrawingUtils::draw3dAxis(outangle,CamParam,rotations[2],translations[2],0.25);
      // CvDrawingUtils::draw3dAxis(outangle,CamParam,rotations[1],translations[3],0.25);

    } else  // humm... let's still use the outliers mask (seems to be quite ok)
    {
      if (debugLevel >= 1) {
        out.log("VisualCompass: homography polygon is concave");
      }
      isHomographyGood = false;
    }
  }

  float sines = 0.0;
  float cosines = 0.0;
  int nb_inliers = 0;
  for (size_t i = 0; i < good_matches.size(); i++) {
    if (!output_mask.empty()) {
      // std::cerr<<"DEBUG EMPTY"<<std::endl;
      if (!output_mask.at<uchar>(i, 0)) {  // outlier mask => we don't consider this marker
        // std::cerr<<"MASK"<<std::endl;

        continue;
      }
    }
    //    else
    //   if(!output_mask.empty() && output_mask.at<uchar>(i, 0))
    {
      int img1_idx = good_matches[i].queryIdx;
      int img2_idx = good_matches[i].trainIdx;
      nb_inliers++;
      auto pt1 = keypoints[img1_idx].pt;
      auto pt2 = keypoints_pano[img2_idx].pt;

      // average angle pos of center relative to marker (cam) + pos of
      // marker relative to center (pano)


      float angle = ((width / 2) - pt1.x) * pixeltoangle + (-(pano_width / 2) + pt2.x) * pixeltoangle_pano;
      // float angle =  -(pt2.x-(pano_width / 2)  ) * pixeltoangle_pano + (pt1.x-(width / 2) ) * pixeltoangle ;
      sines += sin(angle);
      cosines += cos(angle);


      if (debugLevel >= 1)
        out.log("angle: %f", rad2deg(angle));

      // debuging (display the inliers)
      if (debugLevel >= 2) {
        cv::circle(outangle, pt2, 10, cv::Scalar(0, 0, 255), 2);
      }
    }
  }

  if (debugLevel >= 1) {
    out.log("Inliers: %d", nb_inliers);
  }

  if (good_matches.size() >= 5) {
    // sines = sines / good_matches.size();
    // cosines = cosines / good_matches.size();

    sines = sines / nb_inliers;
    cosines = cosines / nb_inliers;

    float avangle = atan2(sines, cosines);
    float stdangle = sqrt(-log(pow(sines, 2) + pow(cosines, 2)));
    float R_angle = sqrt(pow(sines, 2) + pow(cosines, 2));  // 0=>total dispersion 1=>no dispersion
    // DEBUG
    int linepos = (int)(avangle * angletopixel_pano);
    int stdpos = (int)(stdangle * angletopixel_pano);
    // int disppos = (int) (1.0-R_angle * angletopixel_pano);

    // if (debugLevel >= 1)
    // {
    //   std::cerr<<"LINEPOS: "<<linepos<<" "<<stdpos<<" "<<linepos + pano_width / 2<<std::endl;
    // }

    // cv::resize(outangle, outangle, cv::Size(), 0.5, 0.5);

    // average angle line
    if (debugLevel >= 2) {
      // cv::line(outangle, cv::Point((linepos - width + pano_width / 2), pano_height), cv::Point((linepos - width +
      // pano_width / 2), 0), cv::Scalar(0, 255, 0), 2);
      cv::line(outangle, cv::Point((linepos + pano_width / 2), pano_height), cv::Point((linepos + pano_width / 2), 0),
               cv::Scalar(0, 255, 0), 2);

      // 2* standard deviation, bof... not very informative. For debug purpose, let's make std/4
      cv::line(outangle, cv::Point((linepos + 2 * stdpos + pano_width / 2), pano_height),
               cv::Point((linepos + 2 * stdpos + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);
      cv::line(outangle, cv::Point((linepos - 2 * stdpos + pano_width / 2), pano_height),
               cv::Point((linepos - 2 * stdpos + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);

      // cv::line(outangle, cv::Point((linepos + stdpos/4  + pano_width / 2), pano_height), cv::Point((linepos +
      // stdpos/4 + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);
      // cv::line(outangle, cv::Point((linepos - stdpos/4 + pano_width / 2), pano_height), cv::Point((linepos - stdpos/4
      // + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);

      // cv::line(outangle, cv::Point((linepos + disppos*1  + pano_width / 2), pano_height), cv::Point((linepos +
      // disppos*1 + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);
      // cv::line(outangle, cv::Point((linepos - disppos*1 + pano_width / 2), pano_height), cv::Point((linepos
      // -disppos*1 + pano_width / 2), 0), cv::Scalar(255, 255, 0), 1);


      cv::resize(outangle, outangle, cv::Size(), 0.5, 0.5);

      cv::Size sz1 = img_debug.size();
      cv::Size sz2 = outangle.size();
      cv::Mat img_debug_concat(sz1.height+sz2.height, max(sz1.width,sz2.width), CV_8UC3,cv::Scalar(0));
      img_debug.copyTo(img_debug_concat(cv::Rect(0, 0, sz1.width, sz1.height)));
      outangle.copyTo(img_debug_concat(cv::Rect(0, sz1.height, sz2.width, sz2.height)));

      img() = img_debug_concat;
    }
    if (debugLevel >= 1) {
    }

    double score = R_angle;

    if (!isHomographyGood) score /= 2.0;

    pushCompass(avangle, score);  // because I can

    out.log("VisualCompassFilter ANGLE: (mean) %f , (std) %f , (score) %f",
            rad2deg(avangle), stdangle, score);
  }
}
}
}
