#include "camera_intrinsics/calibrate.h"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace std;

const static int NUM_CIRCLEGRID = 10;

void calcChessboardCorners(Size boardSize, float squareSize,
                           vector<Point3f> &corners) {
  corners.resize(0);

  for (int i = 0; i < boardSize.height; i++)
    for (int j = 0; j < boardSize.width; j++)
      corners.push_back(
          Point3f(float(j * squareSize), float(i * squareSize), 0));
}

double computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints,
                                 const vector<vector<Point2f> > &imagePoints,
                                 const vector<Mat> &rvecs,
                                 const vector<Mat> &tvecs,
                                 const Mat &cameraMatrix, const Mat &distCoeffs,
                                 vector<float> &perViewErrors) {

  vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); i++) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

void saveParams(Size imageSize, Size boardSize, float squareSize,
                const Mat &cameraMatrix, const Mat &distCoeffs,
                const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                const vector<float> &reprojErrs,
                const vector<vector<Point2f> > &imagePoints,
                double totalAvgErr) {
  FileStorage fs("calibration.yaml", FileStorage::WRITE);

  if (!rvecs.empty() || !reprojErrs.empty())
    fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << boardSize.width;
  fs << "board_height" << boardSize.height;
  fs << "square_size" << squareSize;
  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;
  if (!reprojErrs.empty())
    fs << "per_view_reprojection_errors" << Mat(reprojErrs);

  fs.release();
}

void calibrateandSave(Mat &view) {
  Size boardSize;
  Mat cameraMatrix, distCoeffs, viewGray;
  vector<Point2f> pointbuf;
  vector<Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  static vector<vector<Point2f> > imagePoints;

  double totalAvgErr = 0;
  float squareSize = 5; //圆点大小，任意设置的值
  static int image_count = 0;

  boardSize.width = 4;
  boardSize.height = 11;
  cameraMatrix = Mat::eye(3, 3, CV_64F);

  Size imageSize = view.size();
  bool result =
      findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
  if (result) {

    cvtColor(view, viewGray, COLOR_BGR2GRAY);
    cornerSubPix(
        viewGray, pointbuf, Size(5, 5), Size(-1, -1),
        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));
    imagePoints.push_back(pointbuf);
  }
  image_count++;

  ROS_INFO("Receive images:%d\n", image_count);

  if (image_count >= NUM_CIRCLEGRID) {
    image_count = 0;
    vector<vector<Point3f> > objectPoints(1);

    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, 0);

    totalAvgErr =
        computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs,
                                  cameraMatrix, distCoeffs, reprojErrs);

    saveParams(imageSize, boardSize, squareSize, cameraMatrix, distCoeffs,
               rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {

  try {

    Mat view = cv_bridge::toCvShare(msg, "bgr8")->image;
    calibrateandSave(view);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

bool startCalibration(camera_intrinsics::calibrate::Request &req,
                      camera_intrinsics::calibrate::Response &res) {

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub =
      it.subscribe("image_raw", 1000, imageCallback);

  res.creatTopic = true;
  ROS_INFO("Ready to calculate.");
  ros::spin();

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  ros::ServiceServer service =
      nh.advertiseService("/calibrate", startCalibration);
  ROS_INFO("Ready to listen.");
  ros::spin();

  return 0;
}