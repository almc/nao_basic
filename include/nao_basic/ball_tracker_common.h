#ifndef BALL_TRACKER_COMMON_H_
#define BALL_TRACKER_COMMON_H_

#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

void ball_tracker_initialize();
void ball_tracker_finalize();
void setwindowSettings();
void setupBlobDetector();

std::pair<float, float> worldBallPosFromImgCoords(AL::ALMotionProxy motionProxy,
                                                  std::pair<int, int> ballPosCam,
                                                  int imgWidth, int imgHeight,
                                                  int camera);


std::pair<int, int> GetThresholdedImage(cv::Mat& img_BGR, cv::Mat& img_THR,
                                        int& HueLow, int& HueHigh,
                                        int& SatLow, int& SatHigh,
                                        int& ValueLow, int& ValueHigh,
                                        cv::Scalar Color, int *nr_pixels_ptr = 0,
                                        int HueLow2 = -1, int HueHigh2 = -1,
                                        int SatLow2 = -1, int SatHigh2 = -1,
                                        int ValueLow2 = -1, int ValueHigh2 = -1);

#endif
