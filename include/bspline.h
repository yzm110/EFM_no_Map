/***********************************************************
 * @file     bspline.h
 * @author   dizhao.jin
 * @date     2021-12-20
 * @brief    bspline fit
 * @version  1.0
 * Copyright © 2021 Horizon Robotics. All rights reserved.
 ***********************************************************/
#ifndef SSM_INCLUDE_SSM_OPTIMIZATION_BSPLINE_H_
#define SSM_INCLUDE_SSM_OPTIMIZATION_BSPLINE_H_
#pragma once
#include <algorithm>
#include "CommonDataType.h"
#include <vector>
namespace NoMapEFM {
class BSpline {
 public:
  BSpline() {}
  ~BSpline() {}
  /**
  * @brief three order B-spline interpolation
  * @param inputs, the input points
  * @param closed, if line is closed, such as a circle
  * @param outputs, the outputs points
  * @param stride, interpolation step, between [0, 1]
  */
  static void CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      bool closed,
                                      std::vector<Point2D<double>>& outputs,
                                      double stride = 0.01f);
  /**
  * @brief three order B-spline smooth, the outputs size is equal to inputs size
  * @param inputs, the inputs points
  * @param outputs, the output points
  * @param keep_end, if true, the B-spline will pass through the end points
  */
  static void CubicBSplineSmooth(std::vector<Point2D<double>>& inputs,
                                 std::vector<Point2D<double>>& outputs,
                                 bool keep_end = true);
  /**
  * @brief three order B-spline smooth, calculate curvature for each point,
  * the outputs size is equal to inputs size
  * @param inputs Inputs points
  * @param outputs Output points
  * @param curves Curvatures for each output point
  * @param keep_end If true, the B-spline will pass through the end points
  */
  static void CubicBSplineSmooth(std::vector<Point2D<double>>& inputs,
                                 std::vector<Point2D<double>>& outputs,
                                 std::vector<double>& curves,
                                 bool keep_end = true);

  /**
  * @brief three order B-spline interpolation, extra points are interpoloted
  * as
  * insert num
  * @param inputs, the inputs points
  * @param insert_num, interpolated points number between each two input
  * points,
  * the size shoule be inputs.size() - 1
  * @param outputs, the outputs points
  * @param keep_end, if true, the B-spline will pass through the end points
  */
  static void CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      std::vector<int>& insert_num,
                                      std::vector<Point2D<double>>& outputs,
                                      bool keep_end = true);
  /**
* @brief three order B-spline interpolation, extra points are interpoloted
* as
* insert num
* @param inputs, the inputs points
* @param insert_num, interpolated points number between each two input
* points,
* the size shoule be inputs.size() - 1
* @param outputs, the outputs points
* @param curves Curvatures
* @param keep_end, if true, the B-spline will pass through the end points
*/
  static void CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      std::vector<int>& insert_num,
                                      std::vector<Point2D<double>>& outputs,
                                      std::vector<double>& curves,
                                      bool keep_end = true);

 private:
  // base function for cubic b-spline
  // f
  static double cubic_f03(double t) {
    return (-t * t * t + 3.0f * t * t - 3.0f * t + 1) / 6.0f;
  }
  static double cubic_f13(double t) {
    return (3.0f * t * t * t - 6.0f * t * t + 4.0f) / 6.0f;
  }
  static double cubic_f23(double t) {
    return (-3.0f * t * t * t + 3.0f * t * t + 3.0f * t + 1.0f) / 6.0f;
  }
  static double cubic_f33(double t) {
    return (t * t * t) / 6.0f;
  }
  // df
  static double cubic_df03(double t) {
    return (-t * t + 2.0f * t - 1.0f) / 2.0f;
  }
  static double cubic_df13(double t) {
    return (3.0f* t * t - 4.0f * t) / 2.0f;
  }
  static double cubic_df23(double t) {
    return (-3.0f * t * t + 2.0f * t + 1.0f) / 2.0f;
  }
  static double cubic_df33(double t) {
    return (t * t) / 2.0f;
  }
  // ddf
  static double cubic_ddf03(double t) {
    return (-t + 1.0f);
  }
  static double cubic_ddf13(double t) {
    return (3.0f * t - 2.0f);
  }
  static double cubic_ddf23(double t) {
    return (-3.0f * t + 1.0f);
  }
  static double cubic_ddf33(double t) {
    return t;
  }
  // calculate curvature
  static double cubic_curvature(Point2D<double>& p0, Point2D<double>& p1,
                               Point2D<double>& p2, Point2D<double>& p3,
                               double t) {
    Point2D<double> dp = p0 * cubic_df03(t) + p1 * cubic_df13(t)
                        + p2 * cubic_df23(t) + p3 * cubic_df33(t);
    Point2D<double> ddp = p0 * cubic_ddf03(t) + p1 * cubic_ddf13(t)
                         + p2 * cubic_ddf23(t) + p3 * cubic_ddf33(t);
    double base = dp.x * dp.x + dp.y * dp.y;
    base = base * base * base;
    base = sqrt(base);
    double curve = fabsf(dp.x * ddp.y - ddp.x * dp.y) / (base + 1e-6f);
    return static_cast<double>(curve);
  }
  // calculate curvature using three points
  static double curvature(Point2D<double>& p1, Point2D<double>& p2,
                         Point2D<double>& p3) {
    double dis1 = sqrt((p1.x - p2.x) *(p1.x - p2.x) +
      (p1.y - p2.y) * (p1.y - p2.y));
    double dis2 = sqrt((p1.x - p3.x) *(p1.x - p3.x) +
      (p1.y - p3.y) * (p1.y - p3.y));
    double dis3 = sqrt((p2.x - p3.x) *(p2.x - p3.x) +
      (p2.y - p3.y) * (p2.y - p3.y));
    double dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
    double cosA = std::min(1.0f, fabsf(dis/(2 * dis1 * dis3)));
    double sinA = sqrt(1- cosA * cosA);
    double curve = sinA / (0.5f * dis2);
    return static_cast<double>(curve);
  }
};
}  // namespace HOBOT_NOA
#endif  // SSM_INCLUDE_SSM_OPTIMIZATION_BSPLINE_H_
