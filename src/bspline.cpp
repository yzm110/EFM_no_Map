#include "bspline.h"
namespace NoMapEFM {
void BSpline::CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      bool closed,
                                      std::vector<Point2D<double>>& outputs,
                                      double stride) {
  // ref :https://blog.csdn.net/cnmgbmsdn/article/details/108141194
  // P = w0 + w1 * t + w2 * t^2 + w3 * t^3, (0 <= w <= 1)
  // w0 = (P0 + 4P1 + P2) / 6.0f
  // w1 = -(P0 - P2) / 2.0f
  // w2 = (P0 - 2P1 + P2) / 2.0f
  // w3 = -(P0 - 3P1 + 3P2 - P3) / 6.0
  // P0~P3 are control points
  int n_size = static_cast<int>(inputs.size());
  if (n_size < 4) {
    outputs = inputs;
    return;
  }
  int c_size = closed ? n_size : n_size - 1;
  for (int i = 0; i < c_size; i++) {
    Point2D<double> xy[4];
    int idx0 = i;
    int idx1 = (i + 1) % n_size;
    int idx2 = (i + 2) % n_size;
    int idx3 = (i + 3) % n_size;
    xy[0] = (inputs[idx0] + 4.0f * inputs[idx1] + inputs[idx2]) / 6.0f;
    xy[1] = -(inputs[idx0] - inputs[idx2]) / 2.0f;
    xy[2] = (inputs[idx0] - 2.0f * inputs[idx1] + inputs[idx2]) / 2.0f;
    xy[3] = -(inputs[idx0] - 3.0f * inputs[idx1] + 3.0f * inputs[idx2]
              - inputs[idx3])
            / 6.0f;
    for (double t = 0.0f; t <= 1.0f; t += stride) {
      Point2D<double> new_pt(0.0f, 0.0f);
      for (int j = 0; j < 4; j++) {
        new_pt += xy[j] * pow(t, j);
      }
      outputs.push_back(new_pt);
    }
  }
}

void BSpline::CubicBSplineSmooth(std::vector<Point2D<double>>& inputs,
                                 std::vector<Point2D<double>>& outputs,
                                 bool keep_end) {
  // ref https://blog.csdn.net/jiangjjp2812/article/details/100176547
  int n_size = static_cast<int>(inputs.size());
  if (n_size < 4) {
    outputs = inputs;
    return;
  }
  outputs.clear();
  std::vector<Point2D<double>> tmp;
  if (keep_end) {  // if keep endpoints, another two fake points will be used
    tmp.resize(n_size + 2);
    for (int i = 0; i < n_size; ++i) {
      tmp[i + 1] = inputs[i];
    }
    tmp[0] = 2.0f * tmp[1] - tmp[2];
    tmp[n_size + 1] = 2.0f * tmp[n_size] - tmp[n_size - 1];
  } else {
    tmp.resize(n_size);
    tmp = inputs;
  }
  double t = 0.0f;
  for (int i = 0; i < n_size - 1; i++) {
    Point2D<double>& p0 = tmp[i];
    Point2D<double>& p1 = tmp[i + 1];
    Point2D<double>& p2 = tmp[i + 2];
    Point2D<double>& p3 = tmp[i + 3];
    if (i == n_size - 4) {
      t = 0.0f;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs.push_back(new_pt);
      t = 1.0f;
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs.push_back(new_pt);
    } else {
      t = 0.0f;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs.push_back(new_pt);
    }
  }
}

void BSpline::CubicBSplineSmooth(std::vector<Point2D<double>>& inputs,
                                 std::vector<Point2D<double>>& outputs,
                                 std::vector<double>& curves, bool keep_end) {
  // ref https://blog.csdn.net/jiangjjp2812/article/details/100176547
  int n_size = static_cast<int>(inputs.size());
  if (n_size < 4) {
    outputs = inputs;
    curves.resize(n_size, 0.0f);
    return;
  }
  outputs.resize(n_size);
  curves.resize(n_size);
  std::vector<Point2D<double>> tmp;
  if (keep_end) {  // if keep endpoints, another two fake points will be used
    tmp.resize(n_size + 2);
    for (int i = 0; i < n_size; ++i) {
      tmp[i + 1] = inputs[i];
    }
    tmp[0] = 2.0f * tmp[1] - tmp[2];
    tmp[n_size + 1] = 2.0f * tmp[n_size] - tmp[n_size - 1];
  } else {
    tmp.resize(n_size);
    tmp = inputs;
  }
  double t = 0.0f;
  for (int i = 0; i < n_size - 1; i++) {
    Point2D<double>& p0 = tmp[i];
    Point2D<double>& p1 = tmp[i + 1];
    Point2D<double>& p2 = tmp[i + 2];
    Point2D<double>& p3 = tmp[i + 3];
    if (i == n_size - 3) {
      double ts[3] = { 0.0f, 0.5f, 1.0f };
      for (int j = 0; j < 3; j++) {
        t = ts[j];
        Point2D<double> new_pt(0.0f, 0.0f);
        new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
                 + p3 * cubic_f33(t);
        outputs[i + j] = new_pt;
        curves[i + j] = cubic_curvature(p0, p1, p2, p3, t);
        // curves[i + j] = curvature(p0, p1, p2);
      }
      break;
    } else {
      t = 0.0f;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs[i] = new_pt;
      curves[i] = cubic_curvature(p0, p1, p2, p3, t);
      // curves[i] = curvature(p0, p1, p2);
    }
  }
}

void BSpline::CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      std::vector<int>& insert_num,
                                      std::vector<Point2D<double>>& outputs,
                                      bool keep_end) {
  // ref https://blog.csdn.net/jiangjjp2812/article/details/100176547
  int n_size = static_cast<int>(inputs.size());
  if (n_size < 4) {
    outputs = inputs;
    return;
  }
  std::vector<Point2D<double>> tmp;
  if (keep_end) {  // if keep endpoints, another two fake points will be used
    tmp.resize(n_size + 2);
    for (int i = 0; i < n_size; ++i) {
      tmp[i + 1] = inputs[i];
    }
    tmp[0] = 2.0f * tmp[1] - tmp[2];
    tmp[n_size + 1] = 2.0f * tmp[n_size] - tmp[n_size - 1];
  } else {
    tmp.resize(n_size);
    tmp = inputs;
  }
  // get insert num sum
  int insert_num_sum = 0.0f;
  for (auto value : insert_num) {
    insert_num_sum += value;
  }
  outputs.resize(n_size + insert_num_sum);
  double t = 0.0f;
  int totalnum = 0;
  for (int i = 0; i < n_size - 1; i++) {
    Point2D<double>& p0 = tmp[i];
    Point2D<double>& p1 = tmp[i + 1];
    Point2D<double>& p2 = tmp[i + 2];
    Point2D<double>& p3 = tmp[i + 3];
    double dt = 1.0f / (insert_num[i] + 1);
    for (int j = 0; j < insert_num[i] + 1; j++) {
      t = dt * j;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs[totalnum++] = new_pt;
    }
    if (i == n_size - 2) {
      t = 1.0f;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      outputs[totalnum++] = new_pt;
    }
  }
}

void BSpline::CubicBSplineInterpolate(std::vector<Point2D<double>>& inputs,
                                      std::vector<int>& insert_num,
                                      std::vector<Point2D<double>>& outputs,
                                      std::vector<double>& curves,
                                      bool keep_end) {
  // ref https://blog.csdn.net/jiangjjp2812/article/details/100176547
  int n_size = static_cast<int>(inputs.size());
  if (n_size < 4) {
    outputs = inputs;
    curves.resize(outputs.size(), 0.0f);
    return;
  }
  std::vector<Point2D<double>> tmp;
  if (keep_end) {  // if keep endpoints, another two fake points will be used
    tmp.resize(n_size + 2);
    for (int i = 0; i < n_size; ++i) {
      tmp[i + 1] = inputs[i];
    }
    tmp[0] = 2.0f * tmp[1] - tmp[2];
    tmp[n_size + 1] = 2.0f * tmp[n_size] - tmp[n_size - 1];
  } else {
    tmp.resize(n_size);
    tmp = inputs;
  }
  // get insert num sum
  int insert_num_sum = 0.0f;
  for (auto value : insert_num) {
    insert_num_sum += value;
  }
  outputs.resize(n_size + insert_num_sum);
  curves.resize(outputs.size(), 0.0f);
  double t = 0.0f;
  int totalnum = 0;
  for (int i = 0; i < n_size - 1; i++) {
    Point2D<double>& p0 = tmp[i];
    Point2D<double>& p1 = tmp[i + 1];
    Point2D<double>& p2 = tmp[i + 2];
    Point2D<double>& p3 = tmp[i + 3];
    double dt = 1.0f / (insert_num[i] + 1);
    for (int j = 0; j < insert_num[i] + 1; j++) {
      t = dt * j;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      curves[totalnum] = cubic_curvature(p0, p1, p2, p3, t);
      outputs[totalnum++] = new_pt;
    }
    if (i == n_size - 2) {
      t = 1.0f;
      Point2D<double> new_pt(0.0f, 0.0f);
      new_pt = p0 * cubic_f03(t) + p1 * cubic_f13(t) + p2 * cubic_f23(t)
               + p3 * cubic_f33(t);
      curves[totalnum] = cubic_curvature(p0, p1, p2, p3, t);
      outputs[totalnum++] = new_pt;
    }
  }
}
}  // namespace HOBOT_NOA
