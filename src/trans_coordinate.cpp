#include "trans_coordinate.h"

namespace CommonTool {
CoordinateTool* CoordinateTool::GetInstance() {
    static CoordinateTool instance;
    return &instance;
}
// 朗歌提供，84直接转车身坐标
bool CoordinateTool::WGS84ToBody(const Point2Dd& point_wgs84, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_body) {
    static double DEG_TO_RAD = M_PI / 180.0;

    // 转换自车的经纬度为弧度
    double ego_lon = ego_pos_wgs84.x * DEG_TO_RAD;
    double ego_lat = ego_pos_wgs84.y * DEG_TO_RAD;

    // 转换目标点的经纬度为弧度
    double point_lon = point_wgs84.x * DEG_TO_RAD;
    double point_lat = point_wgs84.y * DEG_TO_RAD;

    // 计算必要的地球半径
    double tmp, sqrttmp, rm, rn;
    tmp = sin(ego_lat);
    tmp *= tmp;
    tmp = 1 - 0.00669437999013 * tmp;
    sqrttmp = sqrt(tmp);
    rm = 6378137.0 * (1.0 - 0.00669437999013) / (sqrttmp * tmp);
    rn = 6378137.0 / sqrttmp;

    double dn = (point_lat - ego_lat) * rm;
    double de = (point_lon - ego_lon) * rn * cos(ego_lat);

    point_body.x = dn * cos(ego_pos_wgs84.yaw) + de * sin(ego_pos_wgs84.yaw);
    point_body.y = dn * sin(ego_pos_wgs84.yaw) - de * cos(ego_pos_wgs84.yaw);

    return true;
}

bool CoordinateTool::LineWGS84ToBody(const std::vector<Point2Dd>& line_wgs84, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_body) {
    for (const auto& point : line_wgs84) {
        Point2Dd point_body;
        WGS84ToBody(point, ego_pos_wgs84, point_body);
        line_body.emplace_back(point_body);
    }
    return true;
}


bool CoordinateTool::BodyToWGS84(const Point2Dd& point_body, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_wgs84) {
    static double DEG_TO_RAD = M_PI / 180.0;
    static double RAD_TO_DEG = 180.0 / M_PI;

    // 转换自车的经纬度为弧度
    double ego_lon = ego_pos_wgs84.x * DEG_TO_RAD;
    double ego_lat = ego_pos_wgs84.y * DEG_TO_RAD;

    // 计算必要的地球半径
    double rm, rn, tmp, sqrttmp;
    tmp = sin(ego_lat);
    tmp *= tmp;
    tmp = 1 - 0.00669437999013 * tmp;
    sqrttmp = sqrt(tmp);
    rm = 6378137.0 * (1.0 - 0.00669437999013) / (sqrttmp * tmp);
    rn = 6378137.0 / sqrttmp;

    // 反向计算 dn 和 de
    double dn = (point_body.x * cos(ego_pos_wgs84.yaw) + point_body.y * sin(ego_pos_wgs84.yaw));
    double de = (point_body.x * sin(ego_pos_wgs84.yaw) - point_body.y * cos(ego_pos_wgs84.yaw));

    // 计算经纬度
    point_wgs84.y = ego_lat + dn / rm;  // 纬度加上偏移
    point_wgs84.x = ego_lon + de / (rn * cos(ego_lat));  // 经度加上偏移

    // 将结果转换为度
    point_wgs84.y *= RAD_TO_DEG;
    point_wgs84.x *= RAD_TO_DEG;

    // 确保经度在[-180, 180]范围内
    if (point_wgs84.x < -180.0) point_wgs84.x += 360.0;
    if (point_wgs84.x > 180.0) point_wgs84.x -= 360.0;


    return true;
}

bool CoordinateTool::LineBodyToWGS84(const std::vector<Point2Dd>& line_body, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_wgs84) {
    for (const auto& point : line_body) {
        Point2Dd point_wgs84;
        BodyToWGS84(point, ego_pos_wgs84, point_wgs84);
        line_wgs84.emplace_back(point_wgs84);
    }
    return true;
}

void CoordinateTool::CartesianToFrenet(const float rx, const float ry, const float rtheta,
                       const float x, const float y, float* const ptr_l) {
  const float dx = x - rx;
  const float dy = y - ry;
  const float cross_rd_nd = std::cos(rtheta) * dy - std::sin(rtheta) * dx;
  *ptr_l = std::copysign(static_cast<float>(hypot(dx, dy)), cross_rd_nd);  // 显式转换为 float
}

void CoordinateTool::CartesianToFrenet(const float rx, const float ry, const float rtheta,
                       const float rk, const float x, const float y,
                       const float theta, float* const ptr_l,
                       float* const ptr_dl_ds) {
  const float dx = x - rx;
  const float dy = y - ry;
  const float cross_rd_nd = std::cos(rtheta) * dy - std::sin(rtheta) * dx;
  *ptr_l = std::copysign(static_cast<float>(hypot(dx, dy)), cross_rd_nd);
  *ptr_dl_ds = (1 - rk * (*ptr_l)) * std::tan(theta - rtheta);
}

void CoordinateTool::CartesianToFrenet(const float rx, const float ry, const float rtheta,
                       const float rk, const float rdk, const float x,
                       const float y, const float theta, const float k,
                       float* const ptr_l, float* const ptr_dl_ds,
                       float* const ptr_ddl_ds) {
  const float dx = x - rx;
  const float dy = y - ry;
  const float cross_rd_nd = std::cos(rtheta) * dy - std::sin(rtheta) * dx;
  const float delta_theta = theta - rtheta;
  const float cos_delta_theta = std::cos(delta_theta);
  const float tan_delta_theta = std::cos(delta_theta);
  *ptr_l = std::copysign(static_cast<float>(hypot(dx, dy)), cross_rd_nd);
  const float one_minus_rk_l = 1 - rk * (*ptr_l);
  *ptr_dl_ds = one_minus_rk_l * tan_delta_theta;
  *ptr_ddl_ds = -(rdk * (*ptr_l) + rk * (*ptr_dl_ds)) * tan_delta_theta +
                one_minus_rk_l * (k * one_minus_rk_l / cos_delta_theta - rk) /
                    (cos_delta_theta * cos_delta_theta);
}

void CoordinateTool::CartesianToFrenet(const float rs, const float rx, const float ry,
                       const float rtheta, const float rkappa,
                       const float rdkappa, const float x, const float y,
                       const float v, const float a, const float theta,
                       const float kappa,
                       std::array<float, 3>* const ptr_s_condition,
                       std::array<float, 3>* const ptr_d_condition) {
  const float dx = x - rx;
  const float dy = y - ry;

  const float cos_theta_r = std::cos(rtheta);
  const float sin_theta_r = std::sin(rtheta);

  const float cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  ptr_d_condition->at(0) =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

  const float delta_theta = theta - rtheta;
  const float tan_delta_theta = std::tan(delta_theta);
  const float cos_delta_theta = std::cos(delta_theta);

  const float one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
  ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

  const float kappa_r_d_prime =
      rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

  ptr_d_condition->at(2) =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

  ptr_s_condition->at(0) = rs;

  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

  const float delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  ptr_s_condition->at(2) =
      (a * cos_delta_theta -
       ptr_s_condition->at(1) * ptr_s_condition->at(1) *
           (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
}

}

