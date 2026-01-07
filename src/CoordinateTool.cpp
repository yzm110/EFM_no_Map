/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2024-04-09 10:06:27
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-11 18:25:30
 * @FilePath: /repo83/code/dx11_noa/application/environmentmodelfunction/src/CoordinateTool.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "CoordinateTool.h"

namespace CommonTool {

CoordinateTool* CoordinateTool::GetInstance() {
    static CoordinateTool instance;
    return &instance;
}

double CoordinateTool::NormalizeAngle(const double angle) {
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

bool CoordinateTool::SetConvergenceAngle(double ego_wgs84_x, double ego_wgs84_y, float heading) {
    double proNo = (floor(ego_wgs84_x/3)) * 3 + 1.5;
    convergence_angle_ = ((sin((ego_wgs84_y*M_PI)/180.0) * (ego_wgs84_x - proNo))*M_PI)/180.0;

    return true;
}

void CoordinateTool::LineWGS2EC(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points, double ego_lat,
                                double ego_lon, double ego_heading, EFMRefLinePoints& linepts_ec) {
    linepts_ec.reserve(geometry_points.size());
    static double pointNED_x;
    static double pointNED_y;
    static double pointEC_x;
    static double pointEC_y;
    double trans_heading = ego_heading + M_PI;
    double cosH = cos(trans_heading);
    double sinH = sin(trans_heading);

    static double DEG_TO_RAD = M_PI / 180.0;
    static int Earth_R = 6378137;  // Earth radius, unit/m
    static double Earth_B = 0.0033527906953353;
    double veh_lati_rad = ego_lat * DEG_TO_RAD;
    double veh_long_rad = ego_lon * DEG_TO_RAD;
    double df_1 = sin(veh_lati_rad);
    double df_2 = cos(veh_lati_rad);
    double Rm = Earth_R * (1 - 2 * Earth_B + 3 * Earth_B * df_1 * df_1);
    double Rn_df2 = Earth_R * (1 + Earth_B * df_1 * df_1) * df_2;
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        // -------get the relative coordinate of the NED---------------
        // the north coordinate value of NED, x, unit/m
        pointNED_x = ((geometry_points[i].Latitude.Latitude * 360.0 / (4294967296)) * DEG_TO_RAD - veh_lati_rad) * Rm;
        // the east coordinate value of NED, y, unit/m
        pointNED_y =
            ((geometry_points[i].Longitude.Longitude * 360.0 / (4294967296)) * DEG_TO_RAD - veh_long_rad) * Rn_df2;
        // the down coordinate value  of NED, z, unit/m
        // pointNED.z = -linepts_wgs[i].altitude + ori_pt.altitude;

        // ------transform the coordinate of NED to vehicle coordinate--------
        // pointEC_x = pointNED_y * cosH - pointNED_x * sinH;
        // pointEC_y = pointNED_x * cosH + pointNED_y * sinH;
        pointEC_x = pointNED_x * cosH - pointNED_y * sinH;
        pointEC_y = pointNED_y * cosH - pointNED_x * sinH;
        // pointEC.offset = linepts_wgs[i].offset;
        // pointEC.slope = linepts_wgs[i].slope;
        // pointEC.Curve = linepts_wgs[i].Curve;
        point_temp.x = pointEC_x;
        point_temp.y = pointEC_y;
        linepts_ec.emplace_back(point_temp);
    }
}

void CoordinateTool::LineWGS2UTM(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                 EFMRefLinePoints& linepts_utm) {
    double lon = 0;
    double lat = 0;
    double utm_e = 0;  // x
    double utm_n = 0;  // y
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        // -------get the relative coordinate of the NED---------------
        // the north coordinate value of NED, x, unit/m
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        // the east coordinate value of NED, y, unit/m
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
        // the down coordinate value  of NED, z, unit/m
        // pointNED.z = -linepts_wgs[i].altitude + ori_pt.altitude;

        // ------transform the coordinate of NED to vehicle coordinate--------
        // pointEC_x = pointNED_y * cosH - pointNED_x * sinH;
        // pointEC_y = pointNED_x * cosH + pointNED_y * sinH;
        LonLat2UTM(lon, lat, utm_e, utm_n);
        // pointEC.offset = linepts_wgs[i].offset;
        // pointEC.slope = linepts_wgs[i].slope;
        // pointEC.Curve = linepts_wgs[i].Curve;
        point_temp.x = utm_e;
        point_temp.y = utm_n;
        linepts_utm.emplace_back(point_temp);
    }
}

void CoordinateTool::UTM2EgoVehicle(double UTME, double UTMN, double ego_x, double ego_y, double ego_heading,
                                    double& res_x, double& res_y) {
    double trans_heading = ego_heading + convergence_angle_ + M_PI;
    double delta_x = -ego_x + UTME;
    double delta_y = -ego_y + UTMN;
    double cosH = cos(trans_heading);
    double sinH = sin(trans_heading);
    res_x = -delta_x * sinH - delta_y * cosH;
    res_y = delta_x * cosH - delta_y * sinH;
}

void CoordinateTool::LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN) {
    double lon = longitude;
    double lat = latitude;
    // unit: km
    // variable
    double a = 6378.137;  // 赤道半径
    double e = 0.0818192;
    double k0 = 0.9996;
    double E0 = 500;
    double N0 = 0;
    // calc zoneNumber
    double zoneNumber = floor(lon / 6) + 31;
    // calc lambda0
    double lambda0 = (zoneNumber - 1) * 6 - 180 + 3;  // deg
    lambda0 = lambda0 * DEG_TO_RAD_LOCAL;             // radian
    // calc phi and lambda (lat and lon)
    double phi = lat * DEG_TO_RAD_LOCAL;
    double lambda = lon * DEG_TO_RAD_LOCAL;

    // Formula START
    double v = 1 / sqrt(1 - pow(e * sin(phi), 2));
    double A = (lambda - lambda0) * cos(phi);
    double T = pow(tan(phi), 2);
    double C = pow(e, 2) / (1 - pow(e, 2)) * pow(cos(phi), 2);
    double s = (1 - pow(e, 2) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256) * phi -
               (3 * pow(e, 2) / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024) * sin(2 * phi) +
               (15 * pow(e, 4) / 256 + 45 * pow(e, 6) / 1024) * sin(4 * phi) - 35 * pow(e, 6) / 3072 * sin(6 * phi);

    UTME = E0 + k0 * a * v * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T) * pow(A, 5) / 120);
    UTMN = N0 + k0 * a *
                    (s + v * tan(phi) *
                             (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 +
                              (61 - 58 * T + T * T) * pow(A, 6) / 720));

    UTME *= 1000;
    UTMN *= 1000;
}

void CoordinateTool::LineUTM2EgoVehicle(const EFMRefLinePoints& linepts_utm, double ego_x, double ego_y,
                                        double ego_heading, EFMRefLinePoints& linepts_ec) {
    linepts_ec.clear();
    double res_x, res_y;
    for (auto pt : linepts_utm) {
        UTM2EgoVehicle(pt.x, pt.y, ego_x, ego_y,  ego_heading, res_x, res_y);
        linepts_ec.push_back({res_x, res_y});
    }
}

void CoordinateTool::LineWGS2UTM2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_utm,
                                  std::shared_ptr<const message::map_position::s_Position_t> vehicle_point) {
    double lon = 0;
    double lat = 0;
    double utm_e = 0;  // x
    double utm_n = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point->Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point->Lon.Lon * 360.0 / (4294967296);

    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        // -------get the relative coordinate of the NED---------------
        // the north coordinate value of NED, x, unit/m
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        // the east coordinate value of NED, y, unit/m
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
        // the down coordinate value  of NED, z, unit/m
        // pointNED.z = -linepts_wgs[i].altitude + ori_pt.altitude;

        // ------transform the coordinate of NED to vehicle coordinate--------
        // pointEC_x = pointNED_y * cosH - pointNED_x * sinH;
        // pointEC_y = pointNED_x * cosH + pointNED_y * sinH;
        wgs84toUTM2(lat, lon, utm_e, utm_n, vehicle_latitude, vehicle_longitude);
        // pointEC.offset = linepts_wgs[i].offset;
        // pointEC.slope = linepts_wgs[i].slope;
        // pointEC.Curve = linepts_wgs[i].Curve;
        point_temp.x = utm_e;
        point_temp.y = utm_n;
        linepts_utm.emplace_back(point_temp);
    }
}

void CoordinateTool::LineWGS2UTM2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_utm,std::vector<uint32_t>& linepts_offset,
                                  std::shared_ptr<const message::map_position::s_Position_t> vehicle_point,
                                  uint32_t link_path_offset) {
    double lon = 0;
    double lat = 0;
    double utm_e = 0;  // x
    double utm_n = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point->Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point->Lon.Lon * 360.0 / (4294967296);

    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        // -------get the relative coordinate of the NED---------------
        // the north coordinate value of NED, x, unit/m
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        // the east coordinate value of NED, y, unit/m
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
        // the down coordinate value  of NED, z, unit/m
        // pointNED.z = -linepts_wgs[i].altitude + ori_pt.altitude;

        // ------transform the coordinate of NED to vehicle coordinate--------
        // pointEC_x = pointNED_y * cosH - pointNED_x * sinH;
        // pointEC_y = pointNED_x * cosH + pointNED_y * sinH;
        wgs84toUTM2(lat, lon, utm_e, utm_n, vehicle_latitude, vehicle_longitude);
        // pointEC.offset = linepts_wgs[i].offset;
        // pointEC.slope = linepts_wgs[i].slope;
        // pointEC.Curve = linepts_wgs[i].Curve;
        point_temp.x = utm_e;
        point_temp.y = utm_n;
        if(i == 0){
            linepts_offset.push_back(link_path_offset);
        }else{
            if(linepts_offset.size()>0){
                double dist = pow(point_temp.x - linepts_utm.back().x, 2) + pow(point_temp.y - linepts_utm.back().y, 2);
                dist = sqrt(dist);
                uint32_t offset_tmp = linepts_offset.back() + static_cast<uint32_t>(dist*100);
                linepts_offset.push_back(offset_tmp);
            }
        }
        linepts_utm.emplace_back(point_temp);
    }
}

bool CoordinateTool::wgs84toUTM2(const double lat, const double lon, double& x, double& y, const double vehicle_lat,
                                 const double vehicle_lon) {
    double latitude = lat;                   /// (pow(2, 32)) * 360;
    double longitude = lon;                  /// (pow(2, 32)) * 360;
    double vehicle_latitude = vehicle_lat;   /// (pow(2, 32)) * 360;
    double vehicle_longitude = vehicle_lon;  /// (pow(2, 32)) * 360;

    int ProjNo = 0;
    int ZoneWide;  //带宽
    double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
    double a, f, e2, ee, NN, T, C, A, M, iPI;
    iPI = 0.0174532925199433;  // 3.1415926535898/180.0;
    ZoneWide = 3;              // 3度带宽
    // ZoneWide = 6; 6度带宽
    // a=6378245.0; f=1.0/298.3; //54年北京坐标系参数
    // a=6378140.0; f=1/298.257; //80年西安坐标系参数
    a = 6378137.0;
    f = 1.0 / 298.257223563;  // WGS84坐标系参数
    // ProjNo = (int)(longitude / ZoneWide) ;      //6度带
    // longitude0 = ProjNo * ZoneWide + ZoneWide / 2; //6度带
    ProjNo = (int)(vehicle_longitude / ZoneWide + 0.5);
    // ProjNo = (int)(longitude / ZoneWide) ; //--带号
    longitude0 = ProjNo * ZoneWide;  //--中央子午线
    longitude0 = longitude0 * iPI;   //--中央子午线转化为弧度
    latitude0 = 0;
    longitude1 = longitude * iPI;  //经度转换为弧度
    latitude1 = latitude * iPI;    //纬度转换为弧度
    e2 = 2 * f - f * f;
    ee = e2 * (1.0 - e2);
    NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
    T = tan(latitude1) * tan(latitude1);
    C = ee * cos(latitude1) * cos(latitude1);
    A = (longitude1 - longitude0) * cos(latitude1);

    M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 -
             (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * latitude1) +
             (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) -
             (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));
    xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
    yval = M + NN * tan(latitude1) *
                   (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                    (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
    // X0 = 1000000L*(ProjNo+1)+500000L; //6度带
    X0 = 1000000L * ProjNo + 500000L;  // 3度带
    Y0 = 0;
    xval = xval + X0;
    yval = yval + Y0;

    x = xval;
    y = yval;
    // printf("%lf   %lf\r\n",xval,yval);
    return true;
}

bool CoordinateTool::blh2xyz(const double lon, const double lat, const double alt, double& x, double& y, double& z) {
    const double A_WGS84 = 6378137.0;
    const double F_WGS84 = 1 / 298.257223563;
    const double E2_WGS84 = (2 - F_WGS84) * F_WGS84;
    double lon_r = lon * M_PI / 180.0;
    double lat_r = lat * M_PI / 180.0;
    double sin_lat = sin(lat_r);
    double cos_lat = cos(lat_r);
    double N = A_WGS84 / sqrt(1 - E2_WGS84 * sin_lat * sin_lat);
    x = (alt + N) * cos_lat * cos(lon_r);
    y = (alt + N) * cos_lat * sin(lon_r);
    z = (N * (1 - E2_WGS84) + alt) * sin_lat;
    return true;
}

bool CoordinateTool::calculateDistance(const double x0, const double y0, const double z0, const double x1,
                                       const double y1, const double z1, double& distance) {
    distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    return true;
}

bool CoordinateTool::calculateDistanceGC02(const double x0, const double y0, const double z0, const double x1,
                                           const double y1, const double z1, double& distance) {
    double temp_x0 = 0.0;
    double temp_y0 = 0.0;
    double temp_z0 = 0.0;
    blh2xyz(x0, y0, z0, temp_x0, temp_y0, temp_z0);

    double temp_x1 = 0.0;
    double temp_y1 = 0.0;
    double temp_z1 = 0.0;
    blh2xyz(x1, y1, z1, temp_x1, temp_y1, temp_z1);

    calculateDistance(temp_x0, temp_y0, temp_z0, temp_x1, temp_y1, temp_z1, distance);

    return true;
}

bool CoordinateTool::IsPointInside(const EFMPoint& p1, const EFMPoint& p2, const EFMPoint& p) {
    double x_min = EFM_MIN(p1.x, p2.x);
    double x_max = EFM_MAX(p1.x, p2.x);
    double y_min = EFM_MIN(p1.y, p2.y);
    double y_max = EFM_MAX(p1.y, p2.y);

    bool is_inside = false;

    if (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max) {
        is_inside = true;
    }
    return is_inside;
}

bool CoordinateTool::CalcProjectionPoint(const EFMPoint& point1, const EFMPoint& point2, const EFMPoint& p,
                                         EFMPoint& foot_point, bool& is_inside) {
    double x1 = point1.x;
    double y1 = point1.y;
    double x2 = point2.x;
    double y2 = point2.y;
    double x0 = p.x;
    double y0 = p.y;

    double distance = 0.0;

    if (fabs(x1 - x2) < FPRECISION) {
        foot_point.x = x1;
        foot_point.y = y0;
        distance = fabs(x0 - x1);

        is_inside = IsPointInside(point1, point2, foot_point);

        return true;
    } else if (fabs(y1 - y2) < FPRECISION) {
        foot_point.x = x0;
        foot_point.y = y1;
        distance = fabs(y0 - y1);

        is_inside = IsPointInside(point1, point2, foot_point);
        return true;
    }

    // 斜率
    double k = (y2 - y1) / (x2 - x1);

    // ax+by+c=0 方程参数
    double a = k;
    double b = -1;
    double c = y1 - k * x1;

    // 点p到线段p1p2的垂直距离
    distance = fabs(a * x0 + b * y0 + c) / sqrt(a * a + b * b);

    // 垂足
    foot_point.x = (b * b * x0 - a * b * y0 - a * c) / (a * a + b * b);
    foot_point.y = (a * a * y0 - a * b * x0 - b * c) / (a * a + b * b);

    is_inside = IsPointInside(point1, point2, foot_point);

    return true;
}

bool CoordinateTool::GetNearestPoint(const std::vector<EFMPoint>& point_list, const EFMPoint& p, uint16_t& index,
                                     EFMPoint& foot_point, double& distance, bool& find_inside) {
    distance = 4294967295.0;
    EFMPoint temp_point;
    find_inside = false;
    if (point_list.size() < 2) {
        return false;
    }
    for (uint16_t i = 0; i < point_list.size() - 1; ++i) {
        bool is_inside = false;
        CalcProjectionPoint(point_list[i], point_list[i + 1], p, temp_point, is_inside);
        if (is_inside) {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double p_x = 0.0;
            double p_y = 0.0;
            double p_z = 0.0;
            double temp_dis = 0.0;
            blh2xyz(temp_point.x, temp_point.y, 0.0, x, y, z);
            blh2xyz(p.x, p.y, 0.0, p_x, p_y, p_z);
            calculateDistance(x, y, z, p_x, p_y, p_z, temp_dis);
            if (temp_dis < distance) {
                distance = temp_dis;
                index = i;
                foot_point = temp_point;
                find_inside = true;
            }
        } else {
            double start_dis = 0.0;
            double end_dis = 0.0;
            double x_start = 0.0;
            double y_start = 0.0;
            double z_start = 0.0;
            double x_end = 0.0;
            double y_end = 0.0;
            double z_end = 0.0;
            double p_x = 0.0;
            double p_y = 0.0;
            double p_z = 0.0;
            blh2xyz(p.x, p.y, 0.0, p_x, p_y, p_z);
            blh2xyz(point_list[i].x, point_list[i].y, 0.0, x_start, y_start, z_start);
            blh2xyz(point_list[i + 1].x, point_list[i + 1].y, 0.0, x_end, y_end, z_end);
            calculateDistance(x_start, y_start, z_start, p_x, p_y, p_z, start_dis);
            calculateDistance(x_end, y_end, z_end, p_x, p_y, p_z, end_dis);
            if (start_dis < distance) {
                distance = start_dis;
                foot_point = point_list[i];
                index = i;
                if (0 != index) {
                    find_inside = true;
                } else {
                    find_inside = false;
                }
            }

            if (end_dis < distance) {
                distance = end_dis;
                foot_point = point_list[i + 1];
                index = i + 1;
                if (point_list.size() - 1 != index) {
                    find_inside = true;
                } else {
                    find_inside = false;
                }
            }
        }
    }
    return true;
}

bool CoordinateTool::GetNearestPointV2(const EFMRefLinePoints& point_list, const EFMPoint& p,  double& distance) {
    distance = 4294967295.0;
    EFMPoint temp_point;
    if (point_list.size() < 2) {
        return false;
    }
    for (uint16_t i = 0; i < point_list.size() - 1; ++i) {
        bool is_inside = false;
        CalcProjectionPoint(point_list[i], point_list[i + 1], p, temp_point, is_inside);
        if (is_inside) {
            double temp_dis = 0.0;
            calculateDistance(temp_point.x, temp_point.y, 0.0, p.x, p.y, 0.0,temp_dis);
            if (temp_dis < distance) {
                distance = temp_dis;
            }
        } else {
            double start_dis = 0.0;
            double end_dis = 0.0;
            calculateDistance(point_list[i].x, point_list[i].y, 0.0, p.x, p.y, 0.0, start_dis);
            calculateDistance(point_list[i+1].x, point_list[i + 1].y, 0.0, p.x, p.y, 0.0, end_dis);
            if (start_dis < distance) {
                distance = start_dis;
            }

            if (end_dis < distance) {
                distance = end_dis;
            }
        }
    }
    return true;
}

// 计算向量叉积
bool CoordinateTool::crossProduct(const EFMPoint& p1, const EFMPoint& p2, double& cross) {
    cross = p1.x * p2.y - p1.y * p2.x;
    return true;
}

// 朗歌提供，84直接转车身坐标
bool CoordinateTool::WGS84ToBody(double lon, double lat, double ego_lon, double ego_lat, double heading, double& x, double& y) {
    static double DEG_TO_RAD = M_PI / 180.0;
    double pos[2] = {lat * DEG_TO_RAD,lon * DEG_TO_RAD};
    double xy[2] ={0,0};
    ego_lon = ego_lon *DEG_TO_RAD;
    ego_lat = ego_lat *DEG_TO_RAD;
    static double tmp, sqrttmp, rm, rn, de, dn;
    tmp = sin(ego_lat);
    tmp *= tmp;
    tmp = 1 - 0.00669437999013 * tmp;
    sqrttmp = sqrt(tmp);
    rm = 6378137.0 * (1.0 - 0.00669437999013) / (sqrttmp * tmp);
    rn = 6378137.0 / sqrttmp;
    dn = (pos[0] - ego_lat) * rm;
    de = (pos[1] - ego_lon) * rn * cos(ego_lat);
    xy[0] = dn * cos(heading) + de * sin(heading);
    xy[1] = dn * sin(heading) - de * cos(heading);
    x = xy[0];
    y = xy[1];
    return true;
}

void CoordinateTool::LineWGS84ToBody(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                      EFMRefLinePoints& linepts_body,std::vector<uint32_t>& linepts_offset,
                      std::shared_ptr<const message::map_position::s_Position_t> vehicle_point,uint32_t link_path_offset, double ego_heading){
    double lon = 0;
    double lat = 0;
    double body_x = 0;  // x
    double body_y = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point->Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point->Lon.Lon * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "vehicle_latitude: " << vehicle_latitude << ",vehicle_longitude: " << vehicle_longitude<<std::endl;
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "lat: " << lat << ",lon: " << lon<<std::endl;
        WGS84ToBody(lon, lat, vehicle_longitude, vehicle_latitude, ego_heading,body_x, body_y);
                        //         std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "body_x: " << body_x << ",body_y: " << body_y<<std::endl;
        point_temp.x = body_x;
        point_temp.y = body_y;
        if(i == 0){
            linepts_offset.push_back(link_path_offset);
        }else{
            if(linepts_offset.size()>0){
                double dist = pow(point_temp.x - linepts_body.back().x, 2) + pow(point_temp.y - linepts_body.back().y, 2);
                dist = sqrt(dist);
                uint32_t offset_tmp = linepts_offset.back() + static_cast<uint32_t>(dist*100);
                linepts_offset.push_back(offset_tmp);
            }
        }
        linepts_body.emplace_back(point_temp);
    }
}


void CoordinateTool::LineWGS84ToBody(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_body,std::shared_ptr<const message::map_position::s_Position_t> vehicle_point, double ego_heading){
    double lon = 0;
    double lat = 0;
    double body_x = 0;  // x
    double body_y = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point->Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point->Lon.Lon * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "vehicle_latitude: " << vehicle_latitude << ",vehicle_longitude: " << vehicle_longitude<<std::endl;
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "lat: " << lat << ",lon: " << lon<<std::endl;
        WGS84ToBody(lon, lat, vehicle_longitude, vehicle_latitude, ego_heading,body_x, body_y);
                        //         std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "body_x: " << body_x << ",body_y: " << body_y<<std::endl;
        point_temp.x = body_x;
        point_temp.y = body_y;
        linepts_body.emplace_back(point_temp);
    }
}

void CoordinateTool::LineWGS84ToBody(const std::vector<EFMPoint>& geometry_points,
                                  EFMRefLinePoints& linepts_body,std::shared_ptr<const message::map_position::s_Position_t> vehicle_point, double ego_heading){
    //点geometry_points 是经纬度，不需要再经过缩放
    double lon = 0;
    double lat = 0;
    double body_x = 0;  // x
    double body_y = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point->Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point->Lon.Lon * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "vehicle_latitude: " << vehicle_latitude << ",vehicle_longitude: " << vehicle_longitude<<std::endl;
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        lat = geometry_points[i].x;
        lon = geometry_points[i].y;
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "lat: " << lat << ",lon: " << lon<<std::endl;
        WGS84ToBody(lon, lat, vehicle_longitude, vehicle_latitude, ego_heading,body_x, body_y);
                        //         std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "body_x: " << body_x << ",body_y: " << body_y<<std::endl;
        point_temp.x = body_x;
        point_temp.y = body_y;
        linepts_body.emplace_back(point_temp);
    }
}

void CoordinateTool::LineWGS84ToBodyV2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_body,const message::map_position::s_Position_t& vehicle_point, double ego_heading){
    double lon = 0;
    double lat = 0;
    double body_x = 0;  // x
    double body_y = 0;  // y
    // double utm_z = 0;
    double vehicle_latitude = vehicle_point.Lat.Lat * 360.0 / (4294967296);
    double vehicle_longitude = vehicle_point.Lon.Lon * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "vehicle_latitude: " << vehicle_latitude << ",vehicle_longitude: " << vehicle_longitude<<std::endl;
    EFMPoint point_temp;
    for (size_t i = 0; i < geometry_points.size(); ++i) {
        lat = geometry_points[i].Latitude.Latitude * 360.0 / (4294967296);
        lon = geometry_points[i].Longitude.Longitude * 360.0 / (4294967296);
                        // std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "lat: " << lat << ",lon: " << lon<<std::endl;
        WGS84ToBody(lon, lat, vehicle_longitude, vehicle_latitude, ego_heading,body_x, body_y);
                        //         std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                        //   << "body_x: " << body_x << ",body_y: " << body_y<<std::endl;
        point_temp.x = body_x;
        point_temp.y = body_y;
        linepts_body.emplace_back(point_temp);
    }
}
}  // namespace CommonTool
