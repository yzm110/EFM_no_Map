#pragma once

#include <cstdint>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "CommonDataType.h"

namespace CommonTool {

class CoordinateTool{
    public:
    static CoordinateTool* GetInstance();

    //wgs84转自车坐标
    bool WGS84ToBody(const Point2Dd& point_wgs84, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_body);
    //线段 wgs84转自车坐标
    bool LineWGS84ToBody(const std::vector<Point2Dd>& line_wgs84, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_body);
    //自车转wgs84
    bool BodyToWGS84(const Point2Dd& point_body, const DoublePosePoint& ego_pos_wgs84, Point2Dd& point_wgs84);
    //线段 自车转wgs84
    bool LineBodyToWGS84(const std::vector<Point2Dd>& line_body, const DoublePosePoint& ego_pos_wgs84, std::vector<Point2Dd>& line_wgs84);

    void CartesianToFrenet(const float rx, const float ry, const float rtheta,
                        const float x, const float y, float* const ptr_l);
    void CartesianToFrenet(const float rx, const float ry, const float rtheta,
                        const float rk, const float x, const float y,
                        const float theta, float* const ptr_l,
                        float* const ptr_dl_ds);
    void CartesianToFrenet(const float rx, const float ry, const float rtheta,
                        const float rk, const float rdk, const float x,
                        const float y, const float theta, const float k,
                        float* const ptr_l, float* const ptr_dl_ds,
                        float* const ptr_ddl_ds);
    void CartesianToFrenet(const float rs, const float rx, const float ry,
                        const float rtheta, const float rkappa,
                        const float rdkappa, const float x, const float y,
                        const float v, const float a, const float theta,
                        const float kappa,
                        std::array<float, 3>* const ptr_s_condition,
                        std::array<float, 3>* const ptr_d_condition);
};


}

