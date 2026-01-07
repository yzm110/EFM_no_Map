/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-13 20:11:41
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2023-12-15 11:35:59
 * @FilePath: /repo/code/dx11_noa/application/environmentmodelfunction/include/CommonMathMethod.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <iostream>
#include "CommonDataType.h"
#include "CompileConfig.h"
#include "calibration.h"

namespace CommonMathMethod {
const double eps = 1e-6;
const double PI = acos(-1);

class DiscretePointsMath {
   public:
    static bool ComputePathProfile(const EFMRefLinePoints& xy_points,
                                   std::vector<double>* headings, std::vector<double>* accumulated_s,
                                   std::vector<double>* kappas, std::vector<double>* dkappas);

    static bool GetNearestPointOnline(double utm_x, double utm_y, const EFMRefLinePoints& ref_line_points,
                                       int& nearest_index);

    static bool GetProjectPointBodyCoordinate(EFMPoint tar_point, EFMRefLinePoints ref_line_points, 
                                         uint32_t postion_offset, const std::vector<uint32_t>& ref_line_points_offset,
                                         EFMPoint& proj_point, bool& is_inside,int& nearest_index);

    static bool IsPointInsideBodyCoordinate(EFMPoint point1, EFMPoint point2, EFMPoint tar_point);     

    static bool CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint point1, uint32_t postion_offset, const std::vector<uint32_t>& ref_line_points_offset,SLPoint& sl);    

    static bool GetNearestPointOnlineByOffset(EFMPoint tar_point,  uint32_t postion_offset, EFMRefLinePoints& ref_line_points, const std::vector<uint32_t>& ref_line_points_offset,
                                               int& nearest_index);
    static bool CalPointSLBodyCoordinateV2(EFMRefLinePoints ref_line_points, EFMPoint tar_point, SLPoint& sl);
    static bool GetProjectPointBodyCoordinateV2(EFMPoint tar_point, EFMRefLinePoints ref_line_points,
                                                       EFMPoint& proj_point, bool& is_inside, int& nearest_index);

    static bool InPolygon(EFMPoint tar_point, EFMRefLinePoints polygon);
};

}  // namespace CommonMathMethod
