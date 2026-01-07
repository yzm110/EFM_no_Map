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
//CommonDataTyp和 CommonMathMethod 交叉 include
template <class T> struct Point2D ;
template <typename T> struct PointSL;
typedef PointSL<double> PointSLd;
////

using EFMPoint = Point2D<double>;
using EFMRefLinePoints = std::vector<EFMPoint>;
using EFMRefLinePointsSection = std::array<std::vector<EFMPoint>, 2>;

namespace CommonTool {
const double eps = 1e-6;
const double PI = acos(-1);

class DiscretePointsMath {
   public:
    static DiscretePointsMath* GetInstance();

    //输入离散的点，输出由离散点计算出的航向角，弧长，曲率和曲率变化率 
    bool ComputePathProfile(const EFMRefLinePoints& xy_points,
                                   std::vector<double>* headings, std::vector<double>* accumulated_s,
                                   std::vector<double>* kappas, std::vector<double>* dkappas);
    
    //笛卡尔坐标下，计算点 距离 离散点的线上的最近的点
    bool GetNearestPointOnline(double utm_x, double utm_y, const EFMRefLinePoints& ref_line_points,
                                       int& nearest_index);

    //笛卡尔坐标下，计算点tar_point，是否在point1和point2构成的box里
    bool IsPointInsideBodyCoordinate(EFMPoint point1, EFMPoint point2, EFMPoint tar_point);     
    
    //笛卡尔坐标下，计算点tar_point，是否在多边形里
    bool InPolygon(EFMPoint tar_point, EFMRefLinePoints polygon);
    bool PointInPolygon(EFMPoint tar_point, EFMRefLinePoints polygon);
    
    //计算点距离线的sl
    bool CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl);
    bool CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside);
    bool CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside, int& nearest_index);
    bool CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside, EFMPoint& point_front, EFMPoint& point_back, EFMPoint& proj_point_res);
    //计算点在线上的投影点
    bool GetProjectPointBodyCoordinate(EFMPoint tar_point, EFMRefLinePoints ref_line_points,
                                                       EFMPoint& proj_point, bool& is_inside, int& nearest_index, int& sign);
    
    // 将线段marker_raw 按照tar_point在线上的投影分成两段
    bool SaperateLineIntoTwoPart(const EFMRefLinePoints& marker_raw, EFMPoint tar_point,EFMRefLinePointsSection& marker_section);

    //计算线的长度
    double LineLength(const EFMRefLinePoints& line_raw);

    //对离散的线做等距处理，等距sample_distance
    bool FixPathDensity(const EFMRefLinePoints& raw_reference_line, double sample_distance,
                                    EFMRefLinePoints& fixed_reference_line);
    //延长，方向向量注意是单位向量
    template<class T>
    T ExtendPoint(const T& p, const T& direction, double extend_distance){
        return { p.x + direction.x * extend_distance, p.y + direction.y * extend_distance };
    }
    // 计算叉积 (AB × AC)
    template<class T>
    double crossProduct(const T &A, const T &B, const T &C) {
        return (B.x - A.x) * (C.y - A.y) - 
            (B.y - A.y) * (C.x - A.x);
    }

    // 判断点 C 是否在线段 AB 上
    template<class T>
    bool onSegment(const T &A, const T &B, const T &C) {
        if (std::min(A.x, B.x) <= C.x && C.x <= std::max(A.x, B.x) &&
            std::min(A.y, B.y) <= C.y && C.y <= std::max(A.y, B.y)) {
            return (crossProduct(A, B, C) == 0);
        }
        return false;
    }

    // 判断两条线段是否相交
    template<class T>
    bool segmentsIntersect(const T &A, const T &B, const T &C, const T &D) {
        double d1 = crossProduct(A, B, C);
        double d2 = crossProduct(A, B, D);
        double d3 = crossProduct(C, D, A);
        double d4 = crossProduct(C, D, B);

        // 快速排斥实验 + 跨立实验
        if (((d1 * d2) < 0) && ((d3 * d4) < 0)) {
            return true;
        }

        // 检查共线或端点重合的情况
        if (d1 == 0 && onSegment(A, B, C)) return true;
        if (d2 == 0 && onSegment(A, B, D)) return true;
        if (d3 == 0 && onSegment(C, D, A)) return true;
        if (d4 == 0 && onSegment(C, D, B)) return true;

        return false;
    }

    // 判断两条直线是否相交（不平行）
    template<class T>
    bool linesIntersect(const T &A, const T &B, const T &C, const T &D) {
        double ux = B.x - A.x;
        double uy = B.y - A.y;
        double vx = D.x - C.x;
        double vy = D.y - C.y;

        return (ux * vy - uy * vx) != 0;
    }

    // 计算两条直线的交点（如果存在）
    template<class T>
    bool computeIntersection(const T &A, const T &B, const T &C, const T &D, T& res) {
        double a1 = B.y - A.y;
        double b1 = A.x - B.x;
        double c1 = a1 * A.x + b1 * A.y;

        double a2 = D.y - C.y;
        double b2 = C.x - D.x;
        double c2 = a2 * C.x + b2 * C.y;

        double determinant = a1 * b2 - a2 * b1;

        if (determinant == 0) {
            return false; // 平行或重合，无交点
        } else {
            double x = (b2 * c1 - b1 * c2) / determinant;
            double y = (a1 * c2 - a2 * c1) / determinant;
            res = {x, y};
            return true;
        }
    }
};

}  // namespace CommonMathMethod
