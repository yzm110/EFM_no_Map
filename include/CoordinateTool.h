#pragma once
#include "CommonDataType.h"
#include "CompileConfig.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include <cmath>
#include <iostream>
#include <string>

#define EFM_MIN(x, y) ((x) < (y) ? (x) : (y))
#define EFM_MAX(x, y) ((x) > (y) ? (x) : (y))
#define FPRECISION 0.000001

// #include "status.h"

namespace CommonTool {
const double EARTH_RADIUS = 6378137.0;
constexpr double DEG_TO_RAD_LOCAL = 3.1415926535897932 / 180.0;

class CoordinateTool {
   public:
    static CoordinateTool* GetInstance();
    double NormalizeAngle(const double angle);
    void LineWGS2EC(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points, double ego_lat,
                    double ego_lon, double ego_heading, EFMRefLinePoints& linepts_ec);
    void LineWGS2UTM(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                     EFMRefLinePoints& linepts_utm);
    void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN);

    void UTM2EgoVehicle(double UTME, double UTMN, double ego_x, double ego_y, double ego_heading, double& res_x,
                        double& res_y);

    void LineUTM2EgoVehicle(const EFMRefLinePoints& linepts_utm, double ego_x, double ego_y, double ego_heading,
                            EFMRefLinePoints& linepts_ec);

    bool wgs84toUTM2(const double lat, const double lon, double& x, double& y, const double vehicle_lat,
                     const double vehicle_lon);
    void LineWGS2UTM2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                      EFMRefLinePoints& linepts_utm,
                      std::shared_ptr<const message::map_position::s_Position_t> vehicle_point);
    void LineWGS2UTM2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                      EFMRefLinePoints& linepts_utm,std::vector<uint32_t>& linepts_offset,
                      std::shared_ptr<const message::map_position::s_Position_t> vehicle_point,uint32_t link_path_offset);    
    bool blh2xyz(const double lon, const double lat, const double alt, double& x, double& y, double& z);
    bool calculateDistance(const double x0, const double y0, const double z0, const double x1, const double y1,
                           const double z1, double& distance);
    bool IsPointInside(const EFMPoint& p1, const EFMPoint& p2, const EFMPoint& p);
    bool CalcProjectionPoint(const EFMPoint& point1, const EFMPoint& point2, const EFMPoint& p,
                             EFMPoint& foot_point, bool& is_inside);
    bool GetNearestPoint(const std::vector<EFMPoint>& point_list, const EFMPoint& p, uint16_t& index,
                         EFMPoint& foot_point, double& distance, bool& find_inside);
    bool calculateDistanceGC02(const double x0, const double y0, const double z0, const double x1, const double y1,
                               const double z1, double& distance);
    bool crossProduct(const EFMPoint& p1, const EFMPoint& p2, double& cross);

    bool SetConvergenceAngle(double ego_wgs84_x, double ego_wgs84_y, float heading);

    bool WGS84ToBody(double lon, double lat, double ego_lon, double ego_lat, double heading, double& x, double& y);
    void LineWGS84ToBody(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                      EFMRefLinePoints& linepts_body,std::vector<uint32_t>& linepts_offset,
                      std::shared_ptr<const message::map_position::s_Position_t> vehicle_point,uint32_t link_path_offset, double ego_heading); 

    void LineWGS84ToBody(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_body,std::shared_ptr<const message::map_position::s_Position_t> vehicle_point, double ego_heading); 
    //重载1108
    void LineWGS84ToBody(const std::vector<EFMPoint>& geometry_points,
                                  EFMRefLinePoints& linepts_body,std::shared_ptr<const message::map_position::s_Position_t> vehicle_point, double ego_heading);
    void LineWGS84ToBodyV2(const std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                                  EFMRefLinePoints& linepts_body,const message::map_position::s_Position_t& vehicle_point, double ego_heading);                        
    bool GetNearestPointV2(const EFMRefLinePoints& point_list, const EFMPoint& p,  double& distance);
    double convergence_angle_;
};
}  // namespace CommonTool
