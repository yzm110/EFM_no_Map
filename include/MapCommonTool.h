#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "CommonDataType.h"
#include "CommonMathMethod.h"
#include "CompileConfig.h"
#include "CoordinateTool.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"

namespace efm {
using MapStaticInfo = earth::shell::framework::TopicTrait::MapMapMsg;
using MapRouteList = earth::shell::framework::TopicTrait::MapRouteListMsg;
using MapPosition = earth::shell::framework::TopicTrait::MapPositionMsg;
using MapPositionTmp = message::map_position::s_Position_t;
using MapSwitchInfo = earth::shell::framework::TopicTrait::MapSwitchInfoMsg;
using MapGlobalData = earth::shell::framework::TopicTrait::MapGlobalDataMsg;
using MapDynamic = earth::shell::framework::TopicTrait::MapDynamicMsg;
using ComInfo = earth::shell::framework::TopicTrait::ComInfo;
using TrafficInfo = earth::shell::framework::TopicTrait::TrafficInfoMsg;
using IndexMap = std::unordered_map<uint32_t, int>;
using ConnectMap = std::unordered_map<uint32_t, std::vector<int>>;
class MapCommonTool {
   public:
    static MapCommonTool* GetInstance();

    /**
     * @brief 构建高精地图links映射表，便于后续遍历和取值以生成候选groups
     *
     * @return true
     * @return false
     */
    bool MakeIndex(const MapStaticInfo& map_static_info, const MapPosition& map_position_info,
                   const MapRouteList& map_route_list_info, MapRawDataMap& map_raw_data, uint64_t& errorcode);

    //在静态图中，找到link id为link_id 的 linkInfos
    //输入： 1- map_static_info，静态图指针；
    //      2- link_id,需要找的link id; 3-
    //      link_id_index_lane_info_map，first表示map_static_info_.LinkInfos.LinkInfos[i].InstanceId,
    //      second表示first里的i；
    //输出： link_Curvatures, 由link_id和lane_id找到的车道的曲率信息
    bool GetLinkInfos(const MapStaticInfo& map_static_info, const IndexMap& link_id_index_lane_info_map,
                      uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos);

    //在静态图中，找到具体link_id的第lane_id 的车道 laneInfo
    //输入： 1- map_static_info，静态图指针；
    //      2- link_id,需要找的link id; 3- 需要查找的lane id
    //输出： link_Curvatures, 由link_id和lane_id找到的车道的曲率信息
    bool GetLaneInfo(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, message::map_map::s_LaneInfo_t& lane);

    //在静态图中，找到具体link_id的第lane_id 的 车道的link_Curvatures
    //输入： 1- map_static_info，静态图指针；
    //      2- link_id,需要找的link id; 3- 需要查找的lane id
    //输出： link_Curvatures, 由link_id和lane_id找到的车道的曲率信息
    bool GetLaneCurvatures(std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info,
                           uint32_t link_id, uint8_t lane_id, message::map_map::s_LinkCurvature_t& link_Curvatures);

    //在静态图中，找到具体link_id的第lane_id 的车道的连接关系 lane_Connectivity
    //输入： 1- map_static_info，静态图指针； 2- link_id_index_lane_connect_map， first表示<map_static_info_.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId, second表示first里的i集合；
    //      3- link_id,需要找的link id; 4- 需要查找的lane id
    //输出： lane_Connectivity, link_id和lane_id车道的所有连接关系
    bool GetLaneFromConnectivitys(
        std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info,
        std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>> link_id_index_lane_connect_map,
        uint32_t link_id, uint8_t lane_id, std::vector<message::map_map::s_PairConnectivity_t>& lane_Connectivitys);

    // 获取车道线的line_id
    bool GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                           uint32_t& center_line_index);
    bool GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                          uint32_t& line_index);
    bool GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                         uint32_t& line_index);

    //找某一个line_id上的形点
    bool GetLineGeometry(const MapStaticInfo& map_static_info, uint32_t line_id,
                         std::vector<message::map_map::s_GeometryPoint_t>& geometry_points);
    bool GetLineType(const MapStaticInfo& map_static_info, const message::map_map::s_LinkInfo_t& link_infos,
                     uint8_t lane_id, uint8_t& line_type, bool is_left);
    void IsVirtuallyLine(const MapStaticInfo& map_msg, MapPositionTmp& map_position_tmp,
                         const message::map_map::s_LinkInfo_t& link_infos, const uint8_t& left_lane_id,
                         const uint8_t& right_lane_id, bool is_left, uint8_t& virtual_line_case);
    bool GetBackLinkLane(const MapStaticInfo& map_msg, const MapRawDataMap& map_raw_data_map,
                         MapPositionTmp& map_position_tmp, uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id,
                         uint32_t& back_lane_id);
    //在静态图中，找到具体连接到to_link_id的第lane_id 的车道的连接关系 lane_Connectivitys
    //输入： 1- map_static_info，静态图指针； 2- to_link_id_index_lane_connect_map， first表示<map_static_info_.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLinkId, second表示first里的i集合；
    //      3- link_id,需要找的link id; 4- 需要查找的lane id
    //输出： lane_Connectivity, 连接到to_link的lane id车道的所有连接关系
    bool GetLaneToConnectivitys(
        std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info,
        std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>> to_link_id_index_lane_connect_map,
        uint32_t link_id, uint8_t lane_id, std::vector<message::map_map::s_PairConnectivity_t>& lane_Connectivitys);
    
    //在静态图中，找到具体link_id的第lane_id 的车道的曲率信息
    //输入： 。。。
    //输出： 
    bool SaveLaneCurvInfo(const uint8_t& lane_id, const uint32_t& link_id, const std::unordered_map<uint64_t, int>& curve_index_map,
                                            const MapStaticInfo& map_static_info,std::vector<CurvPoint>& curvpoints);
                                            
    //在静态图中，找到具体line_index的线的信息
    //输入： 。。。
    //输出：                 
    bool GetLine(const uint32_t line_index, const MapStaticInfo& map_static_info, const std::unordered_map<uint32_t, int>& linear_obj_id_map,
                             std::vector<message::map_map::s_GeometryPoint_t>& geometry_points, uint8_t& mrk_type,uint8_t& mrk_color); 
    //在静态图中，找到具体line_index的线型                         
    bool GetLineType(const uint32_t line_index, const MapStaticInfo& map_static_info, const std::unordered_map<uint32_t, int>& linear_obj_id_map,uint8_t& mrk_type);

    //在静态图中，找到具体link_id的第lane_id 的车道的宽度信息
    bool GetLaneWidth(std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info, uint32_t link_id,
                           uint8_t lane_id, message::map_map::s_LaneWidth_t& lane_width);

    bool GetLaneSpeed(std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info, uint32_t link_id,
                             uint8_t lane_id, message::map_map::s_LaneSpeedLimit_t& lane_speed_limit);
    
    bool GetTwoLineDistance(const MapStaticInfo& map_static_info,uint32_t left_line, uint32_t right_line,
                                          double& start_distance, double& end_distance);
    bool GetLineGeometry(const MapStaticInfo& map_static_info,uint32_t line_id, std::vector<EFMPoint>& geometry_points);
};
}  // namespace efm
