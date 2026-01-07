#pragma once

#include <iostream>

#include "CommonDataType.h"
#include "CompileConfig.h"
#include "CoordinateTool.h"
#include "MapCommonTool.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"

#define CHANGE_POSITION_RANGE 25000     // cm
#define CHANGE_POSITION_RANGE0630 2000  // cm
#define CHANGE_POSITION_RANGE2 100000   // cm

namespace efm {
using MapMapMsg = earth::shell::framework::TopicTrait::MapMapMsg;
using MapRouteListMsg = earth::shell::framework::TopicTrait::MapRouteListMsg;
using MapPositionMsg = earth::shell::framework::TopicTrait::MapPositionMsg;
using MapPositionTmp = message::map_position::s_Position_t;
using MapSwitchInfoMsg = earth::shell::framework::TopicTrait::MapSwitchInfoMsg;
using MapGlobalDataMsg = earth::shell::framework::TopicTrait::MapGlobalDataMsg;
using MapDynamicMsg = earth::shell::framework::TopicTrait::MapDynamicMsg;
using ComInfo = earth::shell::framework::TopicTrait::ComInfo;
using TrafficInfoMsg = earth::shell::framework::TopicTrait::TrafficInfoMsg;
using EfmInfoMsg = earth::shell::framework::TopicTrait::EFMInfoMsg;
using MapLppInfoMsg = earth::shell::framework::TopicTrait::MapLppInfoMsg;
using MapLaneMsg = earth::shell::framework::TopicTrait::MapLaneMsg;
using RMFMapinfoDataclose = earth::shell::framework::TopicTrait::RMFMapinfoDataclose;
using EFMBusMapDataCloseInfo = earth::shell::framework::TopicTrait::EFMBusMapDataCloseInfo;
using IndexMap = std::unordered_map<uint32_t, int>;
using ConnectMap = std::unordered_map<uint32_t, std::vector<int>>;
class EfmInit {
   public:
    static EfmInit* GetInstance();

    bool InitVariable(const MapMapMsg& map_msg, const MapPositionMsg& position_msg,
                      const MapSwitchInfoMsg& switch_info_msg, const MapRouteListMsg& map_route_list,
                      const MapGlobalDataMsg& global_data_msg, const MapDynamicMsg& map_dynamic_msg,
                      const ComInfo& com_info, const TrafficInfoMsg& traffic_info, EfmInfoMsg& efm_info,
                      MapLppInfoMsg& map_lpp_info, MapLaneMsg& map_lane, RMFMapinfoDataclose& rmf_dc,
                      EFMBusMapDataCloseInfo& efm_dc, MapPositionTmp& map_position_tmp, MapRawData& map_raw_data);
    void InitErrorAndHead(const MapPositionMsg& position_msg, const MapRouteListMsg& map_route_list,
                          MainFuncStaticVar& main_func_static_var, uint64_t& errorcode, EfmInfoMsg& efm_info,
                          MapLaneMsg& map_lane, MapLppInfoMsg& map_lpp_info, RMFMapinfoDataclose& rmf_dc,
                          uint64_t efm_counter);
    bool CheckCurrentLinkIndex(const MapMapMsg& map_msg, const MapPositionTmp& map_position_tmp);
    bool CheckCurrentRoutingIndex(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                                  const MapRouteListMsg& map_route_list, int& cur_rp_index);
    bool InitVariableZTEXT(bool ztext_flag, const MapPositionTmp& map_position_tmp,
                           const MapRouteListMsg& map_route_list, MapLppInfoMsg& map_lpp_info);
    bool MakeErrorCode(uint64_t& errorcode, MainFuncStaticVar& main_func_static_var,const MapMapMsg& map_msg, const MapPositionMsg& position_msg,
                       const MapSwitchInfoMsg& switch_info_msg, const MapRouteListMsg& map_route_list,
                       const MapGlobalDataMsg& global_data_msg, const MapDynamicMsg& map_dynamic_msg,
                       EfmInfoMsg& efm_info);
    bool ChangePositionLaneID(const MapMapMsg& map_msg, const uint8_t& lane_change_state,
                              const PreCycleData& pre_cycle_data, MapPositionTmp& map_position_tmp,
                              const MapRawDataMap& map_raw_data_map, MainFuncStaticVar* main_func_static_var);
    bool GetVirtualMerge(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                         const MapRawDataMap& map_raw_data_map, MapPositionTmp& map_position_tmp,
                         const std::vector<uint32_t>& merge_back_links_id,
                         const std::vector<uint8_t>& merge_ego_back_lanes_id,
                         const std::vector<uint8_t>& merge_side_back_lanes_id, uint8_t& last_ego_lane_num);
    bool GetOneCenterLine(const MapMapMsg& map_msg, const uint8_t& lane_num,
                          const message::map_map::s_LinkInfo_t& link_infos, std::vector<EFMPoint>& center_line_points);
    bool GetSplitSideLaneV2(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                            MapPositionTmp& map_position_tmp, const MapRawDataMap& map_raw_data_map,
                            uint8_t& continue_lane_id, uint32_t& lanes_length, uint32_t& change_position_range,
                            uint32_t& virtual_line_split_link_id);

    bool GetSplitSideLaneV3(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                            MapPositionTmp& map_position_tmp, const MapRawDataMap& map_raw_data_map,
                            uint8_t& continue_lane_id, uint32_t& lanes_length);

    bool GetLaneTran(const PreCycleData& pre_cycle_data, message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id,
                     uint8_t& tran);
    bool GetSplitSideLane(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                          const MapRawDataMap& map_raw_data_map, uint32_t link_id, uint8_t lane_id,
                          std::vector<uint8_t>& split_side_lanes, uint32_t& lanes_length,
                          uint32_t& change_position_range);
    bool ErrorCodeLog(MainFuncStaticVar& main_func_static_var, uint64_t& errorcode);
    bool SetDataHead(EfmInfoMsg& efm_info, MapLaneMsg& map_lane, MapLppInfoMsg& map_lpp_info,
                     RMFMapinfoDataclose& rmf_dc, uint64_t efm_counter, uint64_t errorcode, uint64_t timestamp);
    bool MakeDataLossErrorCode(uint64_t& errorcode, MainFuncStaticVar& main_func_static_var, const MapMapMsg& map_msg,
                               const MapPositionMsg& position_msg, const MapSwitchInfoMsg& switch_info_msg,
                               const MapRouteListMsg& map_route_list, const MapGlobalDataMsg& global_data_msg,
                               const MapDynamicMsg& map_dynamic_msg);
};
}  // namespace efm
