#pragma once

#include <iostream>

#include "CommonDataType.h"
#include "CompileConfig.h"
#include "CoordinateTool.h"
#include "MapCommonTool.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include "CandidateLanesModel.h"

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
class EfmOutput {
   public:
    static EfmOutput* GetInstance();

    // output efm_info,  map_lane, map_lpp_info,  rmf_dc
    bool MakeOutput(const MapMapMsg& map_msg, const MapRouteListMsg& map_route_list,
                    const MapSwitchInfoMsg& switch_info_msg, MapPositionTmp& map_position_tmp,
                    const MapRawData map_raw_data, const MapRawDataMap& map_raw_data_map,
                    const SubModuleOutput& sub_module_output, EfmInfoMsg& efm_info, MapLaneMsg& map_lane,
                    MapLppInfoMsg& map_lpp_info, RMFMapinfoDataclose& rmf_dc, uint64_t& errorcode);
    bool OutputMapGlobalLine(const MapMapMsg& map_msg, const MapSwitchInfoMsg& switch_info_msg,
                             MapPositionTmp& map_position_tmp, const MapRawData map_raw_data,
                             MapLppInfoMsg& map_lpp_info);
    bool OutputKerbLine(const MapMapMsg& map_msg, const MapRouteListMsg& map_route_list,
                        MapPositionTmp& map_position_tmp, const SubModuleOutput& sub_module_output,
                        const MapRawData map_raw_data, const MapRawDataMap& map_raw_data_map, MapLaneMsg& map_lane);
    bool MakeOutputMapLppInfo(const MapMapMsg& map_msg, const MapSwitchInfoMsg& switch_info_msg,
                              const MapRawDataMap& map_raw_data_map, const SubModuleOutput& sub_module_output,
                              MapPositionTmp& map_position_tmp, MapLppInfoMsg& map_lpp_info, MapLaneMsg& map_lane);
    bool MakeOutputMapLppInfoSwitch(const MapSwitchInfoMsg& switch_info_msg, const SubModuleOutput& sub_module_output,
                                    MapLppInfoMsg& map_lpp_info);
    bool MakeOutputMapLppInfoOdd(const SubModuleOutput& sub_module_output, MapPositionTmp& map_position_tmp,
                                 MapLppInfoMsg& map_lpp_info);
    bool MakeOutputMapLppInfoRefPath(const MapMapMsg& map_msg, const SubModuleOutput& sub_module_output,
                                     const MapRawDataMap& map_raw_data_map, MapLaneMsg& map_lane,
                                     MapLppInfoMsg& map_lpp_info);
    bool MakeOutputMapLppInfoOther(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                                   const SubModuleOutput& sub_module_output, MapLppInfoMsg& map_lpp_info);
    bool MakeOutputMapEfmInfo(const SubModuleOutput& sub_module_output, EfmInfoMsg& efm_info);
    bool MakeOutputMapLane(MapPositionTmp& map_position_tmp, const SubModuleOutput& sub_module_output,
                           const EfmInfoMsg& efm_info, const MapRawData& map_raw_data, MapLaneMsg& map_lane);
    bool MakeOutputZTEXT(const EfmInfoMsg& efm_info, const MapLaneMsg& map_lane, const MapLppInfoMsg& map_lpp_info,
                         const RMFMapinfoDataclose& rmf_dc);
    bool SetOneElementExtra(MapPositionTmp& map_position_tmp,std::vector<LaneExtraInfo_s> lane_extra_info, std::vector<uint8_t> lane_num_vec, std::vector<uint32_t> link_id_vec, 
                            const std::map<uint64_t,SplitInfo_S>& split_info_map, const std::deque<uint64_t>& split_info_deque, message::efm::s_Lane_t& efm_lane_info);
    bool JudgeExitDirection(const MapMapMsg& map_msg, const SubModuleOutput& sub_module_output,
                            const MapRawDataMap& map_raw_data_map, bool& is_continue, bool& is_split);
    static bool compareOddDis(const OddDis& a, const OddDis& b);
};
}  // namespace efm
