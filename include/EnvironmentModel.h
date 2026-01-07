#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "CandidateLanesModel.h"
#include "CompileConfig.h"
#include "ExtractRefLine.h"
#include "ScenarioJudgeModel.h"
#include "reference_line_provider.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#ifdef USE_PROJ4
#include "coordinate_convert_tool.h"
#endif
#include "CommonDataType.h"
#include "CoordinateTool.h"
// #include "Plot2D.h"
#include "calibration.h"
#include "data_to_geojson.h"
#include "efm_config.h"
#include "efm_init.h"
#include "efm_output.h"
// #include "reference_line_smoother.h"
#include "speed_limit.h"
#include "store/yaml.h"

#define SPLIT_BACK_LENGTH 40  // m

namespace earth {
namespace shell {
namespace framework {
using namespace mantle;
using MapMapMsg = earth::shell::framework::TopicTrait::MapMapMsg;
using MapPositionMsg = earth::shell::framework::TopicTrait::MapPositionMsg;
using MapSwitchInfoMsg = earth::shell::framework::TopicTrait::MapSwitchInfoMsg;
using MapRouteListMsg = earth::shell::framework::TopicTrait::MapRouteListMsg;
using MapGlobalDataMsg = earth::shell::framework::TopicTrait::MapGlobalDataMsg;
using MapDynamicMsg = earth::shell::framework::TopicTrait::MapDynamicMsg;
using ComInfo = earth::shell::framework::TopicTrait::ComInfo;
using TrafficInfoMsg = earth::shell::framework::TopicTrait::TrafficInfoMsg;
using EFMInfoMsg = earth::shell::framework::TopicTrait::EFMInfoMsg;
using MapLaneMsg = earth::shell::framework::TopicTrait::MapLaneMsg;
using MapLppInfoMsg = earth::shell::framework::TopicTrait::MapLppInfoMsg;
using RMFMapinfoDataclose = earth::shell::framework::TopicTrait::RMFMapinfoDataclose;
using EFMBusMapDataCloseInfo = earth::shell::framework::TopicTrait::EFMBusMapDataCloseInfo;

struct EnvironmentModel final : EFMCmpt {
   public:
    EnvironmentModel();
    virtual ~EnvironmentModel();
    runtime::RR<void> init(const char*) override { 
        EfmConfig("/efm_config.yaml");
        return runtime::RR<void>::Ok(); 
    };
    SimpleString name() const override { return "EnvironmentModel"; }
    uint8_t check(const MapMapMsg&, const MapPositionMsg&, const MapSwitchInfoMsg&, const MapRouteListMsg&,
                  const MapGlobalDataMsg&, const MapDynamicMsg&, const ComInfo&, const TrafficInfoMsg&,const typename TopicTrait::DeToMapData&) override {
        return 0;
    }

    runtime::RR<void> compute(const MapMapMsg&, const MapPositionMsg&, const MapSwitchInfoMsg&, const MapRouteListMsg&,
                              const MapGlobalDataMsg&, const MapDynamicMsg&, const ComInfo&, const TrafficInfoMsg&,const typename TopicTrait::DeToMapData&,
                              EFMInfoMsg&, MapLaneMsg&, MapLppInfoMsg&, RMFMapinfoDataclose&,
                              EFMBusMapDataCloseInfo&) override;

    const std::unordered_map<uint32_t, int>& get_link_id_index_lane_info_map();
    const std::unordered_map<uint32_t, std::vector<int>>& get_link_id_index_lane_connect_map();
    const std::unordered_map<uint32_t, std::vector<int>>& get_to_link_id_index_lane_connect_map();

   private:
    void GetPreCycleData();
    void SavePreCycleData();
    void GetSubModuleOutput();

    // get all lane tree, from car link, to head(for route list),  back lane(50m)
    bool MakeLaneTree(uint64_t& errorcode);

    // get ref lane ,ego lane and node[](node is turn left or turn right)  0-ego;1-left;2-right
    bool MakeAlgo(uint64_t& errorcode);

    // get center lane , car head 150m, car back 50m
    bool MakeCenterLane(uint64_t& errorcode, MapLaneMsg& map_lane, uint32_t& last_ego_first_link_id,
                        uint8_t& last_ego_first_lane_id, uint32_t& last_ego_second_link_id,
                        uint8_t& last_ego_sencond_lane_id, double& last_bounds_val, const uint8_t lane_decision,const uint8_t lane_change_state);

    // get side lane , car head 150m, car back 50m
    bool MakeSideLane(uint64_t& errorcode, MapLaneMsg& map_lane);

    // make guide info ,toll ot tunnel etc..
    bool MakeGuide(uint64_t& errorcode);

    bool GetLastRefInfo(uint32_t& last_ref_first_link_id, uint8_t& last_ref_first_lane_id,
                        uint32_t& last_ref_second_link_id, uint8_t& last_ref_sencond_lane_id);
    bool GetLastEgoInfo(uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                        uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id);

    bool MakeSideLaneLeft(uint64_t& errorcode, MapLaneMsg& map_lane);
    bool MakeSideLaneLeftLeft(uint64_t& errorcode, MapLaneMsg& map_lane);
    bool MakeSideLaneRight(uint64_t& errorcode, MapLaneMsg& map_lane);
    bool MakeSideLaneRightRight(uint64_t& errorcode, MapLaneMsg& map_lane);
    bool MakeSideLaneEgo(uint64_t& errorcode, MapLaneMsg& map_lane);

    // map
    bool FixPathDensity(const EFMRefLinePoints& raw_reference_line, EFMRefLinePoints& fixed_reference_line);

    bool GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool OutputOneCenterLane(const EFMRefLinePointsSection& line_section, message::efm::s_LaneModel_t& centerline);
    void CutCenterLine(const EFMRefLinePointsSection& line_section, EFMRefLinePointsSection& line_section_cut);
    bool ProcessCenterLine(const EFMRefLinePointsSection& line_section, EFMRefLinePointsSection& line_section_processed,
                           uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                           uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id,
                           double& last_bounds_val, const EFMRefLine& path);
    bool ProcessCenterLine(const EFMRefLinePointsSection& line_section, EFMRefLinePointsSection& line_section_processed,
                           const EFMRefLine& path);
    bool SetLastLinksBoundsVal(uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                               uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id,
                               double& last_bounds_val, double& cur_bounds_val);
    bool SetLastLinkBoundsVal(uint8_t& last_ego_lane_id, double& last_bounds_val, double& cur_bounds_val);
    bool OutputOneCenterLane0630(const EFMRefLinePointsSection& line_section, message::efm::s_Lane_t& lane);
    bool OutputOneSideLine0630(const EFMRefLineMarkrSection& line_marker_section,
                               const std::vector<LineMkrInfo_s>& line_infos,
                               const std::vector<LineMkrInfo_s>& back_link_line_infos,
                               struct message::efm::s_LineMkr_t& side_line, std::vector<double>& solid_line_plot_x,
                               std::vector<double>& solid_line_plot_y, std::vector<double>& dot_line_plot_x,
                               std::vector<double>& dot_line_plot_y, std::vector<double>& virtual_line_plot_x,
                               std::vector<double>& virtual_line_plot_y, uint8_t line_direction);
    uint8_t LineColorMapping(uint8_t color_internal);
    // 0630新增
    bool LinearInsertPoint(const EFMRefLinePoints& raw_reference_line, EFMRefLinePoints& line_out);
    void CenterLineNoSmooth(const EFMRefLinePointsSection& line_section, message::efm::s_Lane_t& lane);
    void CenterLineSmooth(const EFMRefLine& path, message::efm::s_Lane_t& lane);
    bool MakeEmergencyLane(MapLppInfoMsg& map_lpp_info);
    // input var
    std::shared_ptr<const MapSwitchInfoMsg> map_switch_info_;
    std::shared_ptr<message::map_position::s_Position_t> map_position_;
    std::shared_ptr<const MapRouteListMsg> map_route_list_;
    std::shared_ptr<const MapMapMsg> map_static_info_;
    std::shared_ptr<const message_common::c2c_message::s_EgoMotion2_t> ego_motion_;
    std::shared_ptr<const MapGlobalDataMsg> global_data_;
    std::shared_ptr<const MapDynamicMsg> dynamic_info_;
    std::shared_ptr<const TrafficInfoMsg> traffic_info_;
    std::shared_ptr<const ComInfo> com_info_;
    std::shared_ptr<const TopicTrait::DeToMapData> de_info_;
    std::shared_ptr<CDataToJson> data_to_json_;
    std::shared_ptr<CandidateLanesModel> candidate_lanes_model_;
    std::shared_ptr<ExtractRefLine> extract_ref_line_;
    std::shared_ptr<ScenarioJudgeModel> scenario_judge_model_;
    std::shared_ptr<noa::environmentmodelfunction::SpeedLimit> speed_limit_info_;

    MapRawDataMap map_raw_data_map_;
    MapRawData map_raw_data_;
    PreCycleData pre_cycle_data_;
    SubModuleOutput sub_module_output_;

    // <map_static_info_.LinkInfos.LinkInfos[i].InstanceId,// i>, laneInfos search here
    std::unordered_map<uint32_t, int> link_id_index_lane_info_map_;
    //<map_static_info_.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId,index_vec> , lane connect info
    // search here
    std::unordered_map<uint32_t, std::vector<int>> link_id_index_lane_connect_map_;
    //<map_static_info_.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLinkId,
    // index_vec> , lane connect info search here
    std::unordered_map<uint32_t, std::vector<int>> to_link_id_index_lane_connect_map_;
    std::vector<double> opt_kappas_, opt_accumulated_s_;
    int cur_rp_index_;  // index in map_route_list_.LinkIds.LinkIds[]
    static MainFuncStaticVar main_func_static_var_;
};

}  // namespace framework
}  // namespace shell
}  // namespace earth
