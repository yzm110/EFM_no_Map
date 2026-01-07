#include "EnvironmentModel.h"
#ifdef __QNX__
#include "gptp/gptp.h"
#endif

std::array<uint8_t, 8> hist_camera_spd_limit_ = {0, 0, 0, 0, 0, 0, 0, 0};
std::vector<uint32_t> merge_attrbute_link_log = {104420485};
namespace earth {
namespace shell {
using namespace mantle;
namespace framework {
MainFuncStaticVar EnvironmentModel::main_func_static_var_;

EnvironmentModel::EnvironmentModel() : cur_rp_index_(-1) {
    extract_ref_line_ = std::make_shared<ExtractRefLine>();
    scenario_judge_model_ = std::make_shared<ScenarioJudgeModel>();
    candidate_lanes_model_ = std::make_shared<CandidateLanesModel>();
    speed_limit_info_ = std::make_shared<noa::environmentmodelfunction::SpeedLimit>();
    data_to_json_ = std::make_shared<CDataToJson>();
}

EnvironmentModel::~EnvironmentModel() {}

runtime::RR<void> EnvironmentModel::compute(const MapMapMsg& map_msg, const MapPositionMsg& position_msg,
                                            const MapSwitchInfoMsg& switch_info_msg,
                                            const MapRouteListMsg& map_route_list,
                                            const MapGlobalDataMsg& global_data_msg,
                                            const MapDynamicMsg& map_dynamic_msg, const ComInfo& com_info,
                                            const TrafficInfoMsg& traffic_info, const typename TopicTrait::DeToMapData& de_info,
                                            EFMInfoMsg& efm_info, MapLaneMsg& map_lane, MapLppInfoMsg& map_lpp_info,
                                            RMFMapinfoDataclose& rmf_dc, EFMBusMapDataCloseInfo& efm_dc) {
    switch_info_msg.get_name();
    global_data_msg.get_name();
    map_dynamic_msg.get_name();
    static uint64_t efm_counter = 0;
    efm_counter++;
    // std::cout << " " << std::endl;
    // std::cout << "position_counter: " << position_msg.header.cntr << std::endl;

    ZTEXT("EFM_INFO", "efm_counter: ", 10, 24, "efm_counter: {}", efm_counter);
    ZTEXT("EFM_INFO", "position_counter: ", 16, 21, "position_counter: {}", position_msg.header.cntr);
#ifdef EM_COUT
    std::cout << "EFM is running " << std::endl;
#endif

#ifdef DATA_TO_JSON
    data_to_json_->GenerateData(map_msg, position_msg, switch_info_msg, map_route_list, global_data_msg,
                                map_dynamic_msg, efm_counter);
#endif

#ifdef __QNX__
    auto efm_start_now = bsw::gptp_clock::now();
    auto efm_start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(efm_start_now.time_since_epoch());
#endif

    uint64_t efm_use_time_ns = 0, efm_input_use_time_ns = 0, efm_make_use_time_ns = 0,
             efm_make_MakeLaneTree_use_time_ns = 0, efm_make_MakeAlgo_use_time_ns = 0, efm_output_use_time_ns = 0,
             efm_output_MakeCenterLane_use_time_ns = 0, efm_output_MakeSideLane_use_time_ns = 0,
             efm_output_MakeGuide_use_time_ns = 0, efm_output_MakeOutput_use_time_ns = 0;

    // EfmConfig("/efm_config.yaml");

    // init out
    ego_motion_ = std::make_shared<const message_common::c2c_message::s_EgoMotion2_t>(com_info.OSM2_Bus_EgoMotionData);
    message::map_position::s_Position_t temp_position = position_msg.Position;
    map_position_ = std::make_shared<message::map_position::s_Position_t>(temp_position);
    map_switch_info_ = std::make_shared<const MapSwitchInfoMsg>(switch_info_msg);
    map_route_list_ = std::make_shared<const MapRouteListMsg>(map_route_list);
    map_static_info_ = std::make_shared<const MapMapMsg>(map_msg);
    global_data_ = std::make_shared<const MapGlobalDataMsg>(global_data_msg);
    dynamic_info_ = std::make_shared<const MapDynamicMsg>(map_dynamic_msg);
    traffic_info_ = std::make_shared<const TrafficInfoMsg>(traffic_info);
    com_info_ = std::make_shared<const ComInfo>(com_info);
    de_info_ = std::make_shared<const TopicTrait::DeToMapData>(de_info);

    uint64_t errorcode = 0;
    static uint32_t last_ref_first_link_id = 0, last_ref_second_link_id = 0, last_ego_first_link_id = 0,
                    last_ego_second_link_id = 0;
    static uint8_t last_ref_first_lane_id = 0, last_ref_sencond_lane_id = 0, last_ego_first_lane_id = 0,
                   last_ego_sencond_lane_id = 0;
    static double last_bounds_val = 0;

    GetPreCycleData();

#ifdef __QNX__
    auto efm_input_start_now = bsw::gptp_clock::now();
    auto efm_input_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_input_start_now.time_since_epoch());
#endif

    {  // input
        if (false == efm::EfmInit::GetInstance()->InitVariable(
                         *map_static_info_, position_msg, *map_switch_info_, *map_route_list_, *global_data_,
                         *dynamic_info_, *com_info_, *traffic_info_, efm_info, map_lpp_info, map_lane, rmf_dc, efm_dc,
                         *map_position_, map_raw_data_)) {
            errorcode = errorcode | EFM_CODE_EFM_NO_DATA;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            speed_limit_info_->init_global_var();
            scenario_judge_model_->InitGlobalVar();
            return runtime::RR<void>::Ok();
        }
        cur_rp_index_ = map_raw_data_.ego_linkid_routelist_idx;
        // get errorcode for static and logic
        efm::EfmInit::GetInstance()->MakeDataLossErrorCode(errorcode, main_func_static_var_, map_msg, position_msg,
                                                           switch_info_msg, map_route_list, global_data_msg,
                                                           map_dynamic_msg);
        efm::EfmInit::GetInstance()->MakeErrorCode(errorcode, main_func_static_var_, map_msg, position_msg,
                                                   switch_info_msg, map_route_list, global_data_msg, map_dynamic_msg,
                                                   efm_info);

        // make static map index,if static map no change, no need
        // TODO 根据map-map-counter判断是否需要重新创建索引;linerobject增加索引
        //std::cout << "map_msg counter: " << map_msg.header.cntr << std::endl;
        ZTEXT("EFM_INFO", "map_msg_cntr: ", -35, 17, "map_msg_cntr: {}", map_msg.header.cntr);
        ZTEXT("EFM_INFO", "re_navigate_cntr: ", -35, 14, "re_navigate_cntr: {}", map_raw_data_.re_navigate_map_map_cntr);
        if(map_msg.header.cntr != pre_cycle_data_.last_map_map_counter){
            map_raw_data_.re_navigate_map_map_cntr = 0;
            if (false == efm::MapCommonTool::GetInstance()->MakeIndex(map_msg, position_msg, map_route_list,
                                                                  map_raw_data_map_, errorcode)) {
                errorcode = errorcode | EFM_CODE_EFM_NO_DATA;
                efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
                speed_limit_info_->init_global_var();
                scenario_judge_model_->InitGlobalVar();
                // candidate_lanes_model_->InitGlobalVal();
                return runtime::RR<void>::Ok();
            }            
        }else if(switch_info_msg.NOAInfo.NavigationStatus.data != 2 ||map_raw_data_.re_navigate_map_map_cntr!=0){
            if(map_raw_data_.re_navigate_map_map_cntr == 0){
                map_raw_data_.re_navigate_map_map_cntr = map_msg.header.cntr;
            }
            map_raw_data_map_.link_id_index_lane_info_map.clear();
            map_raw_data_map_.link_id_index_lane_connect_map.clear();
            map_raw_data_map_.to_link_id_index_lane_connect_map.clear();
            map_raw_data_map_.route_list_link_ids.clear();
            map_raw_data_map_.linear_obj_id_map.clear();
            map_raw_data_map_.curve_index_map.clear();
            map_raw_data_map_.lane_widths_map.clear();
                errorcode = errorcode | EFM_CODE_EFM_NO_DATA;
                efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
                speed_limit_info_->init_global_var();
                scenario_judge_model_->InitGlobalVar();
                // candidate_lanes_model_->InitGlobalVal();
                return runtime::RR<void>::Ok();                                    
        }

        link_id_index_lane_info_map_ = map_raw_data_map_.link_id_index_lane_info_map;
        link_id_index_lane_connect_map_ = map_raw_data_map_.link_id_index_lane_connect_map;
        to_link_id_index_lane_connect_map_ = map_raw_data_map_.to_link_id_index_lane_connect_map;
    }
    if (false == efm::EfmInit::GetInstance()->ChangePositionLaneID(map_msg, com_info.OSM2_St_RainfallAmnt.data,
                                pre_cycle_data_, *map_position_, map_raw_data_map_, &main_func_static_var_)) {
        errorcode = errorcode | EFM_CODE_EHP_POSITION_ERROR;
        efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_, errorcode,
                                                      efm_info, map_lane, map_lpp_info, rmf_dc, efm_counter);
        speed_limit_info_->init_global_var();
        scenario_judge_model_->InitGlobalVar();
        // candidate_lanes_model_->InitGlobalVal();
        return runtime::RR<void>::Ok();
    }
#ifdef __QNX__
    auto efm_input_end_now = bsw::gptp_clock::now();
    auto efm_input_end_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_input_end_now.time_since_epoch());
    efm_input_use_time_ns = static_cast<uint64_t>(efm_input_end_time.count() - efm_input_start_time.count());
#endif

#ifdef __QNX__
    auto efm_make_start_now = bsw::gptp_clock::now();
    auto efm_make_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_start_now.time_since_epoch());
#endif
    {  // make
// get all lane tree, from car link, to head(for route list),  back lane(50m)
#ifdef __QNX__
        auto efm_make_MakeLaneTree_start_now = bsw::gptp_clock::now();
        auto efm_make_MakeLaneTree_start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_MakeLaneTree_start_now.time_since_epoch());
#endif

        if (false == MakeLaneTree(errorcode)) {
            errorcode = errorcode | EFM_CODE_EFM_LANE_TREE_ERROR;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            speed_limit_info_->init_global_var();
            scenario_judge_model_->InitGlobalVar();
            // candidate_lanes_model_->InitGlobalVal();
            return runtime::RR<void>::Ok();
        }
#ifdef __QNX__
        auto efm_make_MakeLaneTree_end_now = bsw::gptp_clock::now();
        auto efm_make_MakeLaneTree_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_MakeLaneTree_end_now.time_since_epoch());
        efm_make_MakeLaneTree_use_time_ns =
            static_cast<uint64_t>(efm_make_MakeLaneTree_end_time.count() - efm_make_MakeLaneTree_start_time.count());
#endif

// get ref lane ,ego lane and node[](node is turn left or turn right)  0-ego;1-left;2-right
#ifdef __QNX__
        auto efm_make_MakeAlgo_start_now = bsw::gptp_clock::now();
        auto efm_make_MakeAlgo_start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_MakeAlgo_start_now.time_since_epoch());
#endif
        if (false == MakeAlgo(errorcode)) {
            errorcode = errorcode | EFM_CODE_EFM_MAKE_ALGO_ERROR;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            speed_limit_info_->init_global_var();
            scenario_judge_model_->InitGlobalVar();
            // candidate_lanes_model_->InitGlobalVal();
            return runtime::RR<void>::Ok();
        }

        GetLastRefInfo(last_ref_first_link_id, last_ref_first_lane_id, last_ref_second_link_id,
                       last_ref_sencond_lane_id);

#ifdef __QNX__
        auto efm_make_MakeAlgo_end_now = bsw::gptp_clock::now();
        auto efm_make_MakeAlgo_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_MakeAlgo_end_now.time_since_epoch());
        efm_make_MakeAlgo_use_time_ns =
            static_cast<uint64_t>(efm_make_MakeAlgo_end_time.count() - efm_make_MakeAlgo_start_time.count());
#endif
    }
#ifdef __QNX__
    auto efm_make_end_now = bsw::gptp_clock::now();
    auto efm_make_end_time = std::chrono::duration_cast<std::chrono::nanoseconds>(efm_make_end_now.time_since_epoch());
    efm_make_use_time_ns = static_cast<uint64_t>(efm_make_end_time.count() - efm_make_start_time.count());
#endif

#ifdef __QNX__
    auto efm_output_start_now = bsw::gptp_clock::now();
    auto efm_output_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_start_now.time_since_epoch());
#endif
    {  // output
// get center lane , car head 150m, car back 50m
#ifdef __QNX__
        auto efm_output_MakeCenterLane_start_now = bsw::gptp_clock::now();
        auto efm_output_MakeCenterLane_start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            efm_output_MakeCenterLane_start_now.time_since_epoch());
#endif
        if (false == MakeCenterLane(errorcode, map_lane, last_ego_first_link_id, last_ego_first_lane_id,
                                    last_ego_second_link_id, last_ego_sencond_lane_id, last_bounds_val, de_info.lane_decision,com_info.OSM2_St_RainfallAmnt.data)) {
            errorcode = errorcode | EFM_CODE_EFM_MAKE_OUTPUT_ERROR;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            return runtime::RR<void>::Ok();
        }
        GetLastEgoInfo(last_ego_first_link_id, last_ego_first_lane_id, last_ego_second_link_id,
                       last_ego_sencond_lane_id);
#ifdef __QNX__
        auto efm_output_MakeCenterLane_end_now = bsw::gptp_clock::now();
        auto efm_output_MakeCenterLane_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeCenterLane_end_now.time_since_epoch());
        efm_output_MakeCenterLane_use_time_ns = static_cast<uint64_t>(efm_output_MakeCenterLane_end_time.count() -
                                                                      efm_output_MakeCenterLane_start_time.count());
#endif

// get side lane , car head 150m, car back 50m
#ifdef __QNX__
        auto efm_output_MakeSideLane_start_now = bsw::gptp_clock::now();
        auto efm_output_MakeSideLane_start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeSideLane_start_now.time_since_epoch());
#endif
        if (false == MakeSideLane(errorcode, map_lane)) {
            errorcode = errorcode | EFM_CODE_EFM_MAKE_OUTPUT_ERROR;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            // speed_limit_info_->init_global_var();
            return runtime::RR<void>::Ok();
        }
        // MakeEmergencyLane(map_lpp_info);
#ifdef __QNX__
        auto efm_output_MakeSideLane_end_now = bsw::gptp_clock::now();
        auto efm_output_MakeSideLane_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeSideLane_end_now.time_since_epoch());
        efm_output_MakeSideLane_use_time_ns = static_cast<uint64_t>(efm_output_MakeSideLane_end_time.count() -
                                                                    efm_output_MakeSideLane_start_time.count());
#endif

// make guide info ,toll or tunnel etc..
#ifdef __QNX__
        auto efm_output_MakeGuide_start_now = bsw::gptp_clock::now();
        auto efm_output_MakeGuide_start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeGuide_start_now.time_since_epoch());
#endif
        if (false == MakeGuide(errorcode)) {
            errorcode = errorcode | EFM_CODE_EFM_SPEED_LIMIT_ERROR;
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            speed_limit_info_->init_global_var();
            scenario_judge_model_->InitGlobalVar();
            return runtime::RR<void>::Ok();
        }
#ifdef __QNX__
        auto efm_output_MakeGuide_end_now = bsw::gptp_clock::now();
        auto efm_output_MakeGuide_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeGuide_end_now.time_since_epoch());
        efm_output_MakeGuide_use_time_ns =
            static_cast<uint64_t>(efm_output_MakeGuide_end_time.count() - efm_output_MakeGuide_start_time.count());
#endif

// make output
#ifdef __QNX__
        auto efm_output_MakeOutput_start_now = bsw::gptp_clock::now();
        auto efm_output_MakeOutput_start_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeOutput_start_now.time_since_epoch());
#endif
        GetSubModuleOutput();
        map_lpp_info.mRefPaths[3].dNodeOffset = static_cast<double>(speed_limit_info_->speed_limit_source());//落盘用
        if (false == efm::EfmOutput::GetInstance()->MakeOutput(map_msg, map_route_list, switch_info_msg, *map_position_,
                                                               map_raw_data_, map_raw_data_map_, sub_module_output_,
                                                               efm_info, map_lane, map_lpp_info, rmf_dc, errorcode)) {
            efm::EfmInit::GetInstance()->InitErrorAndHead(position_msg, *map_route_list_, main_func_static_var_,
                                                          errorcode, efm_info, map_lane, map_lpp_info, rmf_dc,
                                                          efm_counter);
            return runtime::RR<void>::Ok();
        }
#ifdef __QNX__
        auto efm_output_MakeOutput_end_now = bsw::gptp_clock::now();
        auto efm_output_MakeOutput_end_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_MakeOutput_end_now.time_since_epoch());
        efm_output_MakeOutput_use_time_ns =
            static_cast<uint64_t>(efm_output_MakeOutput_end_time.count() - efm_output_MakeOutput_start_time.count());
#endif
    }
#ifdef __QNX__
    auto efm_output_end_now = bsw::gptp_clock::now();
    auto efm_output_end_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_output_end_now.time_since_epoch());
    efm_output_use_time_ns = static_cast<uint64_t>(efm_output_end_time.count() - efm_output_start_time.count());
#endif
    efm::EfmInit::GetInstance()->SetDataHead(efm_info, map_lane, map_lpp_info, rmf_dc, efm_counter, errorcode,
                                             position_msg.header.timestamp);
    SavePreCycleData();
#ifdef __QNX__
    auto efm_end_now = bsw::gptp_clock::now();
    auto efm_end_time = std::chrono::duration_cast<std::chrono::nanoseconds>(efm_end_now.time_since_epoch());
    efm_use_time_ns = static_cast<uint64_t>(efm_end_time.count() - efm_start_time.count());
#endif
    ZTEXT("EFM_INFO", "bIsActive_new: ", -35, 20, "bIsActive_new: {}", map_lpp_info.bIsActive);
    ZTEXT("EFM_INFO", "efm_use_time_ms: ", 80, 45, "efm_use_time_ms: {}", efm_use_time_ns / 1000000);
    ZTEXT("EFM_INFO", "efm_use_time_ns: ", 80, 42, "efm_use_time_ns: {}", efm_use_time_ns);
    ZTEXT("EFM_INFO", "efm_input_use_time_ns: ", 80, 39, "efm_input_use_time_ns: {}", efm_input_use_time_ns);
    ZTEXT("EFM_INFO", "efm_make_use_time_ns: ", 80, 36, "efm_make_use_time_ns: {}", efm_make_use_time_ns);
    ZTEXT("EFM_INFO", "efm_output_use_time_ns: ", 80, 33, "efm_output_use_time_ns: {}", efm_output_use_time_ns);
#ifdef SPLIT_0930
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].lane_split[0].dir.data_: " << (int)map_lane.lanes[2].lane_split[0].dir.data_
              << " ,map_lane.lanes[2].lane_split[0].s_start: " << map_lane.lanes[2].lane_split[0].s_start
              << " ,map_lane.lanes[2].lane_split[0].s_end: " << map_lane.lanes[2].lane_split[0].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].lane_split[1].dir.data_: " << (int)map_lane.lanes[2].lane_split[1].dir.data_
              << " ,map_lane.lanes[2].lane_split[1].s_start: " << map_lane.lanes[2].lane_split[1].s_start
              << " ,map_lane.lanes[2].lane_split[1].s_end: " << map_lane.lanes[2].lane_split[1].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ,map_lane.lanes[2].lane_split[2].dir.data_: " << (int)map_lane.lanes[2].lane_split[2].dir.data_
              << " ,map_lane.lanes[2].lane_split[2].s_start: " << map_lane.lanes[2].lane_split[2].s_start
              << " ,map_lane.lanes[2].lane_split[2].s_end: " << map_lane.lanes[2].lane_split[2].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].lane_split[0].dir.data_: " << (int)map_lane.lanes[1].lane_split[0].dir.data_
              << " ,map_lane.lanes[1].lane_split[0].s_start: " << map_lane.lanes[1].lane_split[0].s_start
              << " ,map_lane.lanes[1].lane_split[0].s_end: " << map_lane.lanes[1].lane_split[0].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].lane_split[1].dir.data_: " << (int)map_lane.lanes[1].lane_split[1].dir.data_
              << " ,map_lane.lanes[1].lane_split[1].s_start: " << map_lane.lanes[1].lane_split[1].s_start
              << " ,map_lane.lanes[1].lane_split[1].s_end: " << map_lane.lanes[1].lane_split[1].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ,map_lane.lanes[1].lane_split[2].dir.data_: " << (int)map_lane.lanes[1].lane_split[2].dir.data_
              << " ,map_lane.lanes[1].lane_split[2].s_start: " << map_lane.lanes[1].lane_split[2].s_start
              << " ,map_lane.lanes[1].lane_split[2].s_end: " << map_lane.lanes[1].lane_split[2].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].lane_split[0].dir.data_: " << (int)map_lane.lanes[3].lane_split[0].dir.data_
              << " ,map_lane.lanes[3].lane_split[0].s_start: " << map_lane.lanes[3].lane_split[0].s_start
              << " ,map_lane.lanes[3].lane_split[0].s_end: " << map_lane.lanes[3].lane_split[0].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].lane_split[1].dir.data_: " << (int)map_lane.lanes[3].lane_split[1].dir.data_
              << " ,map_lane.lanes[3].lane_split[1].s_start: " << map_lane.lanes[3].lane_split[1].s_start
              << " ,map_lane.lanes[3].lane_split[1].s_end: " << map_lane.lanes[3].lane_split[1].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ,map_lane.lanes[3].lane_split[2].dir.data_: " << (int)map_lane.lanes[3].lane_split[2].dir.data_
              << " ,map_lane.lanes[3].lane_split[2].s_start: " << map_lane.lanes[3].lane_split[2].s_start
              << " ,map_lane.lanes[3].lane_split[2].s_end: " << map_lane.lanes[3].lane_split[2].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].lane_split[0].dir.data_: " << (int)map_lane.lanes[0].lane_split[0].dir.data_
              << " ,map_lane.lanes[0].lane_split[0].s_start: " << map_lane.lanes[0].lane_split[0].s_start
              << " ,map_lane.lanes[0].lane_split[0].s_end: " << map_lane.lanes[0].lane_split[0].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].lane_split[1].dir.data_: " << (int)map_lane.lanes[0].lane_split[1].dir.data_
              << " ,map_lane.lanes[0].lane_split[1].s_start: " << map_lane.lanes[0].lane_split[1].s_start
              << " ,map_lane.lanes[0].lane_split[1].s_end: " << map_lane.lanes[0].lane_split[1].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ,map_lane.lanes[0].lane_split[2].dir.data_: " << (int)map_lane.lanes[0].lane_split[2].dir.data_
              << " ,map_lane.lanes[0].lane_split[2].s_start: " << map_lane.lanes[0].lane_split[2].s_start
              << " ,map_lane.lanes[0].lane_split[2].s_end: " << map_lane.lanes[0].lane_split[2].s_end << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].lane_split[0].dir.data_: " << (int)map_lane.lanes[4].lane_split[0].dir.data_
              << " ,map_lane.lanes[4].lane_split[0].s_start: " << map_lane.lanes[4].lane_split[0].s_start
              << " ,map_lane.lanes[4].lane_split[0].s_end: " << map_lane.lanes[4].lane_split[0].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].lane_split[1].dir.data_: " << (int)map_lane.lanes[4].lane_split[1].dir.data_
              << " ,map_lane.lanes[4].lane_split[1].s_start: " << map_lane.lanes[4].lane_split[1].s_start
              << " ,map_lane.lanes[4].lane_split[1].s_end: " << map_lane.lanes[4].lane_split[1].s_end << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ,map_lane.lanes[4].lane_split[2].dir.data_: " << (int)map_lane.lanes[4].lane_split[2].dir.data_
              << " ,map_lane.lanes[4].lane_split[2].s_start: " << map_lane.lanes[4].lane_split[2].s_start
              << " ,map_lane.lanes[4].lane_split[2].s_end: " << map_lane.lanes[4].lane_split[2].s_end << std::endl;
#endif
    // std::cout << "end:!!!!!!!" << std::endl;
    return runtime::RR<void>::Ok();
}

bool EnvironmentModel::MakeLaneTree(uint64_t& errorcode) {
    if (false == candidate_lanes_model_->Execute(*(ego_motion_.get()), *(map_position_.get()), *(map_route_list_.get()),
                                                 *(map_static_info_.get()), get_link_id_index_lane_info_map(),
                                                 get_link_id_index_lane_connect_map(),
                                                 get_to_link_id_index_lane_connect_map(),map_raw_data_map_.curve_index_map,
                                                 map_raw_data_map_.linear_obj_id_map)) {
        return false;
    }

    return true;
}

bool EnvironmentModel::GetLastRefInfo(uint32_t& last_ref_first_link_id, uint8_t& last_ref_first_lane_id,
                                      uint32_t& last_ref_second_link_id, uint8_t& last_ref_sencond_lane_id) {
    std::vector<uint8_t> lane_num_vec{};
    std::vector<uint32_t> link_id_vec{};
    link_id_vec = candidate_lanes_model_->link_id_vec();
    switch (extract_ref_line_->prior_path_index()) {
        case 0:
            lane_num_vec = candidate_lanes_model_->ego_path_lane_num();
            break;
        case 1:
            lane_num_vec = candidate_lanes_model_->left_path_lane_num();
            break;
        case 2:
            lane_num_vec = candidate_lanes_model_->right_path_lane_num();
            break;
        case 3:
            lane_num_vec = candidate_lanes_model_->left_left_path_lane_num();
            break;
        case 4:
            lane_num_vec = candidate_lanes_model_->right_right_path_lane_num();
            break;
        default:
            lane_num_vec = candidate_lanes_model_->ego_path_lane_num();
            break;
    }

    if (link_id_vec.size() < lane_num_vec.size() || 0 == lane_num_vec.size() || 0 == link_id_vec.size()) {
        last_ref_first_link_id = 0;
        last_ref_first_lane_id = 0;
        last_ref_second_link_id = 0;
        last_ref_sencond_lane_id = 0;
    } else {
        last_ref_first_link_id = link_id_vec[0];
        last_ref_first_lane_id = lane_num_vec[0];
        if (lane_num_vec.size() >= 2 && link_id_vec.size() >= 2) {
            last_ref_second_link_id = link_id_vec[1];
            last_ref_sencond_lane_id = lane_num_vec[1];
        }
    }

    return true;
}

bool EnvironmentModel::GetLastEgoInfo(uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                                      uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id) {
    std::vector<uint8_t> lane_num_vec{};
    std::vector<uint32_t> link_id_vec{};
    link_id_vec = candidate_lanes_model_->link_id_vec();
    lane_num_vec = candidate_lanes_model_->ego_path_lane_num();
    if (link_id_vec.size() < lane_num_vec.size() || 0 == lane_num_vec.size() || 0 == link_id_vec.size()) {
        last_ego_first_link_id = 0;
        last_ego_first_lane_id = 0;
        last_ego_second_link_id = 0;
        last_ego_sencond_lane_id = 0;
    } else {
        last_ego_first_link_id = link_id_vec[0];
        last_ego_first_lane_id = lane_num_vec[0];
        if (lane_num_vec.size() >= 2 && link_id_vec.size() >= 2) {
            last_ego_second_link_id = link_id_vec[1];
            last_ego_sencond_lane_id = lane_num_vec[1];
        }
    }
    return true;
}

bool EnvironmentModel::MakeAlgo(uint64_t& errorcode) {
    if (false == extract_ref_line_->Execute(*(map_position_.get()), *(map_static_info_.get()),
                                            get_link_id_index_lane_info_map(), link_id_index_lane_connect_map_,
                                            map_raw_data_map_.linear_obj_id_map,map_raw_data_map_.curve_index_map,map_raw_data_map_.lane_widths_map,
                                            candidate_lanes_model_)) {
        return false;
    }

    if (false == scenario_judge_model_->Execute(
                     cur_rp_index_, candidate_lanes_model_, extract_ref_line_, *(map_position_.get()),
                     *(map_route_list_.get()), *(map_static_info_.get()), get_link_id_index_lane_info_map(),
                     get_link_id_index_lane_connect_map(), get_to_link_id_index_lane_connect_map())) {
        return false;
    }

    return true;
}

// bool EnvironmentModel::MakeEmergencyLane(MapLppInfoMsg& map_lpp_info){
//     std::vector<double> emer_right_right_solid_line_x{};
//     std::vector<double> emer_right_right_solid_line_y{};
//     std::vector<double> emer_right_right_dot_line_x{};
//     std::vector<double> emer_right_right_dot_line_y{};
//     std::vector<double> emer_right_right_vir_line_x{};
//     std::vector<double> emer_right_right_vir_line_y{}; 
//     std::vector<double> emer_right_left_solid_line_x{};
//     std::vector<double> emer_right_left_solid_line_y{};
//     std::vector<double> emer_right_left_dot_line_x{};
//     std::vector<double> emer_right_left_dot_line_y{};
//     std::vector<double> emer_right_left_vir_line_x{};
//     std::vector<double> emer_right_left_vir_line_y{};     
//     std::vector<double> emer_right_center_x{};
//     std::vector<double> emer_right_center_y{};   
//     std::vector<double> emer_left_right_solid_line_x{};
//     std::vector<double> emer_left_right_solid_line_y{};
//     std::vector<double> emer_left_right_dot_line_x{};
//     std::vector<double> emer_left_right_dot_line_y{};
//     std::vector<double> emer_left_right_vir_line_x{};
//     std::vector<double> emer_left_right_vir_line_y{};
//     std::vector<double> emer_left_left_solid_line_x{};
//     std::vector<double> emer_left_left_solid_line_y{};
//     std::vector<double> emer_left_left_dot_line_x{};
//     std::vector<double> emer_left_left_dot_line_y{};
//     std::vector<double> emer_left_left_vir_line_x{};
//     std::vector<double> emer_left_left_vir_line_y{};
//     std::vector<double> emer_left_center_x{};
//     std::vector<double> emer_left_center_y{};  
//     //right lane 
//     CenterLineNoSmooth(extract_ref_line_->emergency_right_path().linePointsSection, map_lpp_info.laneEmergencies[0]);
//     // std::cout << __FILE__ << "," << __LINE__ << ","<< "emergency_right_path: right"  << std::endl;
//     OutputOneSideLine0630(
//         extract_ref_line_->emergency_right_path().rightMarkrSection, extract_ref_line_->emergency_right_path().rightLineMkrInfos,
//         extract_ref_line_->emergency_right_path().back_link_right_line_infos, map_lpp_info.laneEmergencies[0].right_line,
//         emer_right_right_solid_line_x, emer_right_right_solid_line_y, emer_right_right_dot_line_x,
//         emer_right_right_dot_line_y, emer_right_right_vir_line_x, emer_right_right_vir_line_y, 1);
//     // std::cout << __FILE__ << "," << __LINE__ << ","<< "emergency_right_path: left"  << std::endl;
//     OutputOneSideLine0630(
//         extract_ref_line_->emergency_right_path().leftMarkrSection, extract_ref_line_->emergency_right_path().leftLineMkrInfos,
//         extract_ref_line_->emergency_right_path().back_link_left_line_infos, map_lpp_info.laneEmergencies[0].left_line,
//         emer_right_left_solid_line_x, emer_right_left_solid_line_y, emer_right_left_dot_line_x,
//         emer_right_left_dot_line_y, emer_right_left_vir_line_x, emer_right_left_vir_line_y, 1);
//     map_lpp_info.laneEmergencies[0].lane_merge[0].dir.data_ = 5;
//     map_lpp_info.laneEmergencies[0].lane_merge[0].s_end = static_cast<float>(candidate_lanes_model_->emergency_right_lane_dist());

//     //left lane
//     CenterLineNoSmooth(extract_ref_line_->emergency_left_path().linePointsSection, map_lpp_info.laneEmergencies[1]);
//     // std::cout << __FILE__ << "," << __LINE__ << ","<< "emergency_left_path: right"  << std::endl;
//     OutputOneSideLine0630(
//         extract_ref_line_->emergency_left_path().rightMarkrSection, extract_ref_line_->emergency_left_path().rightLineMkrInfos,
//         extract_ref_line_->emergency_left_path().back_link_right_line_infos, map_lpp_info.laneEmergencies[1].right_line,
//         emer_left_right_solid_line_x, emer_left_right_solid_line_y, emer_left_right_dot_line_x,
//         emer_left_right_dot_line_y, emer_left_right_vir_line_x, emer_left_right_vir_line_y, 0);

//     // std::cout << __FILE__ << "," << __LINE__ << ","<< "emergency_left_path: left"  << std::endl;
//     OutputOneSideLine0630(
//         extract_ref_line_->emergency_left_path().leftMarkrSection, extract_ref_line_->emergency_left_path().leftLineMkrInfos,
//         extract_ref_line_->emergency_left_path().back_link_left_line_infos, map_lpp_info.laneEmergencies[1].left_line,
//         emer_left_left_solid_line_x, emer_left_left_solid_line_y, emer_left_left_dot_line_x,
//         emer_left_left_dot_line_y, emer_left_left_vir_line_x, emer_left_left_vir_line_y, 0); 
//     map_lpp_info.laneEmergencies[1].lane_merge[0].dir.data_ = 2;
//     map_lpp_info.laneEmergencies[1].lane_merge[0].s_end = static_cast<float>(candidate_lanes_model_->emergency_left_lane_dist());

//     //plot
//     for (int i = 0; i < map_lpp_info.laneEmergencies[0].center_line.pntSize; i++) {
//         if (i % 4 == 0) {
//             emer_right_center_x.push_back(map_lpp_info.laneEmergencies[0].center_line.linePnt[i].dX);
//             emer_right_center_y.push_back(map_lpp_info.laneEmergencies[0].center_line.linePnt[i].dY);
//         }
//     }
//     for (int i = 0; i < map_lpp_info.laneEmergencies[1].center_line.pntSize; i++) {
//         if (i % 4 == 0) {
//             emer_left_center_x.push_back(map_lpp_info.laneEmergencies[1].center_line.linePnt[i].dX);
//             emer_left_center_y.push_back(map_lpp_info.laneEmergencies[1].center_line.linePnt[i].dY);
//         }
//     }

// #ifdef EM_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_center_x.size(): " << emer_right_center_x.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_center_y.size(): " << emer_right_center_y.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].enable_flag: " << map_lpp_info.laneEmergencies[0].enable_flag << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].right_line.bIsAvailable: " << map_lpp_info.laneEmergencies[0].right_line.bIsAvailable << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].right_line.pntSize: " << (int)map_lpp_info.laneEmergencies[0].right_line.pntSize << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].left_line.bIsAvailable: " << map_lpp_info.laneEmergencies[0].left_line.bIsAvailable << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].left_line.pntSize: " << (int)map_lpp_info.laneEmergencies[0].left_line.pntSize << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].center_line.pntSize: " << (int)map_lpp_info.laneEmergencies[0].center_line.pntSize << std::endl;

//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_left_center_x.size(): " << emer_left_center_x.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_left_center_y.size(): " << emer_left_center_y.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[1].enable_flag: " << map_lpp_info.laneEmergencies[1].enable_flag << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[1].right_line.bIsAvailable: " << map_lpp_info.laneEmergencies[1].right_line.bIsAvailable << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[1].center_line.pntSize: " << (int)map_lpp_info.laneEmergencies[1].center_line.pntSize << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_left_solid_line_x.size(): " << (int)emer_right_left_solid_line_x.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_left_solid_line_y.size(): " << (int)emer_right_left_solid_line_y.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_right_dot_line_x.size(): " << (int)emer_right_right_dot_line_x.size() << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "emer_right_right_dot_line_y.size(): " << (int)emer_right_right_dot_line_y.size() << std::endl;
//     std::cout<< std::cout.precision(12)<< __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[0].lane_merge[0].s_end " << map_lpp_info.laneEmergencies[0].lane_merge[0].s_end << std::endl;
//     std::cout<<std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
//               << "map_lpp_info.laneEmergencies[1].lane_merge[0].s_end " << map_lpp_info.laneEmergencies[1].lane_merge[0].s_end << std::endl;
// {
//     std::stringstream ss2;
//     std::stringstream ss;
//     for (int i = 0; i < emer_right_left_solid_line_x.size(); i++) {
//         ss2 << " " << emer_right_left_solid_line_x[i];
//     }
//     for (int i = 0; i < emer_right_left_solid_line_y.size(); i++) {
//         ss << " " << emer_right_left_solid_line_y[i];
//     }
//     std::cout << std::cout.precision(12) << " emr_right_mk_x=[ " << ss2.str() << " ]" << std::endl;
//     std::cout << std::cout.precision(12) << " emr_right_mk_y=[ " << ss.str() << " ]" << std::endl;  
// }
// #endif
//     ZPLOTXYF("EFM_INFO", "--black2", emer_right_center_x, emer_right_center_y);   
//     ZPLOTXYF("EFM_INFO", "--black2", emer_left_center_x, emer_left_center_y);  
//     // ZPLOTXYF("EFM_INFO", "-black4", emer_right_left_solid_line_x, emer_right_left_solid_line_y);
//     // ZPLOTXYF("EFM_INFO", "--black4", emer_right_left_dot_line_x, emer_right_left_dot_line_y);
//     return true;
// }

bool EnvironmentModel::MakeCenterLane(uint64_t& errorcode, MapLaneMsg& map_lane, uint32_t& last_ego_first_link_id,
                                      uint8_t& last_ego_first_lane_id, uint32_t& last_ego_second_link_id,
                                      uint8_t& last_ego_sencond_lane_id, double& last_bounds_val, const uint8_t lane_decision, const uint8_t lane_change_state) {
    int prior_index = extract_ref_line_->prior_path_index();  // 0-ego;1-left;2-right
    int lane_decision_index = static_cast<int>(lane_decision);
    int lane_change_status = static_cast<int>(lane_change_state);
    // three path
    std::vector<double> ego_plot_x{};          // for plot
    std::vector<double> ego_plot_y{};          // for plot
    std::vector<double> left_plot_x{};         // for plot
    std::vector<double> left_plot_y{};         // for plot
    std::vector<double> left_left_plot_x{};    // for plot
    std::vector<double> left_left_plot_y{};    // for plot
    std::vector<double> right_plot_x{};        // for plot
    std::vector<double> right_plot_y{};        // for plot
    std::vector<double> right_right_plot_x{};  // for plot
    std::vector<double> right_right_plot_y{};  // for plot
    uint64_t efm_centerline_use_time_ns = 0;
    ZTEXT("EFM_INFO", "p_smooth_line_num: ", 140, 76, "p_smooth_line_num: {}", p_smooth_line_num);
    ZTEXT("EFM_INFO", "de_lane_decision: ", 144, 70, "de_lane_decision: {}", lane_decision_index);
    ZTEXT("EFM_INFO", "lane_change_state: ", 144, 73, "lane_change_state: {}", lane_change_status);

#ifdef SMOOTH
    {
        std::stringstream ss;
        std::stringstream ss2, ss3;
        for (int i = 0; i < extract_ref_line_->ego_path().split_start_s_vec.size(); i++) {
            ss << " ,<" << extract_ref_line_->ego_path().split_start_s_vec[i].first << ","
               << extract_ref_line_->ego_path().split_start_s_vec[i].second << ">";
        }
        for (int i = 0; i < extract_ref_line_->left_path().split_start_s_vec.size(); i++) {
            ss2 << " ," << extract_ref_line_->left_path().split_start_s_vec[i].first << ","
                << extract_ref_line_->left_path().split_start_s_vec[i].second << ">";
        }
        for (int i = 0; i < extract_ref_line_->right_path().split_start_s_vec.size(); i++) {
            ss3 << " ," << extract_ref_line_->right_path().split_start_s_vec[i].first << ","
                << extract_ref_line_->right_path().split_start_s_vec[i].second << ">";
            ;
        }
        std::cout << std::cout.precision(12) << " ego_path().split_start_s_vec=[ " << ss.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " left_path().split_start_s_vec=[ " << ss2.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " right_path().split_start_s_vec=[ " << ss3.str() << " ]" << std::endl;
    }
    {
        std::stringstream ss;
        std::stringstream ss2, ss3;
        for (int i = 0; i < extract_ref_line_->ego_path().merge_end_s_vec.size(); i++) {
            ss << " ," << extract_ref_line_->ego_path().merge_end_s_vec[i].first << ","
               << extract_ref_line_->ego_path().merge_end_s_vec[i].second << ">";
            ;
        }
        for (int i = 0; i < extract_ref_line_->left_path().merge_end_s_vec.size(); i++) {
            ss2 << " ," << extract_ref_line_->left_path().merge_end_s_vec[i].first << ","
                << extract_ref_line_->left_path().merge_end_s_vec[i].second << ">";
            ;
            ;
        }
        for (int i = 0; i < extract_ref_line_->right_path().merge_end_s_vec.size(); i++) {
            ss3 << " ," << extract_ref_line_->right_path().merge_end_s_vec[i].first << ","
                << extract_ref_line_->right_path().merge_end_s_vec[i].second << ">";
            ;
            ;
            ;
        }
        std::cout << std::cout.precision(12) << " ego_path().merge_end_s_vec=[ " << ss.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " left_path().merge_end_s_vec=[ " << ss2.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " right_path().merge_end_s_vec=[ " << ss3.str() << " ]" << std::endl;
    }
#endif
    switch (p_smooth_line_num) {
        case 0: {
            CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
            CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
            CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
            CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
            CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            break;
        }
        case 1: {
            // ego line
            EFMRefLinePointsSection ego_linePointsSection_cut{};
            EFMRefLinePointsSection ego_linePointsSection_proc{};
            CutCenterLine(extract_ref_line_->ego_path().linePointsSection, ego_linePointsSection_cut);
            ProcessCenterLine(ego_linePointsSection_cut, ego_linePointsSection_proc, last_ego_first_link_id,
                              last_ego_first_lane_id, last_ego_second_link_id, last_ego_sencond_lane_id,
                              last_bounds_val, extract_ref_line_->ego_path());
            OutputOneCenterLane0630(ego_linePointsSection_proc, map_lane.lanes[2]);

            CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
            CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
            CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
            CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            break;
        }
        case 2: {
            // ego line
            CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);

            // left line
            if (prior_index == 1) {
                CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
            } else {
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
            }

            // right line
            if (prior_index == 2) {
                CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
            } else {
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
            }

            // left_left line
            if (prior_index == 3) {
                CenterLineSmooth(extract_ref_line_->left_left_path(), map_lane.lanes[0]);
            } else {
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
            }

            // right_right line
            if (prior_index == 4) {
                CenterLineSmooth(extract_ref_line_->right_right_path(), map_lane.lanes[4]);
            } else {
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            }
            break;
        }
        case 3: {
            CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);
            CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
            CenterLineSmooth(extract_ref_line_->left_left_path(), map_lane.lanes[0]);
            CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
            CenterLineSmooth(extract_ref_line_->right_right_path(), map_lane.lanes[4]);
            break;
        }
        case 99: {
            if (prior_index == 0) {
                // ego line
                CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);

                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            } else if (prior_index == 1) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);

                CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            } else if (prior_index == 2) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);

                CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
            } else if (prior_index == 3) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);

                CenterLineSmooth(extract_ref_line_->left_left_path(), map_lane.lanes[0]);
            } else if (prior_index == 4) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);

                CenterLineSmooth(extract_ref_line_->right_right_path(), map_lane.lanes[4]);
            } else {
                CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);

                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            }
            break;
        }
        case 100: {//decision告诉要平滑哪一条
            if (lane_decision_index == 0) {
                // ego line
                CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);

                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            } else if (lane_decision_index == 1) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);

                CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            } else if (lane_decision_index == 2) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);

                CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
            } else if (lane_decision_index == 3) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);

                CenterLineSmooth(extract_ref_line_->left_left_path(), map_lane.lanes[0]);
            } else if (lane_decision_index == 4) {
                CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);

                CenterLineSmooth(extract_ref_line_->right_right_path(), map_lane.lanes[4]);
            } else {
                CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);

                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            }
            break;
        }
        case 101: {//decision告诉要平滑哪一条,但是用变道状态信号
            int smooth_lane = 255;
            if (lane_change_status == 7 || lane_change_status == 8) {
                if (prior_index == 0) {
                    // ego line
                    CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);
                    CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                    CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                    CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                    CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                    smooth_lane = 2;
                } else if (prior_index == 1) {
                    CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                    CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
                    CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                    CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                    CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                    smooth_lane = 1;
                } else if (prior_index == 2) {
                    CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                    CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                    CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
                    CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                    CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                    smooth_lane = 3;
                } else if(prior_index == 3){
                    CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                    CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                    CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                    //判断距离目标车道距离，如果大于5m，那么平滑临车道
                    if(extract_ref_line_->left_left_path().linePointsSection[0].size()>0){
                        if(std::abs(extract_ref_line_->left_left_path().linePointsSection[0].at(0).y)>5){
                            CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
                            CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                            smooth_lane = 1;
                        }else{
                            CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                            CenterLineSmooth(extract_ref_line_->left_left_path(), map_lane.lanes[0]);
                            smooth_lane = 0;
                        }
                    }else{
                        CenterLineSmooth(extract_ref_line_->left_path(), map_lane.lanes[1]);
                        CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                        smooth_lane = 1;
                    }
                } else if(prior_index == 4){
                    CenterLineNoSmooth(extract_ref_line_->ego_path().linePointsSection, map_lane.lanes[2]);
                    CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                    CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                    //判断距离目标车道距离，如果大于5m，那么平滑临车道
                    if(extract_ref_line_->right_right_path().linePointsSection[0].size()>0){
                        if(std::abs(extract_ref_line_->right_right_path().linePointsSection[0].at(0).y)>5){
                            CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
                            CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                            smooth_lane = 3;
                        }else{
                            CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                            CenterLineSmooth(extract_ref_line_->right_right_path(), map_lane.lanes[4]);
                            smooth_lane = 4;
                        }
                    }else{
                        CenterLineSmooth(extract_ref_line_->right_path(), map_lane.lanes[3]);
                        CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                        smooth_lane = 3;
                    }
                } else {
                    CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);
                    CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                    CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                    CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                    CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                    smooth_lane = 2;
                }
            } else {
                //ego line
                CenterLineSmooth(extract_ref_line_->ego_path(), map_lane.lanes[2]);
                CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
                CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
                CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
                CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
                smooth_lane = 2;
            }
            ZTEXT("EFM_INFO", "smooth_lane: ", 80, 61, "smooth_lane: {}", smooth_lane);
            break;
        }
        default: {
            // ego line
            EFMRefLinePointsSection ego_linePointsSection_cut{};
            EFMRefLinePointsSection ego_linePointsSection_proc{};
            CutCenterLine(extract_ref_line_->ego_path().linePointsSection, ego_linePointsSection_cut);
            ProcessCenterLine(ego_linePointsSection_cut, ego_linePointsSection_proc, last_ego_first_link_id,
                              last_ego_first_lane_id, last_ego_second_link_id, last_ego_sencond_lane_id,
                              last_bounds_val, extract_ref_line_->ego_path());
            OutputOneCenterLane0630(ego_linePointsSection_proc, map_lane.lanes[2]);

            CenterLineNoSmooth(extract_ref_line_->left_path().linePointsSection, map_lane.lanes[1]);
            CenterLineNoSmooth(extract_ref_line_->right_path().linePointsSection, map_lane.lanes[3]);
            CenterLineNoSmooth(extract_ref_line_->left_left_path().linePointsSection, map_lane.lanes[0]);
            CenterLineNoSmooth(extract_ref_line_->right_right_path().linePointsSection, map_lane.lanes[4]);
            break;
        }
    }

#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " map_lane.lanes[2].center_line.pntSize:: " << (int)map_lane.lanes[2].center_line.pntSize << std::endl;
#endif
#ifdef SMOOTH
    {
        std::stringstream ss3;
        std::stringstream ss4;
        for (int i = 0; i < map_lane.lanes[2].center_line.pntSize; i++) {
            ss3 << " " << map_lane.lanes[2].center_line.linePnt[i].dX;
            ss4 << " " << map_lane.lanes[2].center_line.linePnt[i].dY;
        }
        std::cout << std::cout.precision(12) << " ego_cl_out_x=[ " << ss3.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ego_cl_out_y=[ " << ss4.str() << " ]" << std::endl;
        std::stringstream ss5;
        std::stringstream ss6;       
        for(auto mkr:extract_ref_line_->ego_path().leftMarkr){
            ss5 << " " << mkr.x;
            ss6 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " ego_leline_out_x=[ " << ss5.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ego_leline_out_y=[ " << ss6.str() << " ]" << std::endl;
        std::stringstream ss7;
        std::stringstream ss8; 
        for(auto mkr:extract_ref_line_->ego_path().rightMarkr){
            ss7 << " " << mkr.x;
            ss8 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " ego_riline_out_x=[ " << ss7.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ego_riline_out_y=[ " << ss8.str() << " ]" << std::endl;
    }
#endif

#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].center_line.pntSize:: " << (int)map_lane.lanes[1].center_line.pntSize << std::endl;
#endif
#ifdef SMOOTH
    {
        std::stringstream ss3;
        std::stringstream ss4;
        for (int i = 0; i < map_lane.lanes[1].center_line.pntSize; i++) {
            ss3 << " " << map_lane.lanes[1].center_line.linePnt[i].dX;
            ss4 << " " << map_lane.lanes[1].center_line.linePnt[i].dY;
        }
        std::cout << std::cout.precision(12) << " le_cl_out_x=[ " << ss3.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " le_cl_out_y=[ " << ss4.str() << " ]" << std::endl;
        std::stringstream ss5;
        std::stringstream ss6;       
        for(auto mkr:extract_ref_line_->left_path().leftMarkr){
            ss5 << " " << mkr.x;
            ss6 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " le_leline_out_x=[ " << ss5.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " le_leline_out_y=[ " << ss6.str() << " ]" << std::endl;
        std::stringstream ss7;
        std::stringstream ss8; 
        for(auto mkr:extract_ref_line_->left_path().rightMarkr){
            ss7 << " " << mkr.x;
            ss8 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " le_riline_out_x=[ " << ss7.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " le_riline_out_y=[ " << ss8.str() << " ]" << std::endl;
    }
#endif
    map_lane.priorIndex = extract_ref_line_->prior_path_index();  // 0-ego;1-left;2-right;3-left_left;4-right_right

#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " map_lane.lanes[3].center_line.pntSize:: " << (int)map_lane.lanes[3].center_line.pntSize << std::endl;
#endif
#ifdef SMOOTH
    {
        std::stringstream ss3;
        std::stringstream ss4;
        for (int i = 0; i < map_lane.lanes[3].center_line.pntSize; i++) {
            ss3 << " " << map_lane.lanes[3].center_line.linePnt[i].dX;
            ss4 << " " << map_lane.lanes[3].center_line.linePnt[i].dY;
        }
        std::cout << std::cout.precision(12) << " ri_cl_out_x=[ " << ss3.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ri_cl_out_y=[ " << ss4.str() << " ]" << std::endl;
        std::stringstream ss5;
        std::stringstream ss6;       
        for(auto mkr:extract_ref_line_->right_path().leftMarkr){
            ss5 << " " << mkr.x;
            ss6 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " ri_leline_out_x=[ " << ss5.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ri_leline_out_y=[ " << ss6.str() << " ]" << std::endl;
        std::stringstream ss7;
        std::stringstream ss8; 
        for(auto mkr:extract_ref_line_->right_path().rightMarkr){
            ss7 << " " << mkr.x;
            ss8 << " " << mkr.y;
        }
        std::cout << std::cout.precision(12) << " ri_riline_out_x=[ " << ss7.str() << " ]" << std::endl;
        std::cout << std::cout.precision(12) << " ri_riline_out_y=[ " << ss8.str() << " ]" << std::endl;
    }
#endif

#ifdef EFM_PLOT2D

#ifdef __QNX__
    auto efm_centerline_start_now = bsw::gptp_clock::now();
    auto efm_centerline_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_centerline_start_now.time_since_epoch());
#endif

    for (int i = 0; i < map_lane.lanes[2].center_line.pntSize; i++) {
        if (i % 4 == 0) {
            ego_plot_x.push_back(map_lane.lanes[2].center_line.linePnt[i].dX);
            ego_plot_y.push_back(map_lane.lanes[2].center_line.linePnt[i].dY);
        }
    }
    for (int i = 0; i < map_lane.lanes[1].center_line.pntSize; i++) {
        if (i % 4 == 0) {
            left_plot_x.push_back(map_lane.lanes[1].center_line.linePnt[i].dX);
            left_plot_y.push_back(map_lane.lanes[1].center_line.linePnt[i].dY);
        }
    }
    for (int i = 0; i < map_lane.lanes[3].center_line.pntSize; i++) {
        if (i % 4 == 0) {
            right_plot_x.push_back(map_lane.lanes[3].center_line.linePnt[i].dX);
            right_plot_y.push_back(map_lane.lanes[3].center_line.linePnt[i].dY);
        }
    }
    // for (int i = 0; i < map_lane.lanes[0].center_line.pntSize; i++) {
    //     if (i % 4 == 0) {
    //         left_left_plot_x.push_back(map_lane.lanes[0].center_line.linePnt[i].dX);
    //         left_left_plot_y.push_back(map_lane.lanes[0].center_line.linePnt[i].dY);
    //     }
    // }
    // for (int i = 0; i < map_lane.lanes[4].center_line.pntSize; i++) {
    //     if (i % 4 == 0) {
    //         right_right_plot_x.push_back(map_lane.lanes[4].center_line.linePnt[i].dX);
    //         right_right_plot_y.push_back(map_lane.lanes[4].center_line.linePnt[i].dY);
    //     }
    // }

#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_plot_x.size(): " << ego_plot_x.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_plot_x.size(): " << left_plot_x.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_plot_x.size(): " << right_plot_x.size() << std::endl;
#endif
    ZPLOTXYF("EFM_INFO", "--orange2", ego_plot_x, ego_plot_y);
    ZPLOTXYF("EFM_INFO", "--red2", left_plot_x, left_plot_y);
    ZPLOTXYF("EFM_INFO", "--green2", right_plot_x, right_plot_y);
    // ZPLOTXYF("EFM_INFO", "--brown2", left_left_plot_x, left_left_plot_y);
    // ZPLOTXYF("EFM_INFO", "--yellow2", right_right_plot_x, right_right_plot_y);

#ifdef __QNX__
    auto efm_centerline_end_now = bsw::gptp_clock::now();
    auto efm_centerline_end_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_centerline_end_now.time_since_epoch());
    efm_centerline_use_time_ns =
        static_cast<uint64_t>(efm_centerline_end_time.count() - efm_centerline_start_time.count());
#endif

#endif
    ZTEXT("EFM_INFO", "efm_centerline_use_time_ns: ", 80, 27, "efm_centerline_use_time_ns: {}",
          efm_centerline_use_time_ns);

    return true;
}

bool EnvironmentModel::MakeSideLaneLeft(uint64_t& errorcode, MapLaneMsg& map_lane) {
    std::vector<double> plot_left_right_solid_line_plot_x{};
    std::vector<double> plot_left_right_solid_line_plot_y{};
    std::vector<double> plot_left_right_dot_line_plot_x{};
    std::vector<double> plot_left_right_dot_line_plot_y{};
    std::vector<double> plot_left_right_virtual_line_plot_x{};
    std::vector<double> plot_left_right_virtual_line_plot_y{};
    std::vector<double> plot_left_left_solid_line_plot_x{};
    std::vector<double> plot_left_left_solid_line_plot_y{};
    std::vector<double> plot_left_left_dot_line_plot_x{};
    std::vector<double> plot_left_left_dot_line_plot_y{};
    std::vector<double> plot_left_left_virtual_line_plot_x{};
    std::vector<double> plot_left_left_virtual_line_plot_y{};

#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left lane, left marker######" << std::endl;
#endif
    OutputOneSideLine0630(
        extract_ref_line_->left_path().leftMarkrSection, extract_ref_line_->left_path().leftLineMkrInfos,
        extract_ref_line_->left_path().back_link_left_line_infos, map_lane.lanes[1].left_line,
        plot_left_left_solid_line_plot_x, plot_left_left_solid_line_plot_y, plot_left_left_dot_line_plot_x,
        plot_left_left_dot_line_plot_y, plot_left_left_virtual_line_plot_x, plot_left_left_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left lane, right marker######" << std::endl;
#endif
    OutputOneSideLine0630(
        extract_ref_line_->left_path().rightMarkrSection, extract_ref_line_->left_path().rightLineMkrInfos,
        extract_ref_line_->left_path().back_link_right_line_infos, map_lane.lanes[1].right_line,
        plot_left_right_solid_line_plot_x, plot_left_right_solid_line_plot_y, plot_left_right_dot_line_plot_x,
        plot_left_right_dot_line_plot_y, plot_left_right_virtual_line_plot_x, plot_left_right_virtual_line_plot_y, 1);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].left_line.bIsAvailable: " << (int)map_lane.lanes[1].left_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].left_line.pntSize: " << (int)map_lane.lanes[1].left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << "map_lane.lanes[1].left_line.etype: ";
    ss1 << "map_lane.lanes[1].left_line.eColor: ";
    ss2 << "map_lane.lanes[1].left_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[1].left_line.etype.size(); i++) {
        ss << " ,[v: " << map_lane.lanes[1].left_line.etype[i].valid
           << " ,s: " << map_lane.lanes[1].left_line.etype[i].s
           << " ,type: " << (int)map_lane.lanes[1].left_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[1].left_line.eColor.size(); i++) {
        ss1 << " ,[v: " << map_lane.lanes[1].left_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[1].left_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[1].left_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[1].left_line.virtual_line_s.size(); i++) {
        ss2 << " ,[v: " << map_lane.lanes[1].left_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[1].left_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[1].left_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].right_line.bIsAvailable: " << (int)map_lane.lanes[1].right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[1].right_line.pntSize: " << (int)map_lane.lanes[1].right_line.pntSize << std::endl;
    ss3 << "map_lane.lanes[1].right_line.etype: ";
    ss4 << "map_lane.lanes[1].right_line.eColor: ";
    ss5 << "map_lane.lanes[1].right_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[1].right_line.etype.size(); i++) {
        ss3 << " ,[v: " << map_lane.lanes[1].right_line.etype[i].valid
            << " ,s: " << map_lane.lanes[1].right_line.etype[i].s
            << " ,type: " << (int)map_lane.lanes[1].right_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[1].right_line.eColor.size(); i++) {
        ss4 << " ,[v: " << map_lane.lanes[1].right_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[1].right_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[1].right_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[1].right_line.virtual_line_s.size(); i++) {
        ss5 << " ,[v: " << map_lane.lanes[1].right_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[1].right_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[1].right_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
#endif

#ifdef EFM_PLOT2D
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_left_right_solid_line_plot_x, plot_left_right_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_left_right_dot_line_plot_x, plot_left_right_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_right_virtual_line_plot_x, plot_left_right_virtual_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_left_left_solid_line_plot_x, plot_left_left_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_left_left_dot_line_plot_x, plot_left_left_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_left_virtual_line_plot_x, plot_left_left_virtual_line_plot_y);
#endif

    return true;
}

bool EnvironmentModel::MakeSideLaneLeftLeft(uint64_t& errorcode, MapLaneMsg& map_lane) {
    std::vector<double> plot_leftleft_right_solid_line_plot_x{};
    std::vector<double> plot_leftleft_right_solid_line_plot_y{};
    std::vector<double> plot_leftleft_right_dot_line_plot_x{};
    std::vector<double> plot_leftleft_right_dot_line_plot_y{};
    std::vector<double> plot_leftleft_right_virtual_line_plot_x{};
    std::vector<double> plot_leftleft_right_virtual_line_plot_y{};
    std::vector<double> plot_leftleft_left_solid_line_plot_x{};
    std::vector<double> plot_leftleft_left_solid_line_plot_y{};
    std::vector<double> plot_leftleft_left_dot_line_plot_x{};
    std::vector<double> plot_leftleft_left_dot_line_plot_y{};
    std::vector<double> plot_leftleft_left_virtual_line_plot_x{};
    std::vector<double> plot_leftleft_left_virtual_line_plot_y{};

#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "leftleft lane, left marker######" << std::endl;
#endif
    OutputOneSideLine0630(extract_ref_line_->left_left_path().leftMarkrSection,
                          extract_ref_line_->left_left_path().leftLineMkrInfos,
                          extract_ref_line_->left_left_path().back_link_left_line_infos, map_lane.lanes[0].left_line,
                          plot_leftleft_left_solid_line_plot_x, plot_leftleft_left_solid_line_plot_y,
                          plot_leftleft_left_dot_line_plot_x, plot_leftleft_left_dot_line_plot_y,
                          plot_leftleft_left_virtual_line_plot_x, plot_leftleft_left_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "leftleft lane, right marker######" << std::endl;
#endif
    OutputOneSideLine0630(extract_ref_line_->left_left_path().rightMarkrSection,
                          extract_ref_line_->left_left_path().rightLineMkrInfos,
                          extract_ref_line_->left_left_path().back_link_right_line_infos, map_lane.lanes[0].right_line,
                          plot_leftleft_right_solid_line_plot_x, plot_leftleft_right_solid_line_plot_y,
                          plot_leftleft_right_dot_line_plot_x, plot_leftleft_right_dot_line_plot_y,
                          plot_leftleft_right_virtual_line_plot_x, plot_leftleft_right_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].left_line.bIsAvailable: " << (int)map_lane.lanes[0].left_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].left_line.pntSize: " << (int)map_lane.lanes[0].left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << "map_lane.lanes[0].left_line.etype: ";
    ss1 << "map_lane.lanes[0].left_line.eColor: ";
    ss2 << "map_lane.lanes[0].left_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[0].left_line.etype.size(); i++) {
        ss << " ,[v: " << map_lane.lanes[0].left_line.etype[i].valid
           << " ,s: " << map_lane.lanes[0].left_line.etype[i].s
           << " ,type: " << (int)map_lane.lanes[0].left_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[0].left_line.eColor.size(); i++) {
        ss1 << " ,[v: " << map_lane.lanes[0].left_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[0].left_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[0].left_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[0].left_line.virtual_line_s.size(); i++) {
        ss2 << " ,[v: " << map_lane.lanes[0].left_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[0].left_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[0].left_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].right_line.bIsAvailable: " << (int)map_lane.lanes[0].right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[0].right_line.pntSize: " << (int)map_lane.lanes[0].right_line.pntSize << std::endl;
    ss3 << "map_lane.lanes[0].right_line.etype: ";
    ss4 << "map_lane.lanes[0].right_line.eColor: ";
    ss5 << "map_lane.lanes[0].right_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[0].right_line.etype.size(); i++) {
        ss3 << " ,[v: " << map_lane.lanes[0].right_line.etype[i].valid
            << " ,s: " << map_lane.lanes[0].right_line.etype[i].s
            << " ,type: " << (int)map_lane.lanes[0].right_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[0].right_line.eColor.size(); i++) {
        ss4 << " ,[v: " << map_lane.lanes[0].right_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[0].right_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[0].right_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[0].right_line.virtual_line_s.size(); i++) {
        ss5 << " ,[v: " << map_lane.lanes[0].right_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[0].right_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[0].right_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
#endif

#ifdef EFM_PLOT2D
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_left_right_solid_line_plot_x, plot_left_right_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_left_right_dot_line_plot_x, plot_left_right_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_right_virtual_line_plot_x, plot_left_right_virtual_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_leftleft_left_solid_line_plot_x, plot_leftleft_left_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_leftleft_left_dot_line_plot_x, plot_leftleft_left_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_left_virtual_line_plot_x, plot_left_left_virtual_line_plot_y);
#endif
    return true;
}

bool EnvironmentModel::MakeSideLaneRight(uint64_t& errorcode, MapLaneMsg& map_lane) {
    std::vector<double> plot_right_right_solid_line_plot_x{};
    std::vector<double> plot_right_right_solid_line_plot_y{};
    std::vector<double> plot_right_right_dot_line_plot_x{};
    std::vector<double> plot_right_right_dot_line_plot_y{};
    std::vector<double> plot_right_right_virtual_line_plot_x{};
    std::vector<double> plot_right_right_virtual_line_plot_y{};
    std::vector<double> plot_right_left_solid_line_plot_x{};
    std::vector<double> plot_right_left_solid_line_plot_y{};
    std::vector<double> plot_right_left_dot_line_plot_x{};
    std::vector<double> plot_right_left_dot_line_plot_y{};
    std::vector<double> plot_right_left_virtual_line_plot_x{};
    std::vector<double> plot_right_left_virtual_line_plot_y{};

#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right lane, left marker######: " << std::endl;
#endif
    OutputOneSideLine0630(
        extract_ref_line_->right_path().leftMarkrSection, extract_ref_line_->right_path().leftLineMkrInfos,
        extract_ref_line_->right_path().back_link_left_line_infos, map_lane.lanes[3].left_line,
        plot_right_left_solid_line_plot_x, plot_right_left_solid_line_plot_y, plot_right_left_dot_line_plot_x,
        plot_right_left_dot_line_plot_y, plot_right_left_virtual_line_plot_x, plot_right_left_virtual_line_plot_y, 2);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right lane, right marker######: " << std::endl;
#endif
    OutputOneSideLine0630(extract_ref_line_->right_path().rightMarkrSection,
                          extract_ref_line_->right_path().rightLineMkrInfos,
                          extract_ref_line_->right_path().back_link_right_line_infos, map_lane.lanes[3].right_line,
                          plot_right_right_solid_line_plot_x, plot_right_right_solid_line_plot_y,
                          plot_right_right_dot_line_plot_x, plot_right_right_dot_line_plot_y,
                          plot_right_right_virtual_line_plot_x, plot_right_right_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].left_line.bIsAvailable: " << (int)map_lane.lanes[3].left_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].left_line.pntSize: " << (int)map_lane.lanes[3].left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << "map_lane.lanes[3].left_line.etype: ";
    ss1 << "map_lane.lanes[3].left_line.eColor: ";
    ss2 << "map_lane.lanes[3].left_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[3].left_line.etype.size(); i++) {
        ss << " ,[v: " << map_lane.lanes[3].left_line.etype[i].valid
           << " ,s: " << map_lane.lanes[3].left_line.etype[i].s
           << " ,type: " << (int)map_lane.lanes[3].left_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[3].left_line.eColor.size(); i++) {
        ss1 << " ,[v: " << map_lane.lanes[3].left_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[3].left_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[3].left_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[3].left_line.virtual_line_s.size(); i++) {
        ss2 << " ,[v: " << map_lane.lanes[3].left_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[3].left_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[3].left_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].right_line.bIsAvailable: " << (int)map_lane.lanes[3].right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[3].right_line.pntSize: " << (int)map_lane.lanes[3].right_line.pntSize << std::endl;
    ss3 << "map_lane.lanes[3].right_line.etype: ";
    ss4 << "map_lane.lanes[3].right_line.eColor: ";
    ss5 << "map_lane.lanes[3].right_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[3].right_line.etype.size(); i++) {
        ss3 << " ,[v: " << map_lane.lanes[3].right_line.etype[i].valid
            << " ,s: " << map_lane.lanes[3].right_line.etype[i].s
            << " ,type: " << (int)map_lane.lanes[3].right_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[3].right_line.eColor.size(); i++) {
        ss4 << " ,[v: " << map_lane.lanes[3].right_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[3].right_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[3].right_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[3].right_line.virtual_line_s.size(); i++) {
        ss5 << " ,[v: " << map_lane.lanes[3].right_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[3].right_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[3].right_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
#endif

#ifdef EFM_PLOT2D
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_right_right_solid_line_plot_x, plot_right_right_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_right_right_dot_line_plot_x, plot_right_right_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_right_virtual_line_plot_x, plot_right_right_virtual_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_right_left_solid_line_plot_x, plot_right_left_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_right_left_dot_line_plot_x, plot_right_left_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_left_virtual_line_plot_x, plot_right_left_virtual_line_plot_y);
#endif

    return true;
}

bool EnvironmentModel::MakeSideLaneRightRight(uint64_t& errorcode, MapLaneMsg& map_lane) {
    std::vector<double> plot_rightright_right_solid_line_plot_x{};
    std::vector<double> plot_rightright_right_solid_line_plot_y{};
    std::vector<double> plot_rightright_right_dot_line_plot_x{};
    std::vector<double> plot_rightright_right_dot_line_plot_y{};
    std::vector<double> plot_rightright_right_virtual_line_plot_x{};
    std::vector<double> plot_rightright_right_virtual_line_plot_y{};
    std::vector<double> plot_rightright_left_solid_line_plot_x{};
    std::vector<double> plot_rightright_left_solid_line_plot_y{};
    std::vector<double> plot_rightright_left_dot_line_plot_x{};
    std::vector<double> plot_rightright_left_dot_line_plot_y{};
    std::vector<double> plot_rightright_left_virtual_line_plot_x{};
    std::vector<double> plot_rightright_left_virtual_line_plot_y{};

#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "rightright lane, left marker######: " << std::endl;
#endif
    OutputOneSideLine0630(extract_ref_line_->right_right_path().leftMarkrSection,
                          extract_ref_line_->right_right_path().leftLineMkrInfos,
                          extract_ref_line_->right_right_path().back_link_left_line_infos, map_lane.lanes[4].left_line,
                          plot_rightright_left_solid_line_plot_x, plot_rightright_left_solid_line_plot_y,
                          plot_rightright_left_dot_line_plot_x, plot_rightright_left_dot_line_plot_y,
                          plot_rightright_left_virtual_line_plot_x, plot_rightright_left_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right lane, right marker######: " << std::endl;
#endif
    OutputOneSideLine0630(extract_ref_line_->right_right_path().rightMarkrSection,
                          extract_ref_line_->right_right_path().rightLineMkrInfos,
                          extract_ref_line_->right_right_path().back_link_right_line_infos,
                          map_lane.lanes[4].right_line, plot_rightright_right_solid_line_plot_x,
                          plot_rightright_right_solid_line_plot_y, plot_rightright_right_dot_line_plot_x,
                          plot_rightright_right_dot_line_plot_y, plot_rightright_right_virtual_line_plot_x,
                          plot_rightright_right_virtual_line_plot_y, 0);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].left_line.bIsAvailable: " << (int)map_lane.lanes[4].left_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].left_line.pntSize: " << (int)map_lane.lanes[4].left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << "map_lane.lanes[4].left_line.etype: ";
    ss1 << "map_lane.lanes[4].left_line.eColor: ";
    ss2 << "map_lane.lanes[4].left_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[4].left_line.etype.size(); i++) {
        ss << " ,[v: " << map_lane.lanes[4].left_line.etype[i].valid
           << " ,s: " << map_lane.lanes[4].left_line.etype[i].s
           << " ,type: " << (int)map_lane.lanes[4].left_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[4].left_line.eColor.size(); i++) {
        ss1 << " ,[v: " << map_lane.lanes[4].left_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[4].left_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[4].left_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[4].left_line.virtual_line_s.size(); i++) {
        ss2 << " ,[v: " << map_lane.lanes[4].left_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[4].left_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[4].left_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].right_line.bIsAvailable: " << (int)map_lane.lanes[4].right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[4].right_line.pntSize: " << (int)map_lane.lanes[4].right_line.pntSize << std::endl;
    ss3 << "map_lane.lanes[4].right_line.etype: ";
    ss4 << "map_lane.lanes[4].right_line.eColor: ";
    ss5 << "map_lane.lanes[4].right_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[4].right_line.etype.size(); i++) {
        ss3 << " ,[v: " << map_lane.lanes[4].right_line.etype[i].valid
            << " ,s: " << map_lane.lanes[4].right_line.etype[i].s
            << " ,type: " << (int)map_lane.lanes[4].right_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[4].right_line.eColor.size(); i++) {
        ss4 << " ,[v: " << map_lane.lanes[4].right_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[4].right_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[4].right_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[4].right_line.virtual_line_s.size(); i++) {
        ss5 << " ,[v: " << map_lane.lanes[4].right_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[4].right_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[4].right_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
#endif

#ifdef EFM_PLOT2D
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_rightright_right_solid_line_plot_x, plot_rightright_right_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_rightright_right_dot_line_plot_x, plot_rightright_right_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_right_virtual_line_plot_x, plot_right_right_virtual_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "-blue4", plot_right_left_solid_line_plot_x, plot_right_left_solid_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--blue4", plot_right_left_dot_line_plot_x, plot_right_left_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_left_virtual_line_plot_x, plot_right_left_virtual_line_plot_y);
#endif

    return true;
}

bool EnvironmentModel::MakeSideLaneEgo(uint64_t& errorcode, MapLaneMsg& map_lane) {
    std::vector<double> plot_ego_right_solid_line_plot_x{};
    std::vector<double> plot_ego_right_solid_line_plot_y{};
    std::vector<double> plot_ego_right_dot_line_plot_x{};
    std::vector<double> plot_ego_right_dot_line_plot_y{};
    std::vector<double> plot_ego_right_virtual_line_plot_x{};
    std::vector<double> plot_ego_right_virtual_line_plot_y{};
    std::vector<double> plot_ego_left_solid_line_plot_x{};
    std::vector<double> plot_ego_left_solid_line_plot_y{};
    std::vector<double> plot_ego_left_dot_line_plot_x{};
    std::vector<double> plot_ego_left_dot_line_plot_y{};
    std::vector<double> plot_ego_left_virtual_line_plot_x{};
    std::vector<double> plot_ego_left_virtual_line_plot_y{};

#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego lane, left marker######" << std::endl;
#endif
    OutputOneSideLine0630(
        extract_ref_line_->ego_path().leftMarkrSection, extract_ref_line_->ego_path().leftLineMkrInfos,
        extract_ref_line_->ego_path().back_link_left_line_infos, map_lane.lanes[2].left_line,
        plot_ego_left_solid_line_plot_x, plot_ego_left_solid_line_plot_y, plot_ego_left_dot_line_plot_x,
        plot_ego_left_dot_line_plot_y, plot_ego_left_virtual_line_plot_x, plot_ego_left_virtual_line_plot_y, 2);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego lane, right marker######" << std::endl;
#endif
    OutputOneSideLine0630(
        extract_ref_line_->ego_path().rightMarkrSection, extract_ref_line_->ego_path().rightLineMkrInfos,
        extract_ref_line_->ego_path().back_link_right_line_infos, map_lane.lanes[2].right_line,
        plot_ego_right_solid_line_plot_x, plot_ego_right_solid_line_plot_y, plot_ego_right_dot_line_plot_x,
        plot_ego_right_dot_line_plot_y, plot_ego_right_virtual_line_plot_x, plot_ego_right_virtual_line_plot_y, 1);
#ifdef EM_COUT1
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].left_line.bIsAvailable: " << (int)map_lane.lanes[2].left_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].left_line.pntSize: " << (int)map_lane.lanes[2].left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << "map_lane.lanes[2].left_line.etype: ";
    ss1 << "map_lane.lanes[2].left_line.eColor: ";
    ss2 << "map_lane.lanes[2].left_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[2].left_line.etype.size(); i++) {
        ss << " ,[v: " << map_lane.lanes[2].left_line.etype[i].valid
           << " ,s: " << map_lane.lanes[2].left_line.etype[i].s
           << " ,type: " << (int)map_lane.lanes[2].left_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[2].left_line.eColor.size(); i++) {
        ss1 << " ,[v: " << map_lane.lanes[2].left_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[2].left_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[2].left_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[2].left_line.virtual_line_s.size(); i++) {
        ss2 << " ,[v: " << map_lane.lanes[2].left_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[2].left_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[2].left_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].right_line.bIsAvailable: " << (int)map_lane.lanes[2].right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_lane.lanes[2].right_line.pntSize: " << (int)map_lane.lanes[2].right_line.pntSize << std::endl;
    ss3 << "map_lane.lanes[2].right_line.etype: ";
    ss4 << "map_lane.lanes[2].right_line.eColor: ";
    ss5 << "map_lane.lanes[2].right_line.virtual_line_s: ";
    for (int i = 0; i < map_lane.lanes[2].right_line.etype.size(); i++) {
        ss3 << " ,[v: " << map_lane.lanes[2].right_line.etype[i].valid
            << " ,s: " << map_lane.lanes[2].right_line.etype[i].s
            << " ,type: " << (int)map_lane.lanes[2].right_line.etype[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[2].right_line.eColor.size(); i++) {
        ss4 << " ,[v: " << map_lane.lanes[2].right_line.eColor[i].valid
            << " ,s: " << map_lane.lanes[2].right_line.eColor[i].s
            << " ,type: " << (int)map_lane.lanes[2].right_line.eColor[i].type.data_ << " ]";
    }
    for (int i = 0; i < map_lane.lanes[2].right_line.virtual_line_s.size(); i++) {
        ss5 << " ,[v: " << map_lane.lanes[2].right_line.virtual_line_s[i].valid_
            << " ,start_s: " << map_lane.lanes[2].right_line.virtual_line_s[i].start_s
            << " ,end_s: " << map_lane.lanes[2].right_line.virtual_line_s[i].end_s << " ]";
    }
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
#endif

#ifdef EFM_PLOT2D
    ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_right_solid_line_plot_x, plot_ego_right_solid_line_plot_y);
    ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_right_dot_line_plot_x, plot_ego_right_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_ego_right_virtual_line_plot_x, plot_ego_right_virtual_line_plot_y);
    ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_left_solid_line_plot_x, plot_ego_left_solid_line_plot_y);
    ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_left_dot_line_plot_x, plot_ego_left_dot_line_plot_y);
    // ZPLOTXYF("EFM_INFO", "--orange4", plot_ego_left_virtual_line_plot_x, plot_ego_left_virtual_line_plot_y);
#endif

    return true;
}

bool EnvironmentModel::MakeSideLane(uint64_t& errorcode, MapLaneMsg& map_lane) {
    uint64_t efm_sideline_use_time_ns = 0;

    MakeSideLaneEgo(errorcode, map_lane);
    MakeSideLaneLeft(errorcode, map_lane);
    MakeSideLaneLeftLeft(errorcode, map_lane);
    MakeSideLaneRight(errorcode, map_lane);
    MakeSideLaneRightRight(errorcode, map_lane);

    // #ifdef EFM_PLOT2D

    // #ifdef __QNX__
    //     auto efm_sideline_start_now = bsw::gptp_clock::now();
    //     auto efm_sideline_start_time =
    //         std::chrono::duration_cast<std::chrono::nanoseconds>(efm_sideline_start_now.time_since_epoch());
    // #endif

    //     ZPLOTXYF("EFM_INFO", "-blue4", plot_right_right_solid_line_plot_x, plot_right_right_solid_line_plot_y);
    //     ZPLOTXYF("EFM_INFO", "--blue4", plot_right_right_dot_line_plot_x, plot_right_right_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_right_virtual_line_plot_x,
    //     plot_right_right_virtual_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "-blue4", plot_right_left_solid_line_plot_x, plot_right_left_solid_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--blue4", plot_right_left_dot_line_plot_x, plot_right_left_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_right_left_virtual_line_plot_x,
    //     plot_right_left_virtual_line_plot_y);

    //     ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_right_solid_line_plot_x, plot_ego_right_solid_line_plot_y);
    //     ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_right_dot_line_plot_x, plot_ego_right_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_ego_right_virtual_line_plot_x, plot_ego_right_virtual_line_plot_y);
    //     ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_left_solid_line_plot_x, plot_ego_left_solid_line_plot_y);
    //     ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_left_dot_line_plot_x, plot_ego_left_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_ego_left_virtual_line_plot_x, plot_ego_left_virtual_line_plot_y);

    //     // ZPLOTXYF("EFM_INFO", "-blue4", plot_left_right_solid_line_plot_x, plot_left_right_solid_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--blue4", plot_left_right_dot_line_plot_x, plot_left_right_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_right_virtual_line_plot_x,
    //     plot_left_right_virtual_line_plot_y); ZPLOTXYF("EFM_INFO", "-blue4", plot_left_left_solid_line_plot_x,
    //     plot_left_left_solid_line_plot_y); ZPLOTXYF("EFM_INFO", "--blue4", plot_left_left_dot_line_plot_x,
    //     plot_left_left_dot_line_plot_y);
    //     // ZPLOTXYF("EFM_INFO", "--orange4", plot_left_left_virtual_line_plot_x, plot_left_left_virtual_line_plot_y);

    // #ifdef __QNX__
    //     auto efm_sideline_end_now = bsw::gptp_clock::now();
    //     auto efm_sideline_end_time =
    //         std::chrono::duration_cast<std::chrono::nanoseconds>(efm_sideline_end_now.time_since_epoch());
    //     efm_sideline_use_time_ns = static_cast<uint64_t>(efm_sideline_end_time.count() -
    //     efm_sideline_start_time.count());
    // #endif

    // #endif
    ZTEXT("EFM_INFO", "efm_sideline_use_time_ns: ", 80, 30, "efm_sideline_use_time_ns: {}", efm_sideline_use_time_ns);

    return true;
}

bool EnvironmentModel::MakeGuide(uint64_t& errorcode) {
    if (false == speed_limit_info_->Execute(*(map_position_.get()), *(map_static_info_.get()),
                                            get_link_id_index_lane_info_map(), candidate_lanes_model_,
                                            scenario_judge_model_, extract_ref_line_, traffic_info_, com_info_)) {
        return false;
    }

    return true;
}

bool EnvironmentModel::FixPathDensity(const EFMRefLinePoints& raw_reference_line,
                                      EFMRefLinePoints& fixed_reference_line) {
    if (raw_reference_line.size() < 3) {
        // std::cout << "raw point num to less" << std::endl;
        return false;
    }

    // 路径密度优化
    fixed_reference_line.clear();
    double sample_distance = 2.5;  // unit m
    //
    double accumu_s = 0;
    EFMPoint point0 = raw_reference_line[0];
    fixed_reference_line.emplace_back(point0);
    //
    const double accumu_s_threshold = 5000;
    // std::cout << "FixPathDensity, raw_reference_line.size(): " << raw_reference_line.size() << std::endl;
    for (int i = 1; i < raw_reference_line.size() && accumu_s < accumu_s_threshold; i++) {
        const EFMPoint& point1 = raw_reference_line[i];
        double distance = sqrt(pow(point1.x - point0.x, 2) + pow(point1.y - point0.y, 2));
        // std::cout << "i: " << i << " ,distance: " << distance << std::endl;
        // std::cout << "point0.x: " << point0.first << " , point0.y: " << point0.second << std::endl;
        // std::cout << "point1.x: " << point1.first << " , point1.y: " << point1.second << std::endl;
        if (distance <= sample_distance) {
            // std::cout << "   , continue;" << std::endl;
            continue;
        }
        while (distance > sample_distance && accumu_s < accumu_s_threshold) {
            double coeff = sample_distance / distance;
            double x = coeff * (point1.x - point0.x) + point0.x;
            double y = coeff * (point1.y - point0.y) + point0.y;
            point0.x = x;
            point0.y = y;
            fixed_reference_line.emplace_back(point0);
            accumu_s += sample_distance;
            distance -= sample_distance;
            // std::cout << " accumu_s: " << accumu_s << " distance: " << distance << std::endl;
        }
    }

    return true;
}

bool EnvironmentModel::GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_.find(link_id) != link_id_index_lane_info_map_.end()) {
        int link_index = link_id_index_lane_info_map_.at(link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

const std::unordered_map<uint32_t, int>& EnvironmentModel::get_link_id_index_lane_info_map() {
    return link_id_index_lane_info_map_;
}

const std::unordered_map<uint32_t, std::vector<int>>& EnvironmentModel::get_link_id_index_lane_connect_map() {
    return link_id_index_lane_connect_map_;
}

const std::unordered_map<uint32_t, std::vector<int>>& EnvironmentModel::get_to_link_id_index_lane_connect_map() {
    return to_link_id_index_lane_connect_map_;
}

bool EnvironmentModel::OutputOneCenterLane(const EFMRefLinePointsSection& line_section,
                                           message::efm::s_LaneModel_t& centerline) {
    centerline.centerline.clear();
    centerline.bIsAvailable = false;
    EFMRefLinePoints front_line = line_section[1];
    EFMRefLinePoints back_line = line_section[0];
    int start_index = -1;
    int end_index = -1;

    // fill back line
    for (int i = 1; i < back_line.size() && centerline.centerline.size() < 20; i++) {
        centerline.centerline.insert(centerline.centerline.begin(), {back_line[i].x, back_line[i].y});
        start_index = 20 - centerline.centerline.size();
    }
    while (centerline.centerline.size() < 20) {
        centerline.centerline.insert(centerline.centerline.begin(), {0.0, 0.0});
    }

    // fill front
    for (int i = 0; i < front_line.size() && centerline.centerline.size() < 81; i++) {
        centerline.centerline.push_back({front_line[i].x, front_line[i].y});
        end_index = centerline.centerline.size() - 1;
    }
    while (centerline.centerline.size() < 81) {
        centerline.centerline.push_back({0.0, 0.0});
    }

    if (start_index >= 0 && end_index > start_index && end_index < 81 && centerline.centerline.size() == 81) {
        centerline.bIsAvailable = true;
        centerline.StartIndex = start_index;
        centerline.EndIndex = end_index;
    }

    return true;
}

void EnvironmentModel::CutCenterLine(const EFMRefLinePointsSection& line_section,
                                     EFMRefLinePointsSection& line_section_cut) {
    int front_num = static_cast<int>(155 / p_front_center_line_sample_dist);
    int back_num = static_cast<int>(105 / p_back_center_line_sample_dist);
    line_section_cut[0].clear();
    line_section_cut[1].clear();
    // back
    for (int i = 0; i < line_section[0].size() && i < back_num; i++) {
        line_section_cut[0].push_back({line_section[0].at(i).x, line_section[0].at(i).y});
    }
    // front
    for (int i = 0; i < line_section[1].size() && i < front_num; i++) {
        line_section_cut[1].push_back({line_section[1].at(i).x, line_section[1].at(i).y});
    }
    return;
}

bool EnvironmentModel::SetLastLinkBoundsVal(uint8_t& last_ego_lane_id, double& last_bounds_val,
                                            double& cur_bounds_val) {
    uint16_t index_prior;
    EFMPoint foot_point_prior;
    EFMPoint ego_position_QCJ02;
    double distance_prior = 0.0;
    bool find_insile_prior = false;

    ego_position_QCJ02.x = map_position_->Lon.Lon * 360.0 / (4294967296);
    ego_position_QCJ02.y = map_position_->Lat.Lat * 360.0 / (4294967296);
    std::vector<EFMPoint> prior_line_points;
    prior_line_points.clear();
    message::map_map::s_LinkInfo_t link_infos;
    // TODO GetLinkInfos写成公有函数
    if (false == GetLinkInfos(map_position_->LinkId, link_infos)) {
        // static data none
        return true;
    }

    efm::EfmInit::GetInstance()->GetOneCenterLine(*map_static_info_, map_position_->LaneId, link_infos,
                                                  prior_line_points);
    CommonTool::CoordinateTool::GetInstance()->GetNearestPoint(prior_line_points, ego_position_QCJ02, index_prior,
                                                               foot_point_prior, distance_prior, find_insile_prior);
    distance_prior = distance_prior >= 1.4 ? 1.4 : distance_prior;

    if (last_ego_lane_id == map_position_->LaneId) {
        if (last_bounds_val == cur_bounds_val) {
            return true;
        } else {
            if (distance_prior < 2 * cur_bounds_val) {
                last_bounds_val = cur_bounds_val;
                return true;
            }

            double temp_bounds_val = (1.0 - distance_prior / 1.5) * cur_bounds_val;
            cur_bounds_val = temp_bounds_val;
            last_bounds_val = temp_bounds_val;
            return true;
        }
    } else if (((last_ego_lane_id + 1) == map_position_->LaneId) || (last_ego_lane_id == (map_position_->LaneId + 1))) {
        if (distance_prior < 0.5) {
            last_bounds_val = cur_bounds_val;
            return true;
        } else {
            double temp_bounds_val = (1.0 - distance_prior / 1.5) * cur_bounds_val;
            cur_bounds_val = temp_bounds_val;
            last_bounds_val = temp_bounds_val;
            return true;
        }

    } else {
        last_bounds_val = cur_bounds_val;
    }

    return true;
}

bool EnvironmentModel::SetLastLinksBoundsVal(uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                                             uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id,
                                             double& last_bounds_val, double& cur_bounds_val) {
    if (0 == map_position_->LaneId || 0 == map_position_->LinkId) {
        return false;
    }

    if (0 == last_ego_second_link_id || 0 == last_ego_sencond_lane_id) {
        last_bounds_val = cur_bounds_val;
        return true;
    }

    // error case
    if (last_bounds_val > cur_bounds_val) {
        last_bounds_val = cur_bounds_val;
        return false;
    }

    if (map_position_->LinkId == last_ego_first_link_id) {
        return SetLastLinkBoundsVal(last_ego_first_lane_id, last_bounds_val, cur_bounds_val);
    } else if (map_position_->LinkId == last_ego_second_link_id) {
        return SetLastLinkBoundsVal(last_ego_sencond_lane_id, last_bounds_val, cur_bounds_val);
    } else {
        last_bounds_val = cur_bounds_val;
        return true;
    }

    return true;
}

bool EnvironmentModel::ProcessCenterLine(const EFMRefLinePointsSection& line_section,
                                         EFMRefLinePointsSection& line_section_processed,
                                         uint32_t& last_ego_first_link_id, uint8_t& last_ego_first_lane_id,
                                         uint32_t& last_ego_second_link_id, uint8_t& last_ego_sencond_lane_id,
                                         double& last_bounds_val, const EFMRefLine& path) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "ProcessCenterLine: " << std::endl;
    line_section_processed[0].clear();
    line_section_processed[1].clear();
    ReferenceLineProvider refline_provider;
    EFMRefLinePoints left_mkr_points{};
    EFMRefLinePoints right_mkr_points{};
    EFMRefLinePoints raw_center_points{};
    std::vector<std::pair<double, double>> opt_center_points{};
    std::vector<uint8_t> left_bounds_type{};
    std::vector<uint8_t> right_bounds_type{};
    std::vector<double> left_bounds{};
    std::vector<double> right_bounds{};
    for (auto point : path.leftMarkr) {
        left_mkr_points.push_back({point.x, point.y});
    }
    for (auto point : path.rightMarkr) {
        right_mkr_points.push_back({point.x, point.y});
    }

    // fill back
    for (auto point : line_section[0]) {
        raw_center_points.insert(raw_center_points.begin(), {point.x, point.y});
    }
    int ego_index = raw_center_points.size() - 1;
    // fill front
    for (int i = 1; i < line_section[1].size(); i++) {
        raw_center_points.push_back({line_section[1].at(i).x, line_section[1].at(i).y});
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_mkr_points.size(): " << left_mkr_points.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_mkr_points.size(): " << right_mkr_points.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "raw_center_points.size(): " << raw_center_points.size() << std::endl;
#endif

    {  // get left bounds
        SLPoint sl(0.0, 0.0);
        for (auto point : raw_center_points) {
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinate(
                left_mkr_points, point, map_position_->PathOffset, path.leftMarkrOffset, sl);
            left_bounds.push_back(sl.l);
        }
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_bounds.size(): " << left_bounds.size() << std::endl;
    std::stringstream lb;
    for (auto b : left_bounds) {
        lb << " ," << b;
    }
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_bounds: " << lb.str() << std::endl;
#endif

    {  // get right bounds
        SLPoint sl(0.0, 0.0);
        for (auto point : raw_center_points) {
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinate(
                right_mkr_points, point, map_position_->PathOffset, path.rightMarkrOffset, sl);
            right_bounds.push_back(sl.l);
        }
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_bounds.size(): " << right_bounds.size() << std::endl;
    std::stringstream rb;
    for (auto b : right_bounds) {
        rb << " ," << b;
    }
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_bounds: " << rb.str() << std::endl;
#endif
    SetLastLinksBoundsVal(last_ego_first_link_id, last_ego_first_lane_id, last_ego_second_link_id,
                          last_ego_sencond_lane_id, last_bounds_val, refline_smooth_config.bounds_val);

    bool process_ok = false;
    process_ok = refline_provider.Execute(raw_center_points, opt_center_points, left_bounds, right_bounds,
                                          refline_smooth_config, left_bounds_type, right_bounds_type, path, ego_index);

    if (process_ok == true) {
        // fill back
        for (int i = 0; i <= ego_index && i < opt_center_points.size(); i++) {
            line_section_processed[0].insert(line_section_processed[0].begin(),
                                             {opt_center_points[i].first, opt_center_points[i].second});
        }
        // fill front
        for (int i = ego_index; i < opt_center_points.size(); i++) {
            line_section_processed[1].push_back({opt_center_points[i].first, opt_center_points[i].second});
        }

    } else {
        line_section_processed = line_section;
    }

#ifdef SMOOTH
    std::stringstream ss2;
    std::stringstream ss;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " process_ok: " << process_ok << std::endl;
    for (int i = 0; i < raw_center_points.size(); i++) {
        ss2 << " " << raw_center_points[i].x;
        ss << " " << raw_center_points[i].y;
    }
    std::cout << std::cout.precision(12) << " rawx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " rawy=[ " << ss.str() << " ]" << std::endl;

    std::stringstream ss3;
    std::stringstream ss4;
    for (int i = 0; i < opt_center_points.size(); i++) {
        ss3 << " " << opt_center_points[i].first;
        ss4 << " " << opt_center_points[i].second;
    }
    std::cout << std::cout.precision(12) << " optx=[ " << ss3.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " opty=[ " << ss4.str() << " ]" << std::endl;
#endif
    opt_kappas_.clear();
    opt_accumulated_s_.clear();
    // std::cout << "start to get opt_kappas" << std::endl;
    opt_kappas_ = refline_provider.opt_kappas();
    sub_module_output_.opt_kappas = refline_provider.opt_kappas();
    opt_accumulated_s_ = refline_provider.opt_accumulated_s();
    sub_module_output_.opt_accumulated_s = refline_provider.opt_accumulated_s();
    // std::cout << "end to get opt_kappas" << std::endl;

    return true;
}

bool EnvironmentModel::OutputOneCenterLane0630(const EFMRefLinePointsSection& line_section,
                                               message::efm::s_Lane_t& lane) {
    //保证输入的是间距2.5m一个点
    message::efm::s_CenterLine_t& centerline = lane.center_line;

    EFMRefLinePoints front_line = line_section[1];
    EFMRefLinePoints back_line = line_section[0];
    int centerline_pnt_num = 0;
    // fill back line
    int back_size = std::min(static_cast<int>(back_line.size()), 41);
    if (back_size <= 0) {
    } else {
        // not include ego point !!!! 所以不用[0]点
        for (int i = back_size - 1; i > 0 && i < back_line.size() && centerline_pnt_num < centerline.linePnt.size();
             i--) {
            centerline.linePnt[centerline_pnt_num].dX = static_cast<float>(back_line[i].x);
            centerline.linePnt[centerline_pnt_num].dY = static_cast<float>(back_line[i].y);
            centerline_pnt_num++;
        }
    }
    if (centerline_pnt_num > 0) {
        centerline.egoIndex = centerline_pnt_num - 1;
    }
    int back_point_num = centerline_pnt_num;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " back centerline_pnt_num: " << centerline_pnt_num <<
    // std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << " centerline.egoIndex: " <<
    // (int)centerline.egoIndex  << std::endl; fill front include ego point !!!! 所以要用[0]点
    for (int i = 0; i < front_line.size() && centerline_pnt_num < centerline.linePnt.size(); i++) {
        centerline.linePnt[centerline_pnt_num].dX = static_cast<float>(front_line[i].x);
        centerline.linePnt[centerline_pnt_num].dY = static_cast<float>(front_line[i].y);
        centerline_pnt_num++;
        if ((centerline_pnt_num - back_point_num) >= 61) {
            break;
        }
    }
    centerline.pntSize = centerline_pnt_num;
    if (centerline.pntSize > 0) {
        lane.enable_flag = true;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " tatal centerline_pnt_num: " << centerline_pnt_num <<
    // std::endl;

    return true;
}

bool EnvironmentModel::ProcessCenterLine(const EFMRefLinePointsSection& line_section,
                                         EFMRefLinePointsSection& line_section_processed, const EFMRefLine& path) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "ProcessCenterLine: " << std::endl;
    uint32_t ego_offset = map_position_->PathOffset;
    line_section_processed[0].clear();
    line_section_processed[1].clear();
    ReferenceLineProvider refline_provider;
    EFMRefLinePoints left_mkr_points{};
    EFMRefLinePoints right_mkr_points{};
    EFMRefLinePoints raw_center_points{};
    std::vector<std::pair<double, double>> opt_center_points{};
    std::vector<uint8_t> left_bounds_type{};
    std::vector<uint8_t> right_bounds_type{};
    std::vector<double> left_bounds{};
    std::vector<double> right_bounds{};
    for (auto point : path.leftMarkr) {
        left_mkr_points.push_back({point.x, point.y});
    }
    for (auto point : path.rightMarkr) {
        right_mkr_points.push_back({point.x, point.y});
    }

    // fill back
    for (auto point : line_section[0]) {
        raw_center_points.insert(raw_center_points.begin(), {point.x, point.y});
    }
    int ego_index = raw_center_points.size() - 1;
    // fill front
    for (int i = 1; i < line_section[1].size(); i++) {
        raw_center_points.push_back({line_section[1].at(i).x, line_section[1].at(i).y});
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_mkr_points.size(): " << left_mkr_points.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_mkr_points.size(): " << right_mkr_points.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "raw_center_points.size(): " << raw_center_points.size() << std::endl;
#endif

    {  // get left bounds
        SLPoint sl(0.0, 0.0);
        for (auto point : raw_center_points) {
            // std::cout << __FILE__ << "," << __LINE__ << "," << "ii: " << ii<<std::endl;
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(left_mkr_points, point,sl);
            left_bounds.push_back(sl.l);
        }
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_bounds.size(): " << left_bounds.size() << std::endl;
    std::stringstream lb;
    for (auto b : left_bounds) {
        lb << " ," << b;
    }
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_bounds: " << lb.str() << std::endl;
#endif
#ifdef SMOOTH
{
    std::stringstream ss2;
    std::stringstream ss;
    for (int i = 0; i < right_mkr_points.size(); i++) {
        ss2 << " " << right_mkr_points[i].x;
        ss << " " << right_mkr_points[i].y;
    }
    std::cout << std::cout.precision(12) << " right_mk_x=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " right_mk_y=[ " << ss.str() << " ]" << std::endl;

    std::stringstream ss3;
    std::stringstream ss4;
    for (int i = 0; i < left_mkr_points.size(); i++) {
        ss3 << " " << left_mkr_points[i].x;
        ss4 << " " << left_mkr_points[i].y;
    }
    std::cout << std::cout.precision(12) << " le_mk_x=[ " << ss3.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " le_mk_y=[ " << ss4.str() << " ]" << std::endl;    
}
#endif
    {  // get right bounds
        SLPoint sl(0.0, 0.0);
        for (auto point : raw_center_points) {
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(right_mkr_points, point, sl);
            right_bounds.push_back(sl.l);
        }
    }

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_bounds.size(): " << right_bounds.size() << std::endl;
    std::stringstream rb;
    for (auto b : right_bounds) {
        rb << " ," << b;
    }
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_bounds: " << rb.str() << std::endl;
#endif
    {
        //get bounds type, 和raw_center_points，left_bounds配对，根据offset来判断
        uint32_t offset_tmp = 0;
        bool find_location = false;
        for(int i =ego_index;i>=0;i--){
            offset_tmp = ego_offset - static_cast<uint32_t>(i*p_back_center_line_sample_dist*100);
            find_location = false;
            for(auto type_offset:path.line_type_offset){
                if(offset_tmp<=type_offset.end_offset && offset_tmp>=type_offset.start_offset){
                    find_location = true;
                    left_bounds_type.push_back(type_offset.left_line_type);
                    right_bounds_type.push_back(type_offset.right_line_type);
                    break;
                }
            }
            if(find_location == false){
                left_bounds_type.push_back(0);
                right_bounds_type.push_back(0);                
            }
        }
        for(int i =ego_index+1;i<raw_center_points.size();i++){
            offset_tmp = ego_offset + static_cast<uint32_t>((i-ego_index)*p_front_center_line_sample_dist*100);
            find_location = false;
            for(auto type_offset:path.line_type_offset){
                if(offset_tmp<=type_offset.end_offset && offset_tmp>=type_offset.start_offset){
                    find_location = true;
                    left_bounds_type.push_back(type_offset.left_line_type);
                    right_bounds_type.push_back(type_offset.right_line_type);
                    break;
                }
            }
            if(find_location == false){
                left_bounds_type.push_back(0);
                right_bounds_type.push_back(0);                
            }
        }        
    }

    bool process_ok = false;
    process_ok = refline_provider.Execute(raw_center_points, opt_center_points, left_bounds, right_bounds,
                                          refline_smooth_config, left_bounds_type, right_bounds_type, path, ego_index);

    if (process_ok == true) {
        // fill back
        for (int i = 0; i <= ego_index && i < opt_center_points.size(); i++) {
            line_section_processed[0].insert(line_section_processed[0].begin(),
                                             {opt_center_points[i].first, opt_center_points[i].second});
        }
#ifdef SMOOTH
        {
            std::stringstream ss2;
            std::stringstream ss;
            for (int i = 0; i < line_section_processed[0].size(); i++) {
                ss2 << " " << line_section_processed[0][i].x;
                ss << " " << line_section_processed[0][i].y;
            }
            std::cout << std::cout.precision(12) << " line_section_processed[0]x=[ " << ss2.str() << " ]" << std::endl;
            std::cout << std::cout.precision(12) << " line_section_processed[0]y=[ " << ss.str() << " ]" << std::endl;
        }
#endif

        std::vector<EFMPoint> line_back{};
        LinearInsertPoint(line_section_processed[0], line_back);
        line_section_processed[0].clear();
        line_section_processed[0] = line_back;

#ifdef SMOOTH
        {
            std::stringstream ss2;
            std::stringstream ss;
            for (int i = 0; i < line_back.size(); i++) {
                ss2 << " " << line_back[i].x;
                ss << " " << line_back[i].y;
            }
            std::cout << std::cout.precision(12) << " line_backx=[ " << ss2.str() << " ]" << std::endl;
            std::cout << std::cout.precision(12) << " line_backy=[ " << ss.str() << " ]" << std::endl;
        }
#endif
        // fill front
        for (int i = ego_index; i < opt_center_points.size(); i++) {
            line_section_processed[1].push_back({opt_center_points[i].first, opt_center_points[i].second});
        }

#ifdef SMOOTH
        {
            std::stringstream ss2;
            std::stringstream ss;
            for (int i = 0; i < line_section_processed[1].size(); i++) {
                ss2 << " " << line_section_processed[1][i].x;
                ss << " " << line_section_processed[1][i].y;
            }
            std::cout << std::cout.precision(12) << " line_section_processed[1]x=[ " << ss2.str() << " ]" << std::endl;
            std::cout << std::cout.precision(12) << " line_section_processed[1]y=[ " << ss.str() << " ]" << std::endl;
        }
#endif

        std::vector<EFMPoint> line_front{};
        LinearInsertPoint(line_section_processed[1], line_front);
        line_section_processed[1].clear();
        line_section_processed[1] = line_front;

#ifdef SMOOTH
        {
            std::stringstream ss2;
            std::stringstream ss;
            for (int i = 0; i < line_front.size(); i++) {
                ss2 << " " << line_front[i].x;
                ss << " " << line_front[i].y;
            }
            std::cout << std::cout.precision(12) << " line_frontx=[ " << ss2.str() << " ]" << std::endl;
            std::cout << std::cout.precision(12) << " line_fronty=[ " << ss.str() << " ]" << std::endl;
        }
#endif

    } else {
        std::vector<EFMPoint> line_back{};
        LinearInsertPoint(line_section_processed[0], line_back);
        line_section_processed[0].clear();
        line_section_processed[0] = line_back;

        std::vector<EFMPoint> line_front{};
        LinearInsertPoint(line_section_processed[1], line_front);
        line_section_processed[1].clear();
        line_section_processed[1] = line_front;
    }

#ifdef SMOOTH
    std::stringstream ss2;
    std::stringstream ss;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " process_ok: " << process_ok << std::endl;
    for (int i = 0; i < raw_center_points.size(); i++) {
        ss2 << " " << raw_center_points[i].x;
        ss << " " << raw_center_points[i].y;
    }
    std::cout << std::cout.precision(12) << " rawx=[ " << ss2.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " rawy=[ " << ss.str() << " ]" << std::endl;

    std::stringstream ss3;
    std::stringstream ss4;
    for (int i = 0; i < opt_center_points.size(); i++) {
        ss3 << " " << opt_center_points[i].first;
        ss4 << " " << opt_center_points[i].second;
    }
    std::cout << std::cout.precision(12) << " optx=[ " << ss3.str() << " ]" << std::endl;
    std::cout << std::cout.precision(12) << " opty=[ " << ss4.str() << " ]" << std::endl;
#endif

    opt_kappas_.clear();
    opt_accumulated_s_.clear();
    // std::cout << "start to get opt_kappas" << std::endl;
    opt_kappas_ = refline_provider.opt_kappas();
    sub_module_output_.opt_kappas = refline_provider.opt_kappas();
    opt_accumulated_s_ = refline_provider.opt_accumulated_s();
    sub_module_output_.opt_accumulated_s = refline_provider.opt_accumulated_s();
    // std::cout << "end to get opt_kappas" << std::endl;

    return true;
}

bool EnvironmentModel::OutputOneSideLine0630(
    const EFMRefLineMarkrSection& line_marker_section, const std::vector<LineMkrInfo_s>& line_infos,
    const std::vector<LineMkrInfo_s>& back_link_line_infos, struct message::efm::s_LineMkr_t& side_line,
    std::vector<double>& solid_line_plot_x, std::vector<double>& solid_line_plot_y,
    std::vector<double>& dot_line_plot_x, std::vector<double>& dot_line_plot_y,
    std::vector<double>& virtual_line_plot_x, std::vector<double>& virtual_line_plot_y, uint8_t line_direction) {
    std::vector<uint8_t> point_type_vec{};  // for plot
    side_line.bIsAvailable = false;
    side_line.pntSize = 0;
    side_line.egoIndex = 0;
    for (int i = 0; i < side_line.eColor.size(); i++) {
        side_line.eColor[i].valid = 0;
    }
    for (int i = 0; i < side_line.etype.size(); i++) {
        side_line.etype[i].valid = 0;
    }
    for (int i = 0; i < side_line.virtual_line_s.size(); i++) {
        side_line.virtual_line_s[i].valid_ = 0;
    }

#ifdef EM_COUT1
    {
        std::stringstream ss, ss1, ss2, ss3, ss4, ss5, ss6, ss7;
        ss << "line_marker_section[0].x: ";
        ss1 << "line_marker_section[0].y: ";
        ss2 << "line_marker_section[0].type: ";
        ss3 << "line_marker_section[0].color: ";
        for (int i = 0; i < line_marker_section[0].size(); i++) {
            ss << (line_marker_section[0])[i].x << " ";
            ss1 << (line_marker_section[0])[i].y << " ";
            ss2 << (int)(line_marker_section[0])[i].type << " ";
            ss3 << (int)(line_marker_section[0])[i].color << " ";
        }
        std::cout << std::cout.precision(1) << ss.str() << std::endl;
        std::cout << std::cout.precision(1) << ss1.str() << std::endl;
        std::cout << ss2.str() << std::endl;
        std::cout << ss3.str() << std::endl;
        ss4 << "line_marker_section[1].x: ";
        ss5 << "line_marker_section[1].y: ";
        ss6 << "line_marker_section[1].type: ";
        ss7 << "line_marker_section[1].color: ";
        for (int i = 0; i < line_marker_section[1].size(); i++) {
            ss4 << (line_marker_section[1])[i].x << " ";
            ss5 << (line_marker_section[1])[i].y << " ";
            ss6 << (int)(line_marker_section[1])[i].type << " ";
            ss7 << (int)(line_marker_section[1])[i].color << " ";
        }
        std::cout << std::cout.precision(1) << ss4.str() << std::endl;
        std::cout << std::cout.precision(1) << ss5.str() << std::endl;
        std::cout << ss6.str() << std::endl;
        std::cout << ss7.str() << std::endl;
    }
    {
        std::stringstream ss, ss2, ss3;
        ss << "back_link_line_infos.path_offset: ";
        ss2 << "back_link_line_infos[0].type: ";
        ss3 << "line_marker_section[0].color: ";
        for (int i = 0; i < back_link_line_infos.size(); i++) {
            ss << back_link_line_infos[i].path_offset << " ";
            ss2 << (int)back_link_line_infos[i].type << " ";
            ss3 << (int)back_link_line_infos[i].color << " ";
        }
        std::cout << ss.str() << std::endl;
        std::cout << ss2.str() << std::endl;
        std::cout << ss3.str() << std::endl;
    }
    {
        std::stringstream ss, ss2, ss3;
        ss << "line_infos.path_offset: ";
        ss2 << "line_infos[0].type: ";
        ss3 << "line_infos[0].color: ";
        for (int i = 0; i < line_infos.size(); i++) {
            ss << line_infos[i].path_offset << " ";
            ss2 << (int)line_infos[i].type << " ";
            ss3 << (int)line_infos[i].color << " ";
        }
        std::cout << ss.str() << std::endl;
        std::cout << ss2.str() << std::endl;
        std::cout << ss3.str() << std::endl;
    }
#endif
    std::vector<EFMMarkr> front_line = line_marker_section[1];
    std::vector<EFMMarkr> back_line = line_marker_section[0];
    int line_pnt_num = 0;
    // type
    uint8_t first_line_type = 0;
    float first_line_type_start_s = 10000;  // log type start s
    uint8_t second_line_type = 0;
    int type_section_num = 0;
    // color
    uint8_t first_color = 0;
    float first_color_s = 0;
    uint8_t second_color = 0;
    int color_section_number = 0;  // 接口只要自车前方两段

    // fill back line
    int back_size = std::min(static_cast<int>(back_line.size()), 41);
    if (back_size <= 0) {
        // no bake line
    } else {
        // not include ego point !!!! 所以不用[0]点
        for (int i = back_size - 1; i > 0 && i < back_line.size() && line_pnt_num < side_line.linePoints.size(); i--) {
            side_line.linePoints[line_pnt_num].dX = static_cast<float>(back_line[i].x);
            side_line.linePoints[line_pnt_num].dY = static_cast<float>(back_line[i].y);
            line_pnt_num++;
            point_type_vec.push_back(back_line[i].type);
        }
    }
    if (line_pnt_num > 0) {
        side_line.egoIndex = line_pnt_num - 1;
    }
    int back_point_num = line_pnt_num;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " back side_line_pnt_num: " << line_pnt_num << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " side_line.egoIndex: " << (int)side_line.egoIndex<<std::endl;
    // fill front
    // include ego point !!!! 所以要用[0]点； 前后两段线的[0]点都是一个点
    for (int i = 0; i < front_line.size() && line_pnt_num < side_line.linePoints.size(); i++) {
        side_line.linePoints[line_pnt_num].dX = static_cast<float>(front_line[i].x);
        side_line.linePoints[line_pnt_num].dY = static_cast<float>(front_line[i].y);
        line_pnt_num++;
        point_type_vec.push_back(front_line[i].type);
        if ((line_pnt_num - back_point_num) >= 61) {
            break;
        }
    }

    // deal type, clor, 先整合
    uint32_t ego_offset = map_position_->PathOffset;
    // int type_section_num = 0;
    std::vector<LineMkrInfo_s> line_infos_conbine = back_link_line_infos;
    for (auto info : line_infos) {
        line_infos_conbine.push_back(info);
        if (info.end_offset > (15000 + ego_offset)) {
            break;
        }
    }

#ifdef EM_COUT1
    {
        std::stringstream ss, ss2, ss3;
        ss << "line_infos_conbine.path_offset: ";
        ss2 << "line_infos_conbine[0].type: ";
        ss3 << "line_infos_conbine[0].color: ";
        for (int i = 0; i < line_infos_conbine.size(); i++) {
            ss << line_infos_conbine[i].path_offset << " ";
            ss2 << (int)line_infos_conbine[i].type << " ";
            ss3 << (int)line_infos_conbine[i].color << " ";
        }
        std::cout << ss.str() << std::endl;
        std::cout << ss2.str() << std::endl;
        std::cout << ss3.str() << std::endl;
    }

#endif
    for (int i = 0; i < line_infos_conbine.size(); i++) {
        float path_offset_m =
            static_cast<float>(line_infos_conbine[i].path_offset) / 100.0f - static_cast<float>(ego_offset) / 100.0f;
        if (i == 0) {
            if (path_offset_m <= -100) {
                side_line.etype[type_section_num].valid = true;
                side_line.etype[type_section_num].type.data_ = line_infos_conbine[i].type;
                side_line.etype[type_section_num].s = -100;
                // color
                side_line.eColor[color_section_number].valid = 1;
                side_line.eColor[color_section_number].type.data_ = LineColorMapping(line_infos_conbine[i].color);
                side_line.eColor[color_section_number].s = -100;
            } else {
                side_line.etype[type_section_num].valid = true;
                side_line.etype[type_section_num].type.data_ = line_infos_conbine[i].type;
                side_line.etype[type_section_num].s = path_offset_m;
                // color
                side_line.eColor[color_section_number].valid = true;
                side_line.eColor[color_section_number].type.data_ = LineColorMapping(line_infos_conbine[i].color);
                side_line.eColor[color_section_number].s = path_offset_m;
            }
        } else {
            if (type_section_num >= 0 && type_section_num < side_line.etype.size() - 1 &&
                line_infos_conbine[i].type != side_line.etype[type_section_num].type.data_ &&
                side_line.etype[type_section_num].valid == true) {
                type_section_num++;
                side_line.etype[type_section_num].valid = true;
                side_line.etype[type_section_num].type.data_ = line_infos_conbine[i].type;
                side_line.etype[type_section_num].s = path_offset_m;
            }
            uint8_t color_tmp = LineColorMapping(line_infos_conbine[i].color);
            if (color_section_number >= 0 && color_section_number < side_line.eColor.size() - 1 &&
                color_tmp != side_line.eColor[color_section_number].type.data_ &&
                side_line.eColor[color_section_number].valid == true) {
                color_section_number++;
                side_line.eColor[color_section_number].valid = true;
                side_line.eColor[color_section_number].type.data_ = color_tmp;
                side_line.eColor[color_section_number].s = path_offset_m;
            }
        }
    }
    // type 无变化， 填充
    if (type_section_num == 0 && side_line.etype[type_section_num].valid == true) {
        side_line.etype[type_section_num].s = 0;
    }

    if (color_section_number == 0 && side_line.eColor[color_section_number].valid == true) {
        // color 无变化， 填充
        side_line.eColor[color_section_number].s = 0;
    }

    side_line.pntSize = line_pnt_num;
    if (side_line.pntSize > 0) {
        side_line.bIsAvailable = true;
    }

    // deal virtual
    float virtual_start_s = 0;
    float virtual_end_s = 0;
    bool find_virtual_start = false;
    int virtual_line_section_num = 0;
    for (int i = 0; i < side_line.etype.size() && virtual_line_section_num < side_line.virtual_line_s.size(); i++) {
        if (side_line.etype[i].valid == true) {
            if (find_virtual_start == false && side_line.etype[i].type.data_ == 8) {
                virtual_start_s = side_line.etype[i].s;
                find_virtual_start = true;
            } else if (find_virtual_start == true && side_line.etype[i].type.data_ != 8) {
                virtual_end_s = side_line.etype[i].s;
                find_virtual_start = false;
                side_line.virtual_line_s[virtual_line_section_num].valid_ = true;
                side_line.virtual_line_s[virtual_line_section_num].start_s = virtual_start_s;
                side_line.virtual_line_s[virtual_line_section_num].end_s = virtual_end_s;
                virtual_line_section_num++;
            }
        }
    }
    // 填充最后一段的 end_s
    if (find_virtual_start == true && virtual_line_section_num < side_line.virtual_line_s.size()) {
        side_line.virtual_line_s[virtual_line_section_num].valid_ = true;
        side_line.virtual_line_s[virtual_line_section_num].start_s = virtual_start_s;
        side_line.virtual_line_s[virtual_line_section_num].end_s =
            static_cast<float>((line_pnt_num - back_point_num - 1) * 2.5);
    }

    // std::cout << __FILE__ << "," << __LINE__ << "," << " tatal side_line_pnt_num: " << line_pnt_num << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "  point_type_vec.size(): " << point_type_vec.size() <<
    // std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << "   side_line.pntSize: " <<  (int)side_line.pntSize
    // << std::endl;

    // plot
    int solid_counter = 0;
    int dot_counter = 0;
    int virtual_counter = 0;
    if (side_line.bIsAvailable == true) {
        for (int i = 0; i < side_line.pntSize && i < point_type_vec.size(); i++) {
            if (point_type_vec[i] == 2 || point_type_vec[i] == 4) {
                if (solid_counter % 4 == 0) {
                    solid_line_plot_x.push_back(side_line.linePoints[i].dX);
                    solid_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                solid_counter++;
            } else if (point_type_vec[i] == 3 || point_type_vec[i] == 5) {
                if (dot_counter % 4 == 0) {
                    dot_line_plot_x.push_back(side_line.linePoints[i].dX);
                    dot_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                dot_counter++;
            } else if ((line_direction == 2 && point_type_vec[i] == 6) ||
                       (line_direction == 1 && point_type_vec[i] == 7)) {
                if (dot_counter % 4 == 0) {
                    dot_line_plot_x.push_back(side_line.linePoints[i].dX);
                    dot_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                dot_counter++;
            } else if ((line_direction == 2 && point_type_vec[i] == 7) ||
                       (line_direction == 1 && point_type_vec[i] == 6)) {
                if (solid_counter % 4 == 0) {
                    solid_line_plot_x.push_back(side_line.linePoints[i].dX);
                    solid_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                solid_counter++;
            } else if (point_type_vec[i] == 8) {
                if (virtual_counter % 4 == 0) {
                    // virtual_line_plot_x.push_back(side_line.linePoints[i].dX);
                    // virtual_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                virtual_counter++;
            } else {
                if (solid_counter % 4 == 0) {
                    solid_line_plot_x.push_back(side_line.linePoints[i].dX);
                    solid_line_plot_y.push_back(side_line.linePoints[i].dY);
                }
                solid_counter++;
            }
        }
    }
    return true;
}

uint8_t EnvironmentModel::LineColorMapping(uint8_t color_internal) {
    uint8_t color_out = 0;
    switch (color_internal) {
        case 2:  // white
            color_out = 0;
            break;
        case 3:  // yellow
            color_out = 1;
            break;
        case 4:  // orange
            color_out = 3;
            break;
        case 5:  // red
            color_out = 2;
            break;
        case 6:             // blue
            color_out = 4;  // green
            break;
        default:
            color_out = 5;
            break;
    }

    return color_out;
}

bool EnvironmentModel::LinearInsertPoint(const EFMRefLinePoints& raw_reference_line, EFMRefLinePoints& line_out) {
    if (raw_reference_line.size() < 2) {
        return false;
    }
    line_out.clear();
    double coeff = 0.5;
    for (int i = 0; i < raw_reference_line.size(); i++) {
        if (i == 0) {
            line_out.push_back(raw_reference_line[i]);
        }
        if ((i + 1) < raw_reference_line.size() && line_out.size() > 0) {
            EFMPoint point_tmp;
            point_tmp.x = coeff * (raw_reference_line[i + 1].x - line_out.back().x) + line_out.back().x;
            point_tmp.y = coeff * (raw_reference_line[i + 1].y - line_out.back().y) + line_out.back().y;
            line_out.push_back(point_tmp);
            line_out.push_back(raw_reference_line[i + 1]);
        }
    }
    return true;
}

void EnvironmentModel::CenterLineSmooth(const EFMRefLine& path, message::efm::s_Lane_t& lane) {
    EFMRefLinePointsSection linePointsSection_cut{};
    EFMRefLinePointsSection linePointsSection_proc{};
    CutCenterLine(path.linePointsSection, linePointsSection_cut);
    ProcessCenterLine(linePointsSection_cut, linePointsSection_proc, path);
    OutputOneCenterLane0630(linePointsSection_proc, lane);
    return;
}

void EnvironmentModel::CenterLineNoSmooth(const EFMRefLinePointsSection& line_section, message::efm::s_Lane_t& lane) {
    EFMRefLinePointsSection linePointsSection_cut{};
    EFMRefLinePointsSection line_section_processed{};
    CutCenterLine(line_section, linePointsSection_cut);
    std::vector<EFMPoint> line_back{};
    LinearInsertPoint(linePointsSection_cut[0], line_back);
    line_section_processed[0].clear();
    line_section_processed[0] = line_back;

    std::vector<EFMPoint> line_front{};
    LinearInsertPoint(linePointsSection_cut[1], line_front);
    line_section_processed[1].clear();
    line_section_processed[1] = line_front;

    OutputOneCenterLane0630(line_section_processed, lane);
    return;
}

void EnvironmentModel::GetPreCycleData() {
    pre_cycle_data_.last_merge_back_links_id = scenario_judge_model_->last_merge_back_links_id();
    pre_cycle_data_.last_merge_ego_back_lanes_id = scenario_judge_model_->last_merge_ego_back_lanes_id();
    pre_cycle_data_.last_merge_side_back_lanes_id = scenario_judge_model_->last_merge_side_back_lanes_id();
    pre_cycle_data_.last_split_line_id_lane_id = candidate_lanes_model_->get_split_lind_id_lane_id_map();
}

void EnvironmentModel::SavePreCycleData(){
    pre_cycle_data_.last_map_map_counter = map_static_info_->header.cntr;
}

void EnvironmentModel::GetSubModuleOutput() {
    sub_module_output_.ego_path = extract_ref_line_->ego_path();
    sub_module_output_.left_path = extract_ref_line_->left_path();
    sub_module_output_.right_path = extract_ref_line_->right_path();

    sub_module_output_.is_y_shape_link = scenario_judge_model_->is_y_shape_link();
    sub_module_output_.is_has_virtual_lane = scenario_judge_model_->is_has_virtual_lane();
    sub_module_output_.bypass_merge_distance = scenario_judge_model_->dDistanceByPassMerge();
    sub_module_output_.odd_end_dist = scenario_judge_model_->odd_end_dist();
    sub_module_output_.in_odd_type = scenario_judge_model_->in_odd_type();
    sub_module_output_.is_has_big_curvature = scenario_judge_model_->is_has_big_curvature();
    sub_module_output_.exit_start_dist = scenario_judge_model_->exit_start_dist();
    sub_module_output_.exit_end_dist = scenario_judge_model_->exit_end_dist();
    sub_module_output_.exit_start_link_index = scenario_judge_model_->exit_start_link_index();
    sub_module_output_.entry_start_dist = scenario_judge_model_->entry_start_dist();
    sub_module_output_.entry_end_dist = scenario_judge_model_->entry_end_dist();
    sub_module_output_.geofence_segments = scenario_judge_model_->geofence_segments();
    sub_module_output_.valid_geofence_segments = scenario_judge_model_->valid_geofence_segments();
    sub_module_output_.exit_type = scenario_judge_model_->exit_type();

    sub_module_output_.fixed_lane_id = candidate_lanes_model_->fixed_lane_id();
    sub_module_output_.node_info = candidate_lanes_model_->GetNOdeINfo();
    sub_module_output_.link_id_vec = candidate_lanes_model_->link_id_vec();
    sub_module_output_.ego_path_lane_num = candidate_lanes_model_->ego_path_lane_num();
    sub_module_output_.left_path_lane_num = candidate_lanes_model_->left_path_lane_num();
    sub_module_output_.right_path_lane_num = candidate_lanes_model_->right_path_lane_num();
    sub_module_output_.left_left_path_lane_num = candidate_lanes_model_->left_left_path_lane_num();
    sub_module_output_.right_right_path_lane_num = candidate_lanes_model_->right_right_path_lane_num();
    sub_module_output_.candidate_lanes_split_nodes = candidate_lanes_model_->candidate_lanes_split_nodes();
    sub_module_output_.driveable_lane_size = candidate_lanes_model_->driveable_lane_size();
    sub_module_output_.is_contain_split_vec = candidate_lanes_model_->is_contain_split_vec();
    sub_module_output_.left_extra_info = candidate_lanes_model_->get_left_extra_info();
    sub_module_output_.ego_extra_info = candidate_lanes_model_->get_ego_extra_info();
    sub_module_output_.right_extra_info = candidate_lanes_model_->get_right_extra_info();
    sub_module_output_.right_right_extra_info = candidate_lanes_model_->get_right_right_extra_info();
    sub_module_output_.left_left_extra_info = candidate_lanes_model_->get_left_left_extra_info();

    sub_module_output_.prior_path_index = extract_ref_line_->prior_path_index();
    sub_module_output_.rest_dist = extract_ref_line_->rest_dist();
    sub_module_output_.prior_path_change_type = extract_ref_line_->prior_path_change_type();

    sub_module_output_.cur_speed = speed_limit_info_->cur_speed();
    sub_module_output_.next_speed = speed_limit_info_->next_speed();
    sub_module_output_.pntIdx = speed_limit_info_->pntIdx();
    sub_module_output_.exit_scene_speed_limit = speed_limit_info_->exit_scene_speed_limit();
    sub_module_output_.split_info_map = CandidateLanesModel::split_info_map;
    sub_module_output_.split_info_deque = CandidateLanesModel::split_info_deque;
    sub_module_output_.pre_cutin_merge_offset = candidate_lanes_model_->pre_cutin_merge_offset_vec();
}

}  // namespace framework
EARTH_MANTLE_REG_CMPT_BEGIN
EARTH_MANTLE_REG_CMPT(framework::EnvironmentModel);
EARTH_MANTLE_REG_CMPT_END
}  // namespace shell
}  // namespace earth
