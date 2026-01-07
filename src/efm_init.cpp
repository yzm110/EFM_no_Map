#include "efm_init.h"

namespace efm {
EfmInit* EfmInit::GetInstance() {
    static EfmInit instance;
    return &instance;
}

bool EfmInit::InitVariable(const MapMapMsg& map_msg, const MapPositionMsg& position_msg,
                           const MapSwitchInfoMsg& switch_info_msg, const MapRouteListMsg& map_route_list,
                           const MapGlobalDataMsg& global_data_msg, const MapDynamicMsg& map_dynamic_msg,
                           const ComInfo& com_info, const TrafficInfoMsg& traffic_info, EfmInfoMsg& efm_info,
                           MapLppInfoMsg& map_lpp_info, MapLaneMsg& map_lane, RMFMapinfoDataclose& rmf_dc,
                           EFMBusMapDataCloseInfo& efm_dc, MapPositionTmp& map_position_tmp, MapRawData& map_raw_data) {
    map_raw_data.veh_point_utm_x = 0;
    map_raw_data.veh_point_utm_y = 0;
    map_raw_data.veh_point_wgs84_x = 0;
    map_raw_data.veh_point_wgs84_y = 0;
    map_raw_data.ego_linkid_routelist_idx = 0;

    map_raw_data.veh_point_wgs84_x = map_position_tmp.Lon.Lon * 360.0 / (4294967296);
    map_raw_data.veh_point_wgs84_y = map_position_tmp.Lat.Lat * 360.0 / (4294967296);
    CommonTool::CoordinateTool::GetInstance()->wgs84toUTM2(
        map_raw_data.veh_point_wgs84_y, map_raw_data.veh_point_wgs84_x, map_raw_data.veh_point_utm_x,
        map_raw_data.veh_point_utm_y, map_raw_data.veh_point_wgs84_y, map_raw_data.veh_point_wgs84_x);

    CommonTool::CoordinateTool::GetInstance()->SetConvergenceAngle(
        map_raw_data.veh_point_wgs84_x, map_raw_data.veh_point_wgs84_y, map_position_tmp.Heading.Heading);

    if (CheckCurrentLinkIndex(map_msg, map_position_tmp) &&
        CheckCurrentRoutingIndex(map_msg, map_position_tmp, map_route_list, map_raw_data.ego_linkid_routelist_idx)) {
        InitVariableZTEXT(true, map_position_tmp, map_route_list, map_lpp_info);
    } else {
        InitVariableZTEXT(false, map_position_tmp, map_route_list, map_lpp_info);
        return false;
    }

    memset(&efm_info, 0, sizeof(efm_info));
    memset(&rmf_dc, 0, sizeof(rmf_dc));
    map_lane = {};
    map_lane.lanes[0].enable_flag = 0;
    map_lane.lanes[1].enable_flag = 0;
    map_lane.lanes[2].enable_flag = 0;
    map_lane.lanes[3].enable_flag = 0;
    map_lane.lanes[4].enable_flag = 0;
    map_lane.lanes[5].enable_flag = 0;
    map_lane.Position.mapIsValid = 0;

    map_lpp_info = {};
    map_lpp_info.bIsActive = 0;
    efm_info.SpdLmtInfo.MapSpdLimFirst = 99.0f;
    efm_info.SpdLmtInfo.DstToMapSpdLimFirst = 1.0f;
    efm_info.SpdLmtInfo.MapSpdLimSec = 99.0f;
    efm_info.SpdLmtInfo.DstToMapSpdLimSec = 1.0f;
    efm_info.LocErrorSts.data = 0;
    map_lpp_info.mRampInfo.startPoint.dX = -255;
    map_lpp_info.mRampInfo.endPoint.dX = -255;
    map_lpp_info.mRampInfo.dDistance = -255;
    map_lpp_info.mRampInfo.OnRampStartPoint.dX = -255;
    map_lpp_info.mRampInfo.OnRampEndPoint.dX = -255;
    map_lpp_info.mRampInfo.dOnRampDistance = -255;
    map_lpp_info.mMapOdd.eInOdd.data_ = 0;
    map_lpp_info.header.err_code = 0;
    map_lpp_info.header.cntr = 0;
    map_lpp_info.header.timestamp = 0;
    map_lpp_info.mRefPaths[1].dNodeOffset = 5000.0;
    map_lpp_info.dDistanceByPassMerge = 5000.0;
    map_lpp_info.mSpecialSit.dStartDistance = 5000;
    map_lpp_info.mSpecialSit.eSpecSitType.data_ = 0;
    map_lpp_info.mSpecialSit.dEndDistance = 5000;
    map_lpp_info.mRefPaths[2].bProbabilityLeft = 0;
    return true;
}

bool EfmInit::CheckCurrentLinkIndex(const MapMapMsg& map_msg, const MapPositionTmp& map_position_tmp) {
    uint32_t cur_link_id = map_position_tmp.LinkId;
    int cur_link_index_ = -1;
    for (int i = 0; i < map_msg.LinkInfos.LinkInfos.size(); i++) {
        uint32_t searched_link_id = map_msg.LinkInfos.LinkInfos[i].InstanceId.InstanceId;
        if (cur_link_id == searched_link_id) {
            cur_link_index_ = i;
            break;
        }
    }

    if (cur_link_index_ < 0) {
        return false;
    }
    return true;
}

// TODO 1：静态图里面的非自车当前所在的linkid,可用该link的pathID和position的pathID来判断
// TODO 2: 自车当前linkid不在route list、static里面，做到error code
bool EfmInit::CheckCurrentRoutingIndex(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                                       const MapRouteListMsg& map_route_list, int& cur_rp_index) {
    uint32_t cur_link_id = map_position_tmp.LinkId;
    cur_rp_index = -1;
    for (int i = 0; i < map_route_list.LinkIds.LinkIds.size(); i++) {
        if (cur_link_id == map_route_list.LinkIds.LinkIds[i].LinkId) {
            cur_rp_index = i;
            break;
        }
    }
    if (cur_rp_index < 0) {
        return false;
    }
    //
    for (int i = 0; i < map_msg.LinkInfos.LinkInfos.size(); i++) {
        if (cur_link_id == map_msg.LinkInfos.LinkInfos[i].InstanceId.InstanceId) {
            size_t lane_size = map_msg.LinkInfos.LinkInfos[i].LaneInfos.LaneInfos.size();
            if (lane_size == 0) {
                return false;
            }
        }
    }
    return true;
}

bool EfmInit::InitVariableZTEXT(bool ztext_flag, const MapPositionTmp& map_position_tmp,
                                const MapRouteListMsg& map_route_list, MapLppInfoMsg& map_lpp_info) {
    double ego_wgs84_x = map_position_tmp.Lon.Lon * 360.0 / (4294967296);
    double ego_wgs84_y = map_position_tmp.Lat.Lat * 360.0 / (4294967296);
    ZTEXT("EFM_INFO", "cur_link_id: ", 15, 27, "cur_link_id: {}", map_position_tmp.LinkId);
    // ZTEXT("EFM_INFO", "position_timestamp: ", 18, 30, "position_stamp: {}", map_position_tmp.PositionTimeStamp);
    ZTEXT("EFM_INFO", "map_rt_list.size: ", 14, 53, "map_rt_list.size: {}", map_route_list.LinkIds.LinkIds.size());
    ZTEXT("EFM_INFO", "pos_lat: ", 14, 50, "pos_lat: {}", ego_wgs84_y);
    ZTEXT("EFM_INFO", "pos_lon: ", 14, 47, "pos_lon: {}", ego_wgs84_x);
    ZTEXT("EFM_INFO", "is_parallel_line: ", -30, -50, "is_parallel_line: {}", 0);

    if (false == ztext_flag) {
        // ZTEXT("EFM_INFO", "bIsActive: ", -30, 38, "bIsActive: {}", map_lpp_info.bIsActive);
        ZTEXT("EFM_INFO", "cur_lane_speed: ", 30, 18, " ");
        ZTEXT("EFM_INFO", "exit_start_dist_: ", 30, 30, " ");
        ZTEXT("EFM_INFO", "exit_end_dist_: ", 30, 33, " ");
        ZTEXT("EFM_INFO", "speed_limit[0].value: ", 21, 36, " ");
        ZTEXT("EFM_INFO", "speed_limit[0].dist: ", 20, 39, " ");
        ZTEXT("EFM_INFO", "speed_limit[1].value: ", 21, 42, " ");
        ZTEXT("EFM_INFO", "speed_limit[1].dist: ", 20, 45, " ");
        ZTEXT("EFM_INFO", "ego_path_merge_dist: ", 80, 30, " ");
        ZTEXT("EFM_INFO", "ego_path_merge_type: ", 80, 33, " ");
        ZTEXT("EFM_INFO", "left_path_merge_dist: ", 80, 36, " ");
        ZTEXT("EFM_INFO", "left_path_merge_type: ", 80, 39, " ");
        ZTEXT("EFM_INFO", "right_path_merge_dist: ", 80, 42, " ");
        ZTEXT("EFM_INFO", "right_path_merge_type: ", 80, 45, " ");
        ZTEXT("EFM_INFO", "map_lane_speed: ", 30, 18, " ");
        ZTEXT("EFM_INFO", "entry_dist_: ", 30, 48, " ");
        ZTEXT("EFM_INFO", "entry_end_dist_: ", 30, 51, " ");
        ZTEXT("EFM_INFO", "in_odd: ", -38, -36, "in_odd: {}", map_lpp_info.mMapOdd.eInOdd.data_);
        ZTEXT("EFM_INFO", "odd_end_dist: ", -30, -39, "odd_end_dist: {}", map_lpp_info.mMapOdd.dDisToOdd);
        ZTEXT("EFM_INFO", "out_odd_reason: ", -30, -42, "out_odd_reason: {}", map_lpp_info.mMapOdd.eOddReason.data_);
        ZTEXT("EFM_INFO", "bIsSetNav: ", -30, 53, "bIsSetNav: {}", map_lpp_info.mNavInfo.bIsSetNav);
        ZTEXT("EFM_INFO", "bNavIsYaw: ", -30, 56, "bNavIsYaw: {}", map_lpp_info.mNavInfo.bNavIsYaw);
        ZTEXT("EFM_INFO", "SwtLaneReason: ", -30, 59, " ");
        ZTEXT("EFM_INFO", "lanetype1: ", 14, -20, " ");
        ZTEXT("EFM_INFO", "lanetype1dis: ", 14, -23, " ");
        ZTEXT("EFM_INFO", "lanetype2: ", 14, -26, " ");
        ZTEXT("EFM_INFO", "lanetype2dis: ", 14, -29, " ");
        ZTEXT("EFM_INFO", "leLaneIsAvl: ", 14, -38, " ");
        ZTEXT("EFM_INFO", "egoLaneIsAvl: ", 21, -41, " ");
        ZTEXT("EFM_INFO", "riLaneIsAvl: ", 14, -44, " ");
        ZTEXT("EFM_INFO", "egoLaneId: ", 14, -47, " ");
        ZTEXT("EFM_INFO", "drvLaneSize: ", 14, -50, " ");
        ZTEXT("EFM_INFO", "roadClass: ", 14, -53, " ");
        ZTEXT("EFM_INFO", "errorcode_tolpp: ", -30, 50, "errorcode_tolpp: {}", map_lpp_info.header.err_code);
        ZTEXT("EFM_INFO", "bIsActive_new: ", -35, 20, "bIsActive_new: {}", map_lpp_info.bIsActive);
        ZTEXT("EFM_INFO", "cur_speed_final: ", 140, 55, " ");
        ZTEXT("EFM_INFO", "next_speed_final: ", 140, 58, " ");
        ZTEXT("EFM_INFO", "next_speed_dist_final: ", 140, 61, " ");

        std::vector<double> plot_right_right_solid_line_plot_x{};
        std::vector<double> plot_right_right_solid_line_plot_y{};
        std::vector<double> plot_right_right_dot_line_plot_x{};
        std::vector<double> plot_right_right_dot_line_plot_y{};
        std::vector<double> plot_right_left_solid_line_plot_x{};
        std::vector<double> plot_right_left_solid_line_plot_y{};
        std::vector<double> plot_right_left_dot_line_plot_x{};
        std::vector<double> plot_right_left_dot_line_plot_y{};

        std::vector<double> plot_ego_right_solid_line_plot_x{};
        std::vector<double> plot_ego_right_solid_line_plot_y{};
        std::vector<double> plot_ego_right_dot_line_plot_x{};
        std::vector<double> plot_ego_right_dot_line_plot_y{};
        std::vector<double> plot_ego_left_solid_line_plot_x{};
        std::vector<double> plot_ego_left_solid_line_plot_y{};
        std::vector<double> plot_ego_left_dot_line_plot_x{};
        std::vector<double> plot_ego_left_dot_line_plot_y{};

        std::vector<double> plot_left_right_solid_line_plot_x{};
        std::vector<double> plot_left_right_solid_line_plot_y{};
        std::vector<double> plot_left_right_dot_line_plot_x{};
        std::vector<double> plot_left_right_dot_line_plot_y{};
        std::vector<double> plot_left_left_solid_line_plot_x{};
        std::vector<double> plot_left_left_solid_line_plot_y{};
        std::vector<double> plot_left_left_dot_line_plot_x{};
        std::vector<double> plot_left_left_dot_line_plot_y{};
        std::vector<double> kerb_leftline_points_x{};
        std::vector<double> kerb_leftline_points_y{};
        std::vector<double> kerb_rightline_points_x{};
        std::vector<double> kerb_rightline_points_y{};
        std::vector<double> ego_plot_x{};         // for plot
        std::vector<double> ego_plot_y{};         // for plot
        std::vector<double> left_plot_x{};        // for plot
        std::vector<double> left_plot_y{};        // for plot
        std::vector<double> right_plot_x{};       // for plot
        std::vector<double> right_plot_y{};       // for plot
        std::vector<double> global_ref_plot_x{};  // for plot
        std::vector<double> global_ref_plot_y{};  // for plot
        std::vector<double> emer_right_right_solid_line_x{};
        std::vector<double> emer_right_right_solid_line_y{};
        std::vector<double> emer_right_right_dot_line_x{};
        std::vector<double> emer_right_right_dot_line_y{};

        std::vector<double> emer_right_left_solid_line_x{};
        std::vector<double> emer_right_left_solid_line_y{};
        std::vector<double> emer_right_left_dot_line_x{};
        std::vector<double> emer_right_left_dot_line_y{};   
        std::vector<double> emer_right_center_x{};
        std::vector<double> emer_right_center_y{};   
        std::vector<double> emer_left_right_solid_line_x{};
        std::vector<double> emer_left_right_solid_line_y{};
        std::vector<double> emer_left_right_dot_line_x{};
        std::vector<double> emer_left_right_dot_line_y{};
        std::vector<double> emer_left_left_solid_line_x{};
        std::vector<double> emer_left_left_solid_line_y{};
        std::vector<double> emer_left_left_dot_line_x{};
        std::vector<double> emer_left_left_dot_line_y{};
        std::vector<double> emer_left_center_x{};
        std::vector<double> emer_left_center_y{};  
        ZPLOTXYF("EFM_INFO", "-blue4", plot_right_right_solid_line_plot_x, plot_right_right_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_right_right_dot_line_plot_x, plot_right_right_dot_line_plot_y);
        ZPLOTXYF("EFM_INFO", "-blue4", plot_right_left_solid_line_plot_x, plot_right_left_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_right_left_dot_line_plot_x, plot_right_left_dot_line_plot_y);

        ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_right_solid_line_plot_x, plot_ego_right_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_right_dot_line_plot_x, plot_ego_right_dot_line_plot_y);
        ZPLOTXYF("EFM_INFO", "-blue4", plot_ego_left_solid_line_plot_x, plot_ego_left_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_ego_left_dot_line_plot_x, plot_ego_left_dot_line_plot_y);

        ZPLOTXYF("EFM_INFO", "-blue4", plot_left_right_solid_line_plot_x, plot_left_right_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_left_right_dot_line_plot_x, plot_left_right_dot_line_plot_y);
        ZPLOTXYF("EFM_INFO", "-blue4", plot_left_left_solid_line_plot_x, plot_left_left_solid_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--blue4", plot_left_left_dot_line_plot_x, plot_left_left_dot_line_plot_y);
        ZPLOTXYF("EFM_INFO", "--orange2", ego_plot_x, ego_plot_y);
        ZPLOTXYF("EFM_INFO", "--red2", left_plot_x, left_plot_y);
        ZPLOTXYF("EFM_INFO", "--pink2", right_plot_x, right_plot_y);
        ZPLOTXYF("EFM_INFO", ".black8", global_ref_plot_x, global_ref_plot_y);

        ZPLOTXYF("EFM_INFO", "-blue4", emer_right_left_solid_line_x, emer_right_left_solid_line_y);
        ZPLOTXYF("EFM_INFO", "--blue4", emer_right_left_dot_line_x, emer_right_left_dot_line_y);
        ZPLOTXYF("EFM_INFO", "-blue4", emer_left_right_solid_line_x, emer_left_right_solid_line_y);
        ZPLOTXYF("EFM_INFO", "--blue4", emer_left_right_dot_line_x, emer_left_right_dot_line_y);
        ZPLOTXYF("EFM_INFO", "--orange2", emer_left_center_x, emer_left_center_y);
        ZPLOTXYF("EFM_INFO", "--red2", emer_right_center_x, emer_right_center_y);
    }

    return true;
}

bool EfmInit::ChangePositionLaneID(const MapMapMsg& map_msg, const uint8_t& lane_change_state,
            const PreCycleData& pre_cycle_data, MapPositionTmp& map_position_tmp,
            const MapRawDataMap& map_raw_data_map, MainFuncStaticVar* main_func_static_var) {
    double ego_wgs84_x = map_position_tmp.Lon.Lon * 360.0 / (4294967296);
    double ego_wgs84_y = map_position_tmp.Lat.Lat * 360.0 / (4294967296);
    if (0 == map_position_tmp.LinkId || 0 == map_position_tmp.LaneId) {
        return true;
    }

    message::map_map::s_LinkInfo_t link_infos;
    // TODO GetLinkInfos写成公有函数
    if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(map_msg, map_raw_data_map.link_id_index_lane_info_map,
                                                                 map_position_tmp.LinkId, link_infos)) {
        // static data none
        return true;
    }

    std::vector<uint8_t> split_side_lanes;
    std::vector<EFMPoint> prior_line_points;
    EFMPoint ego_position_QCJ02;
    uint16_t index_prior;
    EFMPoint foot_point_prior;
    double distance_prior;
    bool find_insile_prior = false;
    double ego_x, ego_y, ego_z;
    double priorpoint_x, priorpoint_y, priorpoint_z;
    double priorpoint_ego_x, priorpoint_ego_y;
    EFMPoint ego_point, prior_point;
    ego_position_QCJ02.x = ego_wgs84_x;
    ego_position_QCJ02.y = ego_wgs84_y;

    uint32_t lanes_length = map_position_tmp.PathOffset - link_infos.PathOffset.PathOffset;
    uint32_t change_position_range = CHANGE_POSITION_RANGE;
    uint8_t continue_lane_id = 0;
    uint8_t last_ego_lane_num = 0;

    // std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;
    std::vector<uint32_t> last_merge_back_links_id = pre_cycle_data.last_merge_back_links_id;
    std::vector<uint8_t> last_merge_ego_back_lanes_id = pre_cycle_data.last_merge_ego_back_lanes_id;
    std::vector<uint8_t> last_merge_side_back_lanes_id = pre_cycle_data.last_merge_side_back_lanes_id;

    if (true == GetVirtualMerge(map_msg, pre_cycle_data, map_raw_data_map, map_position_tmp, last_merge_back_links_id,
                                last_merge_ego_back_lanes_id, last_merge_side_back_lanes_id, last_ego_lane_num)) {
        map_position_tmp.LaneId = last_ego_lane_num;
    }
    // std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;

    if (link_infos.PathOffset.PathOffset > map_position_tmp.PathOffset ||
        map_position_tmp.PathOffset > (link_infos.PathOffset.PathOffset + CHANGE_POSITION_RANGE)) {
        return true;
    }

    uint32_t virtual_line_split_link_id = 0;
    if (false == GetSplitSideLaneV2(map_msg, pre_cycle_data, map_position_tmp, map_raw_data_map, continue_lane_id,
                                    lanes_length, change_position_range, virtual_line_split_link_id)) {
    return true;
    }
    bool is_virtual_line_chg_lane_id = false;
    is_virtual_line_chg_lane_id = (map_position_tmp.LaneId != continue_lane_id) ? true : false;
    map_position_tmp.LaneId = continue_lane_id;

    if (main_func_static_var->virtual_line_split_link_id != virtual_line_split_link_id &&
        virtual_line_split_link_id != 0) {
        main_func_static_var->virtual_line_split_link_id = virtual_line_split_link_id;
    }

    // std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;
    lanes_length = map_position_tmp.PathOffset - link_infos.PathOffset.PathOffset;
    change_position_range = CHANGE_POSITION_RANGE2;
    if (false == GetSplitSideLane(map_msg, map_position_tmp, map_raw_data_map, map_position_tmp.LinkId,
                                map_position_tmp.LaneId, split_side_lanes, lanes_length, change_position_range)) {
        return true;
    }
    // std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;


    message::map_map::s_LinkInfo_t virtual_line_split_link_infos{};
    if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(map_msg, map_raw_data_map.link_id_index_lane_info_map,
                                    main_func_static_var->virtual_line_split_link_id, virtual_line_split_link_infos)) {
        // static data none
        virtual_line_split_link_infos.PathOffset.PathOffset = 0;
    }

    bool virtual_line_split_link_is_invalid = false;
    // std::cout << __FILE__ << __LINE__ << "virtual_line_split_link_id: "
    //         << main_func_static_var->virtual_line_split_link_id << std::endl;
    for (const auto& geofence : map_msg.GeoFences.GeoFences) {
        // std::cout << __FILE__ << __LINE__ << "geofence.InstanceId_.InstanceId: " << geofence.InstanceId_.InstanceId
        //     << std::endl;
        // std::cout << __FILE__ << __LINE__ << "geofence.GeoFenceType.data: " <<
        //     int(geofence.GeoFenceType.data) << std::endl;
        if (main_func_static_var->virtual_line_split_link_id == geofence.InstanceId_.InstanceId
            && geofence.GeoFenceType.data == 4) {
            virtual_line_split_link_is_invalid = true;
            break;
        }
    }

    uint32_t lanes_length_v3 = map_position_tmp.PathOffset - virtual_line_split_link_infos.PathOffset.PathOffset;
    // std::cout << __FILE__ << __LINE__ << "map_position_tmp.PathOffset: " << map_position_tmp.PathOffset << std::endl;
    // std::cout << __FILE__ << __LINE__ << "virtual_line_split_link_infos.PathOffset.PathOffset: "
    //     << virtual_line_split_link_infos.PathOffset.PathOffset << std::endl;
    // std::cout << __FILE__ << __LINE__ << "main_func_static_var->virtual_line_split_link_id: "
    //     << main_func_static_var->virtual_line_split_link_id << std::endl;
    // std::cout << __FILE__ << __LINE__ << "lanes_length_v3: " << lanes_length_v3 << std::endl;
    // std::cout << __FILE__ << __LINE__ << "lane_change_state: " << int(lane_change_state) << std::endl;
    if (lane_change_state == 16 && lanes_length_v3 < 70 * 100 &&
        main_func_static_var->last_path_id == map_position_tmp.PathId &&
        false == virtual_line_split_link_is_invalid) {
        uint32_t lanes_length_tmp = map_position_tmp.PathOffset - link_infos.PathOffset.PathOffset;
        // std::cout << __FILE__ << __LINE__ << "lanes_length_tmp: " << lanes_length_tmp << std::endl;
        if (false == GetSplitSideLaneV3(map_msg, pre_cycle_data, map_position_tmp, map_raw_data_map, continue_lane_id,
                                    lanes_length_tmp)) {
            return true;
        }
        map_position_tmp.LaneId = continue_lane_id;
    //  std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;
    } else {
        prior_line_points.clear();
        GetOneCenterLine(map_msg, map_position_tmp.LaneId, link_infos, prior_line_points);
        CommonTool::CoordinateTool::GetInstance()->GetNearestPoint(prior_line_points, ego_position_QCJ02, index_prior,
                                                                foot_point_prior, distance_prior, find_insile_prior);
        CommonTool::CoordinateTool::GetInstance()->wgs84toUTM2(ego_wgs84_x, ego_wgs84_y, ego_x, ego_y, ego_wgs84_x,
                                                            ego_wgs84_x);
        ego_point = {ego_x, ego_y};
        CommonTool::CoordinateTool::GetInstance()->wgs84toUTM2(foot_point_prior.x, foot_point_prior.y, priorpoint_x,
                                                            priorpoint_y, ego_wgs84_x, ego_wgs84_x);
        prior_point = {priorpoint_x, priorpoint_y};
        CommonTool::CoordinateTool::GetInstance()->UTM2EgoVehicle(
            priorpoint_x, priorpoint_y, ego_x, ego_y, map_position_tmp.Heading.Heading, priorpoint_ego_x,
            priorpoint_ego_y);
        // std::cout << "distance_prior: " << distance_prior << std::endl;
        // std::cout << "split_side_lanes.size(): " << split_side_lanes.size() << std::endl;
        if (distance_prior > 1.5 && !split_side_lanes.empty() && true == is_virtual_line_chg_lane_id) {
            for (int i = 0; i < split_side_lanes.size(); i++) {
                double tmppoint_x, tmppoint_y, tmppoint_z;
                double tmppoint_ego_x, tmppoint_ego_y;
                EFMPoint tmp_point;
                std::vector<EFMPoint> tmp_line_points;
                uint16_t index_tmp;
                EFMPoint foot_point_tmp;
                double cross = 0;
                double distance_tmp;
                bool find_insile_tmp = false;
                GetOneCenterLine(map_msg, split_side_lanes[i], link_infos, tmp_line_points);
                CommonTool::CoordinateTool::GetInstance()->GetNearestPoint(tmp_line_points, ego_position_QCJ02,
                                        index_tmp, foot_point_tmp, distance_tmp, find_insile_tmp);

                CommonTool::CoordinateTool::GetInstance()->wgs84toUTM2(foot_point_tmp.x, foot_point_tmp.y, tmppoint_x,
                                                                    tmppoint_y, ego_wgs84_x, ego_wgs84_x);
                tmp_point = {tmppoint_x, tmppoint_y};
                EFMPoint vector1 = {prior_point.x - ego_point.x, prior_point.y - ego_point.y};
                EFMPoint vector2 = {tmp_point.x - ego_point.x, tmp_point.y - ego_point.y};
                CommonTool::CoordinateTool::GetInstance()->UTM2EgoVehicle(
                    tmppoint_x, tmppoint_y, ego_x, ego_y, map_position_tmp.Heading.Heading,
                    tmppoint_ego_x, tmppoint_ego_y);
                CommonTool::CoordinateTool::GetInstance()->crossProduct(vector1, vector2, cross);

                // std::cout << "split_side_lanes[i]: " << int(split_side_lanes[i]) << std::endl;
                // std::cout << "tmppoint_ego_y: " << tmppoint_ego_y << std::endl;
                // std::cout << "priorpoint_ego_y: " << priorpoint_ego_y << std::endl;
                // std::cout << "distance_prior: " << distance_prior << std::endl;
                // std::cout << "distance_tmp: " << distance_tmp << std::endl;
                // (distance_tmp + distance_prior > 1.0) &&
                if (((distance_tmp + distance_prior > 1.2) && distance_tmp < distance_prior &&
                    tmppoint_ego_y * priorpoint_ego_y < 0) ||
                    (distance_prior > distance_tmp + 2) || (tmppoint_ego_y * priorpoint_ego_y > 0 &&
                    distance_tmp < distance_prior)) {
                    map_position_tmp.LaneId = split_side_lanes[i];
                    // std::cout << "distance_tmp: " << distance_tmp << std::endl;
                }
            }
        }
        // std::cout << __FILE__ << __LINE__ << "map_position_tmp.LaneId: " << int(map_position_tmp.LaneId) << std::endl;
    }

    if (main_func_static_var->last_path_id != map_position_tmp.PathId &&
        map_position_tmp.PathId != 0) {
        main_func_static_var->last_path_id = map_position_tmp.PathId;
    }

    return true;
}

bool EfmInit::GetVirtualMerge(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                              const MapRawDataMap& map_raw_data_map, MapPositionTmp& map_position_tmp,
                              const std::vector<uint32_t>& merge_back_links_id,
                              const std::vector<uint8_t>& merge_ego_back_lanes_id,
                              const std::vector<uint8_t>& merge_side_back_lanes_id, uint8_t& last_ego_lane_num) {
    last_ego_lane_num = 0;
    uint32_t cur_link_id = map_position_tmp.LinkId;
    uint8_t cur_lane_id = map_position_tmp.LaneId;
    // std::cout << __FILE__ << __LINE__  << "merge_back_links_id.size(): " << int(merge_back_links_id.size()) <<
    // std::endl; std::cout << __FILE__ << __LINE__  << "merge_ego_back_lanes_id.size(): " <<
    // int(merge_ego_back_lanes_id.size()) << std::endl; std::cout << __FILE__ << __LINE__  <<
    // "merge_side_back_lanes_id.size(): " << int(merge_side_back_lanes_id.size()) << std::endl; std::cout << __FILE__
    // << __LINE__ << "cur_link_id: " << cur_link_id << std::endl; std::cout << "merge_back_links_id:[ "; for(const
    // auto& link_id : merge_back_links_id){
    //     std::cout << link_id << " ";

    // }
    // std::cout << "]" << std::endl;
    if (false == merge_back_links_id.empty() && merge_back_links_id.size() == merge_ego_back_lanes_id.size() &&
        merge_back_links_id.size() == merge_side_back_lanes_id.size() &&
        std::find(merge_back_links_id.begin(), merge_back_links_id.end(), cur_link_id) != merge_back_links_id.end()) {
        bool is_get_virtual_merge = false;
        for (int link_idx = merge_back_links_id.size() - 1; link_idx >= 0; link_idx--) {
            /*
            1：从后往前开始遍历防止边遍历边擦除导致的异常
            2：保留自车离两侧中心线都小于0.5m的link及lane_id
            */
            uint32_t ego_center_line_idx = 0, side_center_line_idx = 0, tmp_offset = 0;
            EFMPoint vehicle_point{0.0, 0.0};
            double ego_center_nearest_distance = 0.0, side_center_nearest_distance = 0.0;
            std::vector<message::map_map::s_GeometryPoint_t> ego_center_line_points_wgs{},
                side_center_line_points_wgs{};
            EFMRefLinePoints ego_center_line_points_ego{}, side_center_line_points_ego{};
            message::map_map::s_LinkInfo_t cur_search_link_infos;
            if (false ==
                efm::MapCommonTool::GetInstance()->GetLinkInfos(map_msg, map_raw_data_map.link_id_index_lane_info_map,
                                                                merge_back_links_id[link_idx], cur_search_link_infos)) {
                return false;
            }
            if (false == efm::MapCommonTool::GetInstance()->GetLaneCenterLine(
                             cur_search_link_infos, merge_ego_back_lanes_id[link_idx], ego_center_line_idx) ||
                false == efm::MapCommonTool::GetInstance()->GetLaneCenterLine(
                             cur_search_link_infos, merge_side_back_lanes_id[link_idx], side_center_line_idx)) {
                return false;
            }

            if (false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_msg, ego_center_line_idx,
                                                                            ego_center_line_points_wgs) ||
                false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_msg, side_center_line_idx,
                                                                            side_center_line_points_wgs)) {
                return false;
            }
            CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                ego_center_line_points_wgs, ego_center_line_points_ego, map_position_tmp,
                static_cast<double>(map_position_tmp.Heading.Heading));
            CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                side_center_line_points_wgs, side_center_line_points_ego, map_position_tmp,
                static_cast<double>(map_position_tmp.Heading.Heading));
            CommonTool::CoordinateTool::GetInstance()->GetNearestPointV2(ego_center_line_points_ego, vehicle_point,
                                                                         ego_center_nearest_distance);
            CommonTool::CoordinateTool::GetInstance()->GetNearestPointV2(side_center_line_points_ego, vehicle_point,
                                                                         side_center_nearest_distance);
            // std::cout <<  __FILE__ << __LINE__ << "cur_search_link: " << merge_back_links_id[link_idx] << std::endl;
            // std::cout << "ego_center_nearest_distance: " << ego_center_nearest_distance << std::endl;
            // std::cout << "side_center_nearest_distance: " << side_center_nearest_distance << std::endl;
            double distance_diff = std::abs(ego_center_nearest_distance - side_center_nearest_distance);
            if (distance_diff < 0.6 && ego_center_nearest_distance < 1.2 && side_center_nearest_distance < 1.2) {
                is_get_virtual_merge = true;
                break;
            }
        }

        if (true == is_get_virtual_merge) {
            for (size_t link_idx = 0; link_idx < merge_back_links_id.size(); link_idx++) {
                //当前link_id等于上一次merge的link_id，且当前lane_id不等于上一次merge的lane_id,那么修改绑路
                if (cur_link_id == merge_back_links_id[link_idx] && cur_lane_id != merge_ego_back_lanes_id[link_idx]) {
                    // std::cout << __FILE__ << __LINE__ << "cur_link_id: " << cur_link_id << std::endl;
                    // std::cout << __FILE__ << __LINE__ << "cur_lane_id: " << int(cur_lane_id) << std::endl;
                    // std::cout << __FILE__ << __LINE__ << "merge_back_links_id: " << merge_back_links_id[link_idx] <<
                    // std::endl;
                    last_ego_lane_num = merge_ego_back_lanes_id[link_idx];
                    return true;
                }
            }
        }
    }
    return false;
}

bool EfmInit::GetOneCenterLine(const MapMapMsg& map_msg, const uint8_t& lane_num,
                               const message::map_map::s_LinkInfo_t& link_infos,
                               std::vector<EFMPoint>& center_line_points) {
    for (auto& lane_info : link_infos.LaneInfos.LaneInfos) {
        if (lane_info.LaneNum.LaneNum == lane_num) {
            uint32_t cente_line = lane_info.Centeline.Centeline;
            for (auto& lineobj : map_msg.LinearObjects.LinearObjects) {
                if (lineobj.IDLinearObject.IDLinearObject == cente_line) {
                    int point_max_size = (lineobj.PointCount.PointCount < lineobj.GeometryPoints.GeometryPoints.size())
                                             ? lineobj.PointCount.PointCount
                                             : lineobj.GeometryPoints.GeometryPoints.size();
                    for (int point_idx = 0; point_idx < point_max_size; point_idx++) {
                        double lon =
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                            360.0 / (pow(2, 32));
                        double lat =
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                            360.0 / (pow(2, 32));
                        center_line_points.push_back({lon, lat});
                    }
                }
            }
            break;
        }
    }
    return true;
}

bool EfmInit::GetSplitSideLaneV2(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                                 MapPositionTmp& map_position_tmp, const MapRawDataMap& map_raw_data_map,
                                 uint8_t& continue_lane_id, uint32_t& lanes_length, uint32_t& change_position_range,
                                 uint32_t& virtual_line_split_link_id) {
    continue_lane_id = map_position_tmp.LaneId;
    if (0 == map_position_tmp.LaneId) {
        return false;
    }

    uint32_t cur_link_id = map_position_tmp.LinkId;
    uint32_t cur_lane_id = map_position_tmp.LaneId;
    uint32_t left_cur_lane_id = map_position_tmp.LaneId + 1;
    uint32_t right_cur_lane_id = map_position_tmp.LaneId - 1;
    bool left_lane_flag = true;
    bool right_lane_flag = true;
    bool split_exits_flag = false;
    int count = 0;
    while (lanes_length < CHANGE_POSITION_RANGE && count < 50) {
        count++;
        if (false == left_lane_flag && false == right_lane_flag) {
            break;
        }

        message::map_map::s_LinkInfo_t cur_link_infos;
        if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(
                         map_msg, map_raw_data_map.link_id_index_lane_info_map, cur_link_id, cur_link_infos)) {
            // static data none
            break;
        }

        uint8_t left_virtual_line_case = 0;
        uint8_t right_virtual_line_case = 0;
        efm::MapCommonTool::GetInstance()->IsVirtuallyLine(map_msg, map_position_tmp, cur_link_infos, left_cur_lane_id,
                                                           cur_lane_id, true, left_virtual_line_case);
        efm::MapCommonTool::GetInstance()->IsVirtuallyLine(map_msg, map_position_tmp, cur_link_infos, cur_lane_id,
                                                           right_cur_lane_id, false, right_virtual_line_case);
        // std::cout << __FILE__ << __LINE__ << "left_virtual_line_case: " << int(left_virtual_line_case) << std::endl;
        // std::cout << __FILE__ << __LINE__ << "right_virtual_line_case: " << int(right_virtual_line_case) << std::endl;
        if (left_virtual_line_case == 2 || right_virtual_line_case == 2) {
            ZTEXT("EFM_INFO", "is_parallel_line: ", -30, -50, "is_parallel_line: {}", 1);
        }else{
            ZTEXT("EFM_INFO", "is_parallel_line: ", -30, -50, "is_parallel_line: {}", 0);
        }
        

        if (left_virtual_line_case != 1 && left_lane_flag) {
            left_lane_flag = false;
        }

        if (right_virtual_line_case != 1 && right_lane_flag) {
            right_lane_flag = false;
        }

        uint32_t back_spit_link = 0;
        uint32_t back_spit_lane = 0;
        efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                           cur_lane_id, back_spit_link, back_spit_lane);

        uint32_t left_back_spit_link = 0;
        uint32_t left_back_spit_lane = 0;
        if (left_lane_flag) {
            efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                               left_cur_lane_id, left_back_spit_link,
                                                               left_back_spit_lane);
            if (left_back_spit_link == back_spit_link && left_back_spit_link > 0) {
                if (back_spit_lane == left_back_spit_lane) {
                    right_lane_flag = false;
                    split_exits_flag = true;
                    virtual_line_split_link_id = back_spit_link;
                    uint8_t cur_lane_id_tran;
                    // std::cout << __FILE__ << __LINE__ << "left_cur_lane_id: " << int(left_cur_lane_id) << std::endl;
                    GetLaneTran(pre_cycle_data, cur_link_infos, cur_lane_id, cur_lane_id_tran);
                    // uint8_t cur_left_lane_id_tran;
                    // GetLaneTran(pre_cycle_data, cur_link_infos, left_cur_lane_id, cur_left_lane_id_tran);
                    if (1 == cur_lane_id_tran || 99 == cur_lane_id_tran) {
                        left_lane_flag = false;
                    }
                }
            } else {
                left_lane_flag = false;
            }
        }

        uint32_t right_back_spit_link = 0;
        uint32_t right_back_spit_lane = 0;
        if (right_lane_flag) {
            efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                               right_cur_lane_id, right_back_spit_link,
                                                               right_back_spit_lane);
            if (right_back_spit_link == back_spit_link && right_back_spit_link > 0) {
                if (back_spit_lane == right_back_spit_lane) {
                    left_lane_flag = false;
                    split_exits_flag = true;
                    virtual_line_split_link_id = back_spit_link;
                    uint8_t cur_lane_id_tran;
                    // std::cout << __FILE__ << __LINE__ << "right_cur_lane_id: " << int(right_cur_lane_id) <<
                    // std::endl;
                    GetLaneTran(pre_cycle_data, cur_link_infos, cur_lane_id, cur_lane_id_tran);
                    // uint8_t cur_right_lane_id_tran;
                    // GetLaneTran(pre_cycle_data, cur_link_infos, right_cur_lane_id, cur_right_lane_id_tran);
                    if (1 == cur_lane_id_tran || 99 == cur_lane_id_tran) {
                        right_lane_flag = false;
                    }
                }
            } else {
                right_lane_flag = false;
            }
        }

        if (split_exits_flag) {
            break;
        }

        cur_link_id = back_spit_link;
        cur_lane_id = back_spit_lane;
        left_cur_lane_id = left_lane_flag ? left_back_spit_lane : 0;
        right_cur_lane_id = right_lane_flag ? right_back_spit_lane : 0;

        message::map_map::s_LinkInfo_t back_link_infos;
        if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(
                         map_msg, map_raw_data_map.link_id_index_lane_info_map, back_spit_link, back_link_infos)) {
            // static data none
            break;
        }

        lanes_length += back_link_infos.EndOffset.EndOffset - back_link_infos.PathOffset.PathOffset;
    }

    // std::cout << __FILE__ << __LINE__ << "left_lane_flag: " << left_lane_flag << std::endl;
    // std::cout << __FILE__ << __LINE__ << "right_lane_flag: " << right_lane_flag << std::endl;
    // std::cout << __FILE__ << __LINE__ << "split_exits_flag: " << split_exits_flag << std::endl;
    if (split_exits_flag) {
        if (left_lane_flag) {
            continue_lane_id = map_position_tmp.LaneId + 1;
        } else if (right_lane_flag) {
            continue_lane_id = map_position_tmp.LaneId - 1;
        } else {
            continue_lane_id = map_position_tmp.LaneId;
        }
    }

    return true;
}

bool EfmInit::GetSplitSideLaneV3(const MapMapMsg& map_msg, const PreCycleData& pre_cycle_data,
                                 MapPositionTmp& map_position_tmp, const MapRawDataMap& map_raw_data_map,
                                 uint8_t& continue_lane_id, uint32_t& lanes_length) {
    continue_lane_id = map_position_tmp.LaneId;
    if (0 == map_position_tmp.LaneId) {
        return false;
    }

    uint32_t cur_link_id = map_position_tmp.LinkId;
    uint32_t cur_lane_id = map_position_tmp.LaneId;
    uint32_t left_cur_lane_id = map_position_tmp.LaneId + 1;
    uint32_t right_cur_lane_id = map_position_tmp.LaneId - 1;
    bool left_lane_flag = true;
    bool right_lane_flag = true;
    bool split_exits_flag = false;
    uint8_t max_roop_times = 0;
    int count = 0;
    while (lanes_length < 70 * 100 && max_roop_times < 20 && count < 100) {
        count++;
        // std::cout << __FILE__ << __LINE__ << "length: " << lanes_length << std::endl;
        if (false == left_lane_flag && false == right_lane_flag) {
            break;
        }
        // std::cout << __FILE__ << __LINE__ << "cur_link_id: " << cur_link_id << std::endl;
        // std::cout << __FILE__ << __LINE__ << "cur_lane_id: " << int(cur_lane_id) << std::endl;
        // std::cout << __FILE__ << __LINE__ << "left_cur_lane_id: " << int(left_cur_lane_id) << std::endl;
        // std::cout << __FILE__ << __LINE__ << "right_cur_lane_id: " << int(right_cur_lane_id) << std::endl;

        message::map_map::s_LinkInfo_t cur_link_infos;
        if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(
                         map_msg, map_raw_data_map.link_id_index_lane_info_map, cur_link_id, cur_link_infos)) {
            // static data none
            break;
        }
        max_roop_times++;

        uint32_t back_spit_link = 0;
        uint32_t back_spit_lane = 0;
        efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                           cur_lane_id, back_spit_link, back_spit_lane);
        // std::cout << __FILE__ << __LINE__ << "back_spit_link: " << back_spit_link << std::endl;
        // std::cout << __FILE__ << __LINE__ << "back_spit_lane: " << int(back_spit_lane) << std::endl;

        uint32_t left_back_spit_link = 0;
        uint32_t left_back_spit_lane = 0;
        efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                            left_cur_lane_id, left_back_spit_link,
                                                            left_back_spit_lane);
        // std::cout << __FILE__ << __LINE__ << "left_back_spit_link: " << left_back_spit_link << std::endl;
        // std::cout << __FILE__ << __LINE__ << "left_back_spit_lane: " << int(left_back_spit_lane) << std::endl;
        if (left_back_spit_link == back_spit_link && left_back_spit_link > 0) {
            if (back_spit_lane == left_back_spit_lane) {
                uint8_t left_virtual_line_case = 0;
                efm::MapCommonTool::GetInstance()->IsVirtuallyLine(map_msg, map_position_tmp, cur_link_infos,
                                            left_cur_lane_id, cur_lane_id, true, left_virtual_line_case);
                // std::cout << __FILE__ << __LINE__ << "left_virtual_line_case: " << int(left_virtual_line_case)
                // << std::endl;
                if (left_virtual_line_case == 1) {
                    split_exits_flag = true;
                    right_lane_flag = false;
                }
                uint8_t cur_lane_id_tran;
                // std::cout << __FILE__ << __LINE__ << "left_cur_lane_id: " << int(left_cur_lane_id) << std::endl;
                GetLaneTran(pre_cycle_data, cur_link_infos, cur_lane_id, cur_lane_id_tran);
                // uint8_t cur_left_lane_id_tran;
                // GetLaneTran(pre_cycle_data, cur_link_infos, left_cur_lane_id, cur_left_lane_id_tran);
                if (1 == cur_lane_id_tran || 99 == cur_lane_id_tran) {
                    left_lane_flag = false;
                }
            }
        }
        // else {
        //     left_lane_flag = false;
        // }

        uint32_t right_back_spit_link = 0;
        uint32_t right_back_spit_lane = 0;

        efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                            right_cur_lane_id, right_back_spit_link,
                                                            right_back_spit_lane);
        // std::cout << __FILE__ << __LINE__ << "right_back_spit_link: " << right_back_spit_link << std::endl;
        // std::cout << __FILE__ << __LINE__ << "right_back_spit_lane: " << int(right_back_spit_lane) << std::endl;
        if (right_back_spit_link == back_spit_link && right_back_spit_link > 0) {
            if (back_spit_lane == right_back_spit_lane) {
                uint8_t right_virtual_line_case = 0;
                efm::MapCommonTool::GetInstance()->IsVirtuallyLine(map_msg, map_position_tmp, cur_link_infos,
                                                    cur_lane_id, right_cur_lane_id, false, right_virtual_line_case);
                // std::cout << __FILE__ << __LINE__ << "right_virtual_line_case: " << int(right_virtual_line_case)
                // << std::endl;
                if (right_virtual_line_case == 1) {
                    left_lane_flag = false;
                    split_exits_flag = true;
                }
                uint8_t cur_lane_id_tran;
                // std::cout << __FILE__ << __LINE__ << "right_cur_lane_id: " << int(right_cur_lane_id) <<
                // std::endl;
                GetLaneTran(pre_cycle_data, cur_link_infos, cur_lane_id, cur_lane_id_tran);
                // uint8_t cur_right_lane_id_tran;
                // GetLaneTran(pre_cycle_data, cur_link_infos, right_cur_lane_id, cur_right_lane_id_tran);
                if (1 == cur_lane_id_tran || 99 == cur_lane_id_tran) {
                    right_lane_flag = false;
                }
            }
        }
        // else {
        //     right_lane_flag = false;
        // }

        // std::cout << __FILE__ << __LINE__ << "split_exits_flag: " << split_exits_flag << std::endl;
        if (split_exits_flag) {
            break;
        }

        cur_link_id = back_spit_link;
        cur_lane_id = back_spit_lane;
        left_cur_lane_id = left_lane_flag ? left_back_spit_lane : 0;
        right_cur_lane_id = right_lane_flag ? right_back_spit_lane : 0;

        message::map_map::s_LinkInfo_t back_link_infos;
        if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(
                         map_msg, map_raw_data_map.link_id_index_lane_info_map, back_spit_link, back_link_infos)) {
            // static data none
            // std::cout << __FILE__ << __LINE__ << "static data none" << std::endl;
            break;
        }

        lanes_length += back_link_infos.EndOffset.EndOffset - back_link_infos.PathOffset.PathOffset;
    }

    // std::cout << __FILE__ << __LINE__ << "left_lane_flag: " << left_lane_flag << std::endl;
    // std::cout << __FILE__ << __LINE__ << "right_lane_flag: " << right_lane_flag << std::endl;
    // std::cout << __FILE__ << __LINE__ << "split_exits_flag: " << split_exits_flag << std::endl;
    // std::cout << __FILE__ << __LINE__ << "length: " << lanes_length << std::endl;
    if (split_exits_flag) {
        if (left_lane_flag) {
            continue_lane_id = map_position_tmp.LaneId + 1;
        } else if (right_lane_flag) {
            continue_lane_id = map_position_tmp.LaneId - 1;
        } else {
            continue_lane_id = map_position_tmp.LaneId;
        }
    }

    return true;
}

bool EfmInit::GetLaneTran(const PreCycleData& pre_cycle_data, message::map_map::s_LinkInfo_t& link_infos,
                          uint8_t lane_id, uint8_t& tran) {
    tran = 0;
    uint64_t key_temp =
        ((static_cast<uint64_t>(lane_id)) << 32) | (static_cast<uint64_t>(link_infos.InstanceId.InstanceId));
    // std::map<uint64_t, uint8_t> lane_link_tarn_map = candidate_lanes_model_->get_split_lind_id_lane_id_map();
    std::map<uint64_t, uint8_t> lane_link_tarn_map = pre_cycle_data.last_split_line_id_lane_id;
    if (!lane_link_tarn_map.empty()) {
        if (lane_link_tarn_map.find(key_temp) != lane_link_tarn_map.end()) {
            tran = lane_link_tarn_map[key_temp];
            return true;
        }
    }

    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            tran = laneinfo.Transit.data;
            break;
        }
    }

    return true;
}

bool EfmInit::GetSplitSideLane(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                               const MapRawDataMap& map_raw_data_map, uint32_t link_id, uint8_t lane_id,
                               std::vector<uint8_t>& split_side_lanes, uint32_t& lanes_length,
                               uint32_t& change_position_range) {
    uint32_t cur_link_id = link_id;
    uint32_t cur_lane_id = lane_id;
    split_side_lanes.clear();
    int count = 0;                            
    while (lanes_length < CHANGE_POSITION_RANGE && count < 50) {
        count++;
        uint32_t back_spit_link = 0;
        uint32_t back_spit_lane = 0;
        efm::MapCommonTool::GetInstance()->GetBackLinkLane(map_msg, map_raw_data_map, map_position_tmp, cur_link_id,
                                                           cur_lane_id, back_spit_link, back_spit_lane);
        message::map_map::s_LinkInfo_t back_link_infos;
        if (false == efm::MapCommonTool::GetInstance()->GetLinkInfos(
                         map_msg, map_raw_data_map.link_id_index_lane_info_map, back_spit_link, back_link_infos)) {
            // static data none
            break;
        }
        cur_link_id = back_spit_link;
        cur_lane_id = back_spit_lane;
        lanes_length += back_link_infos.EndOffset.EndOffset - back_link_infos.PathOffset.PathOffset;
    }
    if (cur_link_id == link_id && cur_lane_id == lane_id) {
        return false;
    }

    std::vector<uint32_t> link_lane_id;
    link_lane_id.push_back(cur_link_id);
    link_lane_id.push_back(cur_lane_id);
    std::vector<std::vector<uint32_t>> link_lane_ids;
    link_lane_ids.push_back(link_lane_id);
    for (int i = 0; i < link_lane_ids.size(); i++) {
        auto temp_link_id = link_lane_ids[i][0];
        auto temp_lane_id = link_lane_ids[i][1];
        if (map_raw_data_map.link_id_index_lane_connect_map.find(temp_link_id) !=
            map_raw_data_map.link_id_index_lane_connect_map.end()) {
            for (auto idx : map_raw_data_map.link_id_index_lane_connect_map.at(temp_link_id)) {
                if (temp_lane_id == map_msg.LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum &&
                    temp_link_id == map_msg.LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                    if (link_id == map_msg.LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink) {
                        if (lane_id + 1 == map_msg.LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum ||
                            lane_id == map_msg.LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum + 1) {
                            split_side_lanes.push_back(
                                map_msg.LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum);
                        } else {
                            continue;
                        }
                    } else {
                        if (map_msg.LaneConnectivitys.PairConnectivity[idx].NewPath.NewPath ==
                            map_position_tmp.PathId) {
                            std::vector<uint32_t> temp_link_lane_id;
                            temp_link_lane_id.push_back(
                                map_msg.LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink);
                            temp_link_lane_id.push_back(
                                map_msg.LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum);
                            link_lane_ids.push_back(temp_link_lane_id);
                        }
                    }
                }
            }
        }
    }

    if (0 == split_side_lanes.size()) {
        return false;
    }

    return true;
}

void EfmInit::InitErrorAndHead(const MapPositionMsg& position_msg, const MapRouteListMsg& map_route_list,
                               MainFuncStaticVar& main_func_static_var, uint64_t& errorcode, EfmInfoMsg& efm_info,
                               MapLaneMsg& map_lane, MapLppInfoMsg& map_lpp_info, RMFMapinfoDataclose& rmf_dc,
                               uint64_t efm_counter) {
    ErrorCodeLog(main_func_static_var, errorcode);
    SetDataHead(efm_info, map_lane, map_lpp_info, rmf_dc, efm_counter, errorcode, position_msg.header.timestamp);
    InitVariableZTEXT(false, position_msg.Position, map_route_list, map_lpp_info);
    main_func_static_var.errorcode = errorcode;
    return;
}

bool EfmInit::ErrorCodeLog(MainFuncStaticVar& main_func_static_var, uint64_t& errorcode) {
    if (main_func_static_var.loge_counter >= 30 || errorcode != main_func_static_var.errorcode) {
        LOGE("errorcode: {}", errorcode);
        main_func_static_var.loge_counter = 0;
    } else {
        main_func_static_var.loge_counter++;
    }
    return true;
}

bool EfmInit::SetDataHead(EfmInfoMsg& efm_info, MapLaneMsg& map_lane, MapLppInfoMsg& map_lpp_info,
                          RMFMapinfoDataclose& rmf_dc, uint64_t efm_counter, uint64_t errorcode, uint64_t timestamp) {
    ZTEXT("EFM_INFO", "errorcode_efm: ", -30, 24, "errorcode_efm: {}", errorcode);
    uint64_t ignore_errorcode = EFM_CODE_EHP_NO_STATIC_MAP | EFM_CODE_EHP_NO_POSITION_ERROR |
                                EFM_CODE_EHP_NO_SWITCH_ERROR | EFM_CODE_EHP_NO_ROUTE_ERROR |
                                EFM_CODE_EHP_NO_GLOBAL_ERROR | EFM_CODE_EHP_NO_DYNAMIC_ERROR |
                                EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR;
    if (0 == ((errorcode | ignore_errorcode) ^ ignore_errorcode)) {
        // 某几位不为0的条件成立
        map_lpp_info.bIsActive = true;
        errorcode = 0 | (errorcode & EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR);
    } else {
        map_lpp_info.bIsActive = false;
        errorcode = (errorcode | EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR) ^ EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR;
    }
    ZTEXT("EFM_INFO", "errorcode_tolpp: ", -30, 50, "errorcode_tolpp: {}", errorcode);

    // efm_info
    efm_info.header.cntr = efm_counter;
    efm_info.header.timestamp = timestamp;
    efm_info.header.err_code = errorcode;

    // map_lane
    map_lane.header.cntr = efm_counter;
    map_lane.header.timestamp = timestamp;
    map_lane.header.err_code = errorcode;

    // map_lpp_info
    map_lpp_info.header.cntr = efm_counter;
    map_lpp_info.header.timestamp = timestamp;
    map_lpp_info.header.err_code = errorcode;

    // rmf_dc
    rmf_dc.header.cntr = efm_counter;
    rmf_dc.header.timestamp = timestamp;
    rmf_dc.header.err_code = errorcode;

    map_lane.efm_info = map_lpp_info;

    return true;
}

bool EfmInit::MakeDataLossErrorCode(uint64_t& errorcode, MainFuncStaticVar& main_func_static_var,
                                    const MapMapMsg& map_msg, const MapPositionMsg& position_msg,
                                    const MapSwitchInfoMsg& switch_info_msg, const MapRouteListMsg& map_route_list,
                                    const MapGlobalDataMsg& global_data_msg, const MapDynamicMsg& map_dynamic_msg) {
    uint64_t map_position_counter_now = 0;
    uint64_t map_map_counter_now = 0;
    uint64_t map_switch_counter_now = 0;
    uint64_t map_route_counter_now = 0;
    uint64_t map_global_counter_now = 0;
    uint64_t map_dynamic_counter_now = 0;
    map_position_counter_now = position_msg.header.cntr;
    map_map_counter_now = map_msg.header.cntr;
    map_switch_counter_now = switch_info_msg.header.cntr;
    map_route_counter_now = map_route_list.header.cntr;
    map_global_counter_now = global_data_msg.header.cntr;
    map_dynamic_counter_now = map_dynamic_msg.header.cntr;

    // std::cout << "map_position_counter_now: " << map_position_counter_now << std::endl;
    // std::cout << "main_func_static_var.map_position_counter: " << main_func_static_var.map_position_counter <<
    // std::endl; std::cout << "map_map_counter_now: " << map_map_counter_now << std::endl; std::cout <<
    // "main_func_static_var.map_map_counter: " << main_func_static_var.map_map_counter << std::endl; std::cout <<
    // "map_switch_counter_now: " << map_switch_counter_now << std::endl; std::cout << "map_switch_counter_last: " <<
    // main_func_static_var.map_switch_counter << std::endl; std::cout << "map_route_counter_now: " <<
    // map_route_counter_now << std::endl; std::cout << "main_func_static_var.map_route_counter: " <<
    // main_func_static_var.map_route_counter << std::endl; std::cout << "map_global_counter_now: " <<
    // map_global_counter_now << std::endl; std::cout << "main_func_static_var.map_global_counter: " <<
    // main_func_static_var.map_global_counter << std::endl; std::cout << "map_dynamic_counter_now: " <<
    // map_dynamic_counter_now << std::endl; std::cout << "main_func_static_var.map_dynamic_counter: " <<
    // main_func_static_var.map_dynamic_counter << std::endl;

    // if (map_position_counter_now != map_position_counter_last + 1) {
    if (map_position_counter_now != main_func_static_var.map_position_counter + 1) {
        errorcode = errorcode | EFM_CODE_EHP_NO_POSITION_ERROR;
    }
    // if (map_position_counter_now == map_position_counter_last) {
    if (map_position_counter_now == main_func_static_var.map_position_counter) {
        main_func_static_var.counter_position_lost++;
        if (main_func_static_var.counter_position_lost >= 6) {
            errorcode = errorcode | EFM_CODE_EHP_POSITON_COUNTER_REPEAT_ERROR;
        }
    } else {
        main_func_static_var.counter_position_lost = 0;
    }
    if (!(map_map_counter_now == main_func_static_var.map_map_counter ||
          map_map_counter_now == main_func_static_var.map_map_counter + 1)) {
        errorcode = errorcode | EFM_CODE_EHP_NO_STATIC_MAP;
    }
    if (!(map_switch_counter_now == main_func_static_var.map_switch_counter ||
          map_switch_counter_now == main_func_static_var.map_switch_counter + 1)) {
        errorcode = errorcode | EFM_CODE_EHP_NO_SWITCH_ERROR;
    }
    if (!(map_route_counter_now == main_func_static_var.map_route_counter ||
          map_route_counter_now == main_func_static_var.map_route_counter + 1)) {
        errorcode = errorcode | EFM_CODE_EHP_NO_ROUTE_ERROR;
    }
    if (!(map_global_counter_now == main_func_static_var.map_global_counter ||
          map_global_counter_now == main_func_static_var.map_global_counter + 1)) {
        errorcode = errorcode | EFM_CODE_EHP_NO_GLOBAL_ERROR;
    }
    if (!(map_dynamic_counter_now == main_func_static_var.map_dynamic_counter ||
          map_dynamic_counter_now == main_func_static_var.map_dynamic_counter + 1)) {
        errorcode = errorcode | EFM_CODE_EHP_NO_DYNAMIC_ERROR;
    }
    main_func_static_var.map_position_counter = map_position_counter_now;
    main_func_static_var.map_map_counter = map_map_counter_now;
    main_func_static_var.map_switch_counter = map_switch_counter_now;
    main_func_static_var.map_route_counter = map_route_counter_now;
    main_func_static_var.map_global_counter = map_global_counter_now;
    main_func_static_var.map_dynamic_counter = map_dynamic_counter_now;
    return true;
}

bool EfmInit::MakeErrorCode(uint64_t& errorcode, MainFuncStaticVar& main_func_static_var, const MapMapMsg& map_msg,
                            const MapPositionMsg& position_msg, const MapSwitchInfoMsg& switch_info_msg,
                            const MapRouteListMsg& map_route_list, const MapGlobalDataMsg& global_data_msg,
                            const MapDynamicMsg& map_dynamic_msg, EfmInfoMsg& efm_info) {
    efm_info.LocErrorSts.data = 0;
    bool is_navigation_ok = false;
    bool is_matching_ok = false;
    bool is_failsafe_loc_ok = false;
    // ZTEXT("EFM_INFO", "SwitchLaneDistance: ", -30, 33, "SwitchLaneDistance: {}",
    //       switch_info_msg.NOAInfo.SwitchLaneDistance.SwitchLaneDistance);
    // ZTEXT("EFM_INFO", "SwitchLaneReason: ", -30, 36, "SwitchLaneReason: {}",
    //       switch_info_msg.NOAInfo.SwitchLaneReason.data);
    // ZTEXT("EFM_INFO", "SwitchLaneEndDistance: ", -30, 30, "SwitchLaneEndDistance: {}",
    //       switch_info_msg.NOAInfo.SwitchLaneEndDistance.SwitchLaneDistance);
    // ZTEXT("EFM_INFO", "SwitchLaneDirection: ", -30, 27, "SwitchLaneDirection: {}",
    //       switch_info_msg.NOAInfo.SwitchLaneDirection.data);
    // 2
    ZTEXT("EFM_INFO", "NavigationStatus: ", -30, 47, "NavigationStatus: {}",
          switch_info_msg.NOAInfo.NavigationStatus.data);
    if (switch_info_msg.NOAInfo.NavigationStatus.data == 2) {
        is_navigation_ok = true;
    } else {
        is_navigation_ok = false;
    }
    // 1 2 3
    ZTEXT("EFM_INFO", "MatchingStatus: ", -30, 44, "MatchingStatus: {}", switch_info_msg.NOAInfo.MatchingStatus.data);
    if (switch_info_msg.NOAInfo.MatchingStatus.data == 1 || switch_info_msg.NOAInfo.MatchingStatus.data == 2 ||
        switch_info_msg.NOAInfo.MatchingStatus.data == 3) {
        is_matching_ok = true;
    } else {
        is_matching_ok = false;
    }
    ZTEXT("EFM_INFO", "FailSafeLocStatus: ", -30, 41, "FailSafeLocStatus: {}", position_msg.Position.FailSafeLocStatus.data);
    if (position_msg.Position.FailSafeLocStatus.data == 1 || position_msg.Position.FailSafeLocStatus.data == 0) {
        is_failsafe_loc_ok = true;
    } else {
        is_failsafe_loc_ok = false;
        // efm_info.LocErrorSts.data = 1;
    }

    if (!is_navigation_ok) {
        errorcode = errorcode | EFM_CODE_EHP_NO_NAVIGATION_STATUS;
    }
    if (!is_matching_ok) {
        errorcode = errorcode | EFM_CODE_EHP_NO_MATCHING_STATUS;
    }
    if ((is_failsafe_loc_ok == false) || position_msg.Position.PathOffset == 0) {
        errorcode = errorcode | EFM_CODE_EHP_NO_LOC_STATUS;
    }
    // ZTEXT("EFM_INFO", "errorcode_efm: ", -30, 24, "errorcode_efm: {}", errorcode);

    if ((errorcode & (EFM_CODE_EHP_NO_NAVIGATION_STATUS | EFM_CODE_EHP_NO_MATCHING_STATUS | EFM_CODE_EHP_NO_LOC_STATUS |
                      EFM_CODE_EHP_POSITON_COUNTER_REPEAT_ERROR)) != 0) {
        efm::EfmInit::GetInstance()->ErrorCodeLog(main_func_static_var, errorcode);
        main_func_static_var.errorcode = errorcode;
    }

    return true;
}

}  // namespace efm
