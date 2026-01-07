/**
 * @file ExtractRefLine.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-12-05
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "MapCommonTool.h"

namespace efm {
MapCommonTool* MapCommonTool::GetInstance() {
    static MapCommonTool instance;
    return &instance;
}

bool MapCommonTool::MakeIndex(const MapStaticInfo& map_static_info, const MapPosition& map_position_info,
                              const MapRouteList& map_route_list_info, MapRawDataMap& map_raw_data,
                              uint64_t& errorcode) {
    map_raw_data.link_id_index_lane_info_map.clear();
    map_raw_data.link_id_index_lane_connect_map.clear();
    map_raw_data.to_link_id_index_lane_connect_map.clear();
    map_raw_data.route_list_link_ids.clear();
    map_raw_data.linear_obj_id_map.clear();
    map_raw_data.curve_index_map.clear();
    map_raw_data.lane_widths_map.clear();
    // store all links
    for (int i = 0; i < map_static_info.LinkInfos.LinkInfos.size(); i++) {
        // if (map_static_info.LinkInfos.LinkInfos[i].InstanceId.InstanceId == 0){
        //     std::cout << __FILE__ << __LINE__ << "Link with InstanceId 0 found at index: " << i << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "cur_link_id: " << map_static_info.LinkInfos.LinkInfos[i].InstanceId.InstanceId << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "PathId: " << map_static_info.LinkInfos.LinkInfos[i].PathId.PathId << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "PathOffset: " << map_static_info.LinkInfos.LinkInfos[i].PathOffset.PathOffset << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "EndOffset: " << map_static_info.LinkInfos.LinkInfos[i].EndOffset.EndOffset << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "TotalNumLane: " << map_static_info.LinkInfos.LinkInfos[i].TotalNumLane.TotalNumLane << std::endl;
        // }
        if (map_static_info.LinkInfos.LinkInfos[i].InstanceId.InstanceId > 0){
            map_raw_data.link_id_index_lane_info_map.emplace(map_static_info.LinkInfos.LinkInfos[i].InstanceId.InstanceId, i);
        }
    }

    // store connectivity on cur_path
    // uint32_t cur_path_id = map_position_->PathId;
    // todo: only store link on map_route_list????????????

    for (int i = 0; i < map_static_info.LaneConnectivitys.PairConnectivity.size(); i++) {
        uint32_t cur_search_link_id = map_static_info.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId;

        if (map_raw_data.link_id_index_lane_connect_map.find(cur_search_link_id) ==
            map_raw_data.link_id_index_lane_connect_map.end()) {
            // new link id, add index vector to store
            std::vector<int> index_vec;
            index_vec.emplace_back(i);
            map_raw_data.link_id_index_lane_connect_map.emplace(
                map_static_info.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId, index_vec);
        } else {
            map_raw_data.link_id_index_lane_connect_map[cur_search_link_id].emplace_back(i);
        }
        uint32_t cur_to_link_id = map_static_info.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLink;
        if (map_raw_data.to_link_id_index_lane_connect_map.find(cur_to_link_id) ==
            map_raw_data.to_link_id_index_lane_connect_map.end()) {
            // new link id, add index vector to store
            std::vector<int> index_vec;
            index_vec.emplace_back(i);
            map_raw_data.to_link_id_index_lane_connect_map.emplace(
                map_static_info.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLink, index_vec);
        } else {
            map_raw_data.to_link_id_index_lane_connect_map[cur_to_link_id].emplace_back(i);
        }
    }
    if (map_raw_data.link_id_index_lane_info_map.empty()) {
        errorcode = errorcode | EFM_CODE_EHP_NO_LINK_INFOS;
        return false;
    } else if (map_raw_data.link_id_index_lane_connect_map.empty() ||
               map_raw_data.to_link_id_index_lane_connect_map.empty()) {
        errorcode = errorcode | EFM_CODE_EHP_NO_CONNECT_INFOS;
        return false;
    }

    {  // route list path id check
        uint32_t postionPathId = map_position_info.Position.PathId;
        for (int i = 0; i < map_route_list_info.LinkIds.LinkIds.size(); i++) {
            uint32_t link_id = map_route_list_info.LinkIds.LinkIds[i].LinkId;
            message::map_map::s_LinkInfo_t link_infos{};
            if (false == GetLinkInfos(map_static_info, map_raw_data.link_id_index_lane_info_map, link_id, link_infos)) {
                // static data none
                break;
            }
            if (postionPathId != link_infos.PathId.PathId) {
                errorcode = errorcode | EFM_CODE_EHP_PATH_ID_NOT_MATCH;
                return false;
            }
            map_raw_data.route_list_link_ids.emplace_back(link_id);
        }
    }

    {//linear object
        // uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
        uint32_t linear_obj_id = 0;
        for(int i = 0; i < map_static_info.LinearObjects.LinearObjects.size();i++){
            linear_obj_id = map_static_info.LinearObjects.LinearObjects[i].IDLinearObject.IDLinearObject;
            map_raw_data.linear_obj_id_map.emplace(linear_obj_id,i);
        }
    } 
    {//LinkCurvatures
        // uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
        uint64_t key_temp = 0;
        uint32_t link_id = 0;
        uint8_t lane_id = 0;
        for(int i = 0; i < map_static_info.LinkCurvatures.LinkCurvatures.size();i++){
            link_id = map_static_info.LinkCurvatures.LinkCurvatures[i].InstanceId.InstanceId;
            lane_id = map_static_info.LinkCurvatures.LinkCurvatures[i].LaneNum.LaneNum;
            key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
            map_raw_data.curve_index_map.emplace(key_temp,i);
        }
    } 

    {//lane widths
        uint64_t key_temp = 0;
        uint32_t link_id = 0;
        uint8_t lane_id = 0;
        for(int i = 0; i < map_static_info.LaneWidths.LaneWidths.size();i++){
            link_id = map_static_info.LaneWidths.LaneWidths[i].InstanceId.InstanceId;
            lane_id = map_static_info.LaneWidths.LaneWidths[i].LaneNum.LaneNum;
            key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
            map_raw_data.lane_widths_map.emplace(key_temp,i);
        }
    }
    return true;
}

bool MapCommonTool::GetLinkInfos(const MapStaticInfo& map_static_info, const IndexMap& link_id_index_lane_info_map,
                                 uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map.find(link_id) != link_id_index_lane_info_map.end()) {
        int link_index = link_id_index_lane_info_map.at(link_id);
        link_infos = map_static_info.LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

bool MapCommonTool::GetLaneInfo(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id,
                                message::map_map::s_LaneInfo_t& lane) {
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            lane = laneinfo;
            return true;
        }
    }
    return false;
}

bool MapCommonTool::GetLaneCurvatures(
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info, uint32_t link_id,
    uint8_t lane_id, message::map_map::s_LinkCurvature_t& link_Curvatures) {
    for (auto& link_curv_info : map_static_info->LinkCurvatures.LinkCurvatures) {
        if (link_curv_info.InstanceId.InstanceId == link_id && link_curv_info.LaneNum.LaneNum == lane_id) {
            link_Curvatures = link_curv_info;
            return true;
        }
    }
    return false;
}

bool MapCommonTool::GetLaneWidth(
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info, uint32_t link_id,
    uint8_t lane_id, message::map_map::s_LaneWidth_t& lane_width) {
    for (auto& lane_width_tmp : map_static_info->LaneWidths.LaneWidths) {
        if (lane_width_tmp.InstanceId.InstanceId == link_id && lane_width_tmp.LaneNum.LaneNum == lane_id) {
            lane_width = lane_width_tmp;
            return true;
        }
    }
    return false;
}

bool MapCommonTool::GetLaneFromConnectivitys(
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info,
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>> link_id_index_lane_connect_map,
    uint32_t link_id, uint8_t lane_id, std::vector<message::map_map::s_PairConnectivity_t>& lane_Connectivitys) {
    lane_Connectivitys.clear();
    std::vector<int> cur_link_staticmap_indices{};
    if (link_id_index_lane_connect_map->find(link_id) != link_id_index_lane_connect_map->end()) {
        cur_link_staticmap_indices = link_id_index_lane_connect_map->at(link_id);
        for (auto index : cur_link_staticmap_indices) {
            if (map_static_info->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum == lane_id) {
                lane_Connectivitys.push_back(map_static_info->LaneConnectivitys.PairConnectivity[index]);
            }
        }
    } else {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << " can't find connect : " << link_id << std::endl;
        return false;
    }
    
    if(lane_Connectivitys.size()>0){
        return true;
    }else{
        return false;
    }
    
}

bool MapCommonTool::GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                      uint32_t& center_line_index) {
    center_line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            center_line_index = lane.Centeline.Centeline;
            break;
        }
    }
    if (center_line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool MapCommonTool::GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                     uint32_t& line_index) {
    line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            line_index = lane.RBound.RBound;
            break;
        }
    }
    if (line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool MapCommonTool::GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                    uint32_t& line_index) {
    line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            line_index = lane.LBound.LBound;
            break;
        }
    }
    if (line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool MapCommonTool::GetLineGeometry(const MapStaticInfo& map_static_info, uint32_t line_id,
                                    std::vector<message::map_map::s_GeometryPoint_t>& geometry_points) {
    geometry_points.clear();
    if (0 == line_id) {
        return false;
    }

    for (auto& lineinfo : map_static_info.LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            geometry_points = lineinfo.GeometryPoints.GeometryPoints;
            return true;
        }
    }

    if (geometry_points.empty()) {
        return false;
    }

    return true;
}

void MapCommonTool::IsVirtuallyLine(const MapStaticInfo& map_msg, MapPositionTmp& map_position_tmp,
                                    const message::map_map::s_LinkInfo_t& link_infos, const uint8_t& left_lane_id,
                                    const uint8_t& right_lane_id, bool is_left, uint8_t& virtual_line_case) {
    // virtual_line_case: 0: not virtual line, 1: valid virtual line, 2: non-valid virtual line
    virtual_line_case = 0;
    uint8_t left_line_type = 0;
    uint8_t right_line_type = 0;
    auto map_position_ = map_position_tmp;
    GetLineType(map_msg, link_infos, left_lane_id, right_line_type, false);
    GetLineType(map_msg, link_infos, right_lane_id, left_line_type, true);

    if (8 == left_line_type && 8 == right_line_type) {
        uint32_t ego_bound_line_idx = 0, side_bound_line_idx = 0;
        std::vector<message::map_map::s_GeometryPoint_t> ego_bound_geometry_points_wgs{},side_bound_geometry_points_wgs{};
        EFMRefLinePoints ego_bound_geometry_points_ego{}, side_bound_geometry_points_ego{};
        std::vector<double> ego_bounds{}, side_bounds{};
        double points_sl_sum = 0;
        bool is_parallel_line = false;
        if (is_left) {
            GetLaneLeftLine(link_infos, right_lane_id, ego_bound_line_idx);
            GetLaneRightLine(link_infos, left_lane_id, side_bound_line_idx);
        } else {
            GetLaneRightLine(link_infos, left_lane_id, ego_bound_line_idx);
            GetLaneLeftLine(link_infos, right_lane_id, side_bound_line_idx);
        }

        GetLineGeometry(map_msg, ego_bound_line_idx, ego_bound_geometry_points_wgs);
        GetLineGeometry(map_msg, side_bound_line_idx, side_bound_geometry_points_wgs);

        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
            ego_bound_geometry_points_wgs, ego_bound_geometry_points_ego, map_position_, map_position_.Heading.Heading);
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
            side_bound_geometry_points_wgs,side_bound_geometry_points_ego, map_position_,map_position_.Heading.Heading);

        SLPoint sl(0.0, 0.0);

        for (auto point : ego_bound_geometry_points_ego) {
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(side_bound_geometry_points_ego, point, sl);
            side_bounds.push_back(sl.l);
        }
        // std::cout << "side_bounds.size(): " << side_bounds.size() << " {";
        // for(const auto& l:side_bounds){
        //     std::cout << l << ",";
        // }
        // std::cout << "}" << std::endl;
        // points_num = (ego_bounds.size() < side_bounds.size()) ? ego_bounds.size() : side_bounds.size();
        int side_bounds_size = side_bounds.size();
        for (int i = 0; i < side_bounds_size; i++) {
            points_sl_sum +=  side_bounds[i];
        }

        if (side_bounds_size > 0 && points_sl_sum > 0.1 * side_bounds_size) {
            virtual_line_case = 1;
        } else if (side_bounds_size > 0 && points_sl_sum  < 0.1 * side_bounds_size && ego_bound_line_idx == side_bound_line_idx) {
            virtual_line_case = 1;
        } else if (side_bounds_size > 0 && points_sl_sum   < 0.1 * side_bounds_size && ego_bound_line_idx != side_bound_line_idx) {
            virtual_line_case = 2;
        } else {
            virtual_line_case = 0;
        }
        // std::cout << "is_parallel_line: " << int(is_parallel_line) << std::endl;
    }

    return;
}

bool MapCommonTool::GetLineType(const MapStaticInfo& map_static_info, const message::map_map::s_LinkInfo_t& link_infos,
                                uint8_t lane_id, uint8_t& line_type, bool is_left) {
    uint32_t line_id = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            if (is_left) {
                line_id = laneinfo.LBound.LBound;
            } else {
                line_id = laneinfo.RBound.RBound;
            }
            break;
        }
    }

    line_type = 0;
    for (auto& lineinfo : map_static_info.LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            line_type = lineinfo.LinearObjectMarking.data;
            break;
        }
    }

    return true;
}

bool MapCommonTool::GetBackLinkLane(const MapStaticInfo& map_msg, const MapRawDataMap& map_raw_data_map,
                                    MapPositionTmp& map_position_tmp, uint32_t link_id, uint8_t lane_id,
                                    uint32_t& back_link_id, uint32_t& back_lane_id) {
    back_link_id = 0;
    back_lane_id = 0;
    int link_idx = -1;
    // for (int i = 0; i < map_route_list_->LinkIds.LinkIds.size(); i++) {
    //     if (map_route_list_->LinkIds.LinkIds[i].LinkId == link_id) {
    //         link_idx = i;
    //         break;
    //     }
    // }
    // if(link_idx > 0){
    // back_link_id = map_route_list_->LinkIds.LinkIds[link_idx - 1].LinkId;
    if (map_raw_data_map.to_link_id_index_lane_connect_map.find(link_id) !=
        map_raw_data_map.to_link_id_index_lane_connect_map.end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : map_raw_data_map.to_link_id_index_lane_connect_map.at(link_id)) {
            uint32_t temp_back_link_id = map_msg.LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
            uint8_t temp_lane_id = map_msg.LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            if (map_msg.LaneConnectivitys.PairConnectivity[idx].InitPath.InitPath == map_position_tmp.PathId &&
                lane_id == temp_lane_id) {
                back_lane_id = map_msg.LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
                back_link_id = map_msg.LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                break;
            }
        }
    }
    // }

    if (0 == back_link_id || 0 == back_lane_id) {
        // link_id is last link
        return false;
    }

    return true;
}

bool MapCommonTool::GetLaneToConnectivitys(
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info,
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>> to_link_id_index_lane_connect_map,
    uint32_t link_id, uint8_t lane_id, std::vector<message::map_map::s_PairConnectivity_t>& lane_Connectivitys) {
    lane_Connectivitys.clear();
    std::vector<int> cur_link_staticmap_indices{};
    if (to_link_id_index_lane_connect_map->find(link_id) != to_link_id_index_lane_connect_map->end()) {
        cur_link_staticmap_indices = to_link_id_index_lane_connect_map->at(link_id);
        for (auto index : cur_link_staticmap_indices) {
            if (map_static_info->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum == lane_id) {
                lane_Connectivitys.push_back(map_static_info->LaneConnectivitys.PairConnectivity[index]);
            }
        }
    } else {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << " can't find connect : " << link_id << std::endl;
        return false;
    }
    
    if(lane_Connectivitys.size()>0){
        return true;
    }else{
        return false;
    }
    
}

bool MapCommonTool::GetLine(const uint32_t line_index, const MapStaticInfo& map_static_info, const std::unordered_map<uint32_t, int>& linear_obj_id_map,
                             std::vector<message::map_map::s_GeometryPoint_t>& geometry_points, uint8_t& mrk_type,uint8_t& mrk_color) {
    geometry_points.clear();
    mrk_type = 0;
    mrk_color = 0;
    if(linear_obj_id_map.find(line_index) != linear_obj_id_map.end()){
        int index = linear_obj_id_map.at(line_index);
        geometry_points = map_static_info.LinearObjects.LinearObjects[index].GeometryPoints.GeometryPoints;
        mrk_type = map_static_info.LinearObjects.LinearObjects[index].LinearObjectMarking.data;
        mrk_color = map_static_info.LinearObjects.LinearObjects[index].LinearObjectColour.data;
        return true;
    }
    return false;
}

bool MapCommonTool::GetLineType(const uint32_t line_index, const MapStaticInfo& map_static_info, const std::unordered_map<uint32_t, int>& linear_obj_id_map,uint8_t& mrk_type) {
    mrk_type = 0;
    if(linear_obj_id_map.find(line_index) != linear_obj_id_map.end()){
        int index = linear_obj_id_map.at(line_index);
        mrk_type = map_static_info.LinearObjects.LinearObjects[index].LinearObjectMarking.data;
        return true;
    }
    return false;
}

bool MapCommonTool::SaveLaneCurvInfo(const uint8_t& lane_id, const uint32_t& link_id,const std::unordered_map<uint64_t, int>& curve_index_map,
                                            const MapStaticInfo& map_static_info,std::vector<CurvPoint>& curvpoints){
    // std::vector<CurvPoint> curvpoints{};
    // std::cout << __FILE__ << __LINE__ << "lane_info_raw.LaneNum.LaneNum: " << int(lane_info_raw.LaneNum.LaneNum) << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_id: " << link_id << std::endl;
    curvpoints.clear();
    CurvPoint curvpoint{};
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    if(curve_index_map.find(key_temp) != curve_index_map.end()){
        int index = curve_index_map.at(key_temp);
        auto& link_curv_info =  map_static_info.LinkCurvatures.LinkCurvatures[index];
            for(auto& curvpoint_info: link_curv_info.CurvPoints.CurvPoints){
                // std::cout << __FILE__ << __LINE__ << "curvpoint_info.CurvPointValue.CurvPointValue: " << curvpoint_info.CurvPointValue.CurvPointValue << std::endl;
                // std::cout << __FILE__ << __LINE__ << "curvpoint_info.PathOffset.PathOffset: " << int(curvpoint_info.PathOffset.PathOffset + link_curv_info.PathOffset.PathOffset) << std::endl;
                curvpoint.CurvPointValue = curvpoint_info.CurvPointValue.CurvPointValue / 100000.0f;
                curvpoint.CurvPointPathOffset = curvpoint_info.PathOffset.PathOffset + link_curv_info.PathOffset.PathOffset;
                curvpoint.linkid = link_id;
                curvpoints.push_back(curvpoint);
            }        
    }else{
        return false;
    }
    return true;
}

bool MapCommonTool::GetLaneSpeed(
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info, uint32_t link_id,
    uint8_t lane_id, message::map_map::s_LaneSpeedLimit_t& lane_speed_limit) {
    for (auto& lane_speed_tmp : map_static_info->LaneSpeedLimitis.LaneSpeedLimits) {
        if (lane_speed_tmp.InstanceId.InstanceId == link_id && lane_speed_tmp.LaneNum.LaneNum == lane_id) {
            lane_speed_limit = lane_speed_tmp;
            return true;
        }
    }
    return false;
}

bool MapCommonTool::GetTwoLineDistance(const MapStaticInfo& map_static_info,uint32_t left_line, uint32_t right_line,
                                          double& start_distance, double& end_distance) {
    if (0 == left_line || 0 == right_line){
        return false;
    }
    
    std::vector<EFMPoint> left_geometry_points;
    if (false == GetLineGeometry(map_static_info,left_line, left_geometry_points)){
        return false;
    }
    
    std::vector<EFMPoint> right_geometry_points;
    if (false == GetLineGeometry(map_static_info,right_line, right_geometry_points)){
        return false;
    }

    if (left_geometry_points.size() < 2 || right_geometry_points.size() < 2){
        return false;
    }

    CommonTool::CoordinateTool::GetInstance()->calculateDistanceGC02(left_geometry_points[0].x,
                                                                 left_geometry_points[0].y,
                                                                 0.0,
                                                                 right_geometry_points[0].x,
                                                                 right_geometry_points[0].y,
                                                                 0.0,
                                                                 start_distance);
    start_distance = start_distance *100.0;

    CommonTool::CoordinateTool::GetInstance()->calculateDistanceGC02(left_geometry_points[left_geometry_points.size() - 1].x,
                                                                 left_geometry_points[left_geometry_points.size() - 1].y,
                                                                 0.0,
                                                                 right_geometry_points[right_geometry_points.size() - 1].x,
                                                                 right_geometry_points[right_geometry_points.size() - 1].y,
                                                                 0.0,
                                                                 end_distance);
    end_distance = end_distance *100.0;

    return true;
}

bool MapCommonTool::GetLineGeometry(const MapStaticInfo& map_static_info,uint32_t line_id, std::vector<EFMPoint>& geometry_points) {

    geometry_points.clear();
    if (0 == line_id){
        return false;
    }
    
    for (auto& lineinfo : map_static_info.LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            int point_max_size = (lineinfo.PointCount.PointCount < lineinfo.GeometryPoints.GeometryPoints.size())
                                        ? lineinfo.PointCount.PointCount
                                        : lineinfo.GeometryPoints.GeometryPoints.size();
            for (int point_idx = 0; point_idx < point_max_size; point_idx++) {
                EFMPoint temp_point;
                temp_point.x =
                    static_cast<double>(lineinfo.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                    360.0 / (pow(2, 32));
                temp_point.y =
                    static_cast<double>(lineinfo.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                    360.0 / (pow(2, 32));
                geometry_points.push_back(temp_point);
            }
            break;
        }
    }

    if (geometry_points.empty()){
        return false;
    }
    
    return true;
}

}  // namespace efm
