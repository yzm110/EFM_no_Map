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
#include "ExtractRefLine.h"

#define SPLIT_BACK_LENGTH 50  // m
#define VIRTUAL_LINE_STEP 50  // m

namespace earth {
namespace shell {
namespace framework {

bool ExtractRefLine::Execute(const message::map_position::s_Position_t& map_position,
                             const TopicTrait::MapMapMsg& map_static_info,
                             const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                             const std::unordered_map<uint32_t, std::vector<int>> link_id_index_lane_connect_map,
                             const std::unordered_map<uint32_t, int> linear_obj_id_map,const std::unordered_map<uint64_t, int> curve_index_map,const std::unordered_map<uint64_t, int> lane_widths_map,
                             const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ExtractRefLine::Execute: " << std::endl;
    // plot merge info
    // std::cout << "AllMergePoints: " << std::endl;
    // for (auto merge : map_static_info.AllMergePoints.AllMergePoints) {
    //    std::cout << "link_id: " << merge.InstanceId.InstanceId
    //              << " ,lane_num: " << static_cast<int>(merge.LaneNum.LaneNum) << std::endl;
    //    for (auto merge_point : merge.MergePoints.MergePoints) {
    //        std::cout << "dist: "
    //                  << (static_cast<double>(merge_point.PathOffset.PathOffset) -
    //                      static_cast<double>(map_position.PathOffset)) /
    //                         100.0
    //                  << " ,isMaster: " << static_cast<int>(merge_point.IsMaster.IsMaster) << std::endl;
    //    }
    //}

#endif
    // get input
    // ego_motion_ = std::make_shared<const message::C2C_Message::s_EgoMotion2_t>(ego_motion);
    map_position_ = std::make_shared<const message::map_position::s_Position_t>(map_position);
    // map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_static_info);
    link_id_index_lane_info_map_ =
        std::make_shared<const std::unordered_map<uint32_t, int>>(link_id_index_lane_info_map);
    link_id_index_lane_connect_map_ = std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(link_id_index_lane_connect_map);   
    linear_obj_id_map_ = std::make_shared<const std::unordered_map<uint32_t, int>>(linear_obj_id_map);
    curve_index_map_ = std::make_shared<std::unordered_map<uint64_t, int>>( curve_index_map);
    lane_widths_map_ = std::make_shared<std::unordered_map<uint64_t, int>>( lane_widths_map);

    link_id_vec_ = candidate_lanes_model->link_id_vec();
    link_length_vec_ = candidate_lanes_model->link_length_vec();
    all_lanes_vec_vec_ = candidate_lanes_model->all_lanes_vec_vec();
    ego_lane_prior_index_ = candidate_lanes_model->ego_lane_prior_index();
    left_lane_prior_index_ = candidate_lanes_model->left_lane_prior_index();
    left_left_lane_prior_index_ = candidate_lanes_model->left_left_lane_prior_index();
    right_lane_prior_index_ = candidate_lanes_model->right_lane_prior_index();
    right_right_lane_prior_index_ = candidate_lanes_model->right_right_lane_prior_index();

    back_ego_link_id_vec_ = candidate_lanes_model->back_ego_link_id_vec();
    back_left_link_id_vec_ = candidate_lanes_model->back_left_link_id_vec();
    back_left_left_link_id_vec_ = candidate_lanes_model->back_left_left_link_id_vec();
    back_right_link_id_vec_ = candidate_lanes_model->back_right_link_id_vec();
    back_right_right_link_id_vec_ = candidate_lanes_model->back_right_right_link_id_vec();
    back_ego_lane_num_vec_ = candidate_lanes_model->back_ego_lane_num_vec();
    back_left_lane_num_vec_ = candidate_lanes_model->back_left_lane_num_vec();
    back_left_left_lane_num_vec_ = candidate_lanes_model->back_left_left_lane_num_vec();
    back_right_lane_num_vec_ = candidate_lanes_model->back_right_lane_num_vec();
    back_right_right_lane_num_vec_ = candidate_lanes_model->back_right_right_lane_num_vec();
    back_emergency_left_link_id_vec_ = candidate_lanes_model->back_emergency_left_link_id_vec();
    back_emergency_right_link_id_vec_ = candidate_lanes_model->back_emergency_right_link_id_vec();
    back_emergency_left_lane_num_vec_= candidate_lanes_model->back_emergency_left_lane_num_vec();
    back_emergency_right_lane_num_vec_= candidate_lanes_model->back_emergency_right_lane_num_vec();
    all_emergency_lanes_vec_vec_ = candidate_lanes_model->all_emergency_lanes_vec_vec();

// #undef ERL_COUT
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " all_lanes_vec_vec_.size(): " << all_lanes_vec_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " all_lanes_vec_vec_:: " << std::endl;
    for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
        std::cout << " all_lanes_vec_vec_[ " << i << " ]::  [";
        for (auto lane_num : all_lanes_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(lane_num) << ",";
        }
        std::cout << "]" << std::endl;
    }
#endif
    EFMRefLine init_path{};
    ego_path_ = init_path;
    left_path_ = init_path;
    left_left_path_ = init_path;
    right_path_ = init_path;
    right_right_path_ = init_path;
    emergency_right_path_ = init_path;
    emergency_left_path_ = init_path;
    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    // ego_path_for_lane_maker_ = init_path;
    // left_path_for_lane_maker_ = init_path;
    // right_path_for_lane_maker_ = init_path;
    prior_path_index_ = 0;

    SetRefPath(candidate_lanes_model);
    // decision_info_ = init_decision_info;

    // only for lane marker
// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << " only for lane marker: start" << std::endl;
// #endif
//     GeneratePaths(candidate_lanes_model->ego_lane_index_for_lane_mkr(),
//                   candidate_lanes_model->left_lane_index_for_lane_mkr(),
//                   candidate_lanes_model->right_lane_index_for_lane_mkr(), ego_path_for_lane_maker_,
//                   left_path_for_lane_maker_, right_path_for_lane_maker_);
//     // CutLineToFixRange(ego_path_for_lane_maker_, left_path_for_lane_maker_, right_path_for_lane_maker_);
// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << " only for lane marker: end" << std::endl;
// #endif
    ////

    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePaths(ego_lane_prior_index_, left_lane_prior_index_, left_left_lane_prior_index_, right_lane_prior_index_, 
                  right_right_lane_prior_index_, candidate_lanes_model->emergency_left_index(),candidate_lanes_model->emergency_right_index(),
                ego_path_, left_path_, left_left_path_, right_path_, right_right_path_, emergency_left_path_, emergency_right_path_);
    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePathsAttribute(ego_path_, left_path_, left_left_path_, right_path_, right_right_path_);
    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    SetPathAvailable(candidate_lanes_model, ego_path_, left_path_, left_left_path_, right_path_, right_right_path_);
    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    CalPriorIndex(candidate_lanes_model, ego_path_);
    //std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    // CutLineToFixRange(ego_path_, left_path_, right_path_);

    // plot
    double merge_dist = ego_path_.merge_dist;
    uint8_t merge_type = static_cast<uint8_t>(ego_path_.merge_type);
    double left_merge_dist = left_path_.merge_dist;
    uint8_t left_merge_type = left_path_.merge_type;
    double right_merge_dist = right_path_.merge_dist;
    uint8_t right_merge_type = right_path_.merge_type;
#ifdef EFM_PLOT2D
    auto rect = rotatedRect(0, 0, 2, 1, 0);
    auto& self_car = rect.first;
    auto& self_car_y = rect.second;
    ZPLOTXYF("EFM_INFO", "-black2", self_car, self_car_y);
#endif
    return true;
}

bool ExtractRefLine::SetRefPath(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
    int32_t ego_lane_idx = candidate_lanes_model->ego_lane_prior_index();
    int32_t left_lane_idx = candidate_lanes_model->left_lane_prior_index();
    int32_t left_left_lane_idx = candidate_lanes_model->left_left_lane_prior_index();
    int32_t right_lane_idx = candidate_lanes_model->right_lane_prior_index();
    int32_t right_right_lane_idx = candidate_lanes_model->right_right_lane_prior_index();
    int32_t ref_lane_idx = candidate_lanes_model->get_ref_lane_prior_index();
    if (ref_lane_idx == left_lane_idx) {
        // left_path_.is_ref_line = true;
        // left_path_.split_position_index = candidate_lanes_model->get_ref_lane_split_position();
        // left_path_.next_position_index = candidate_lanes_model->get_ref_lane_next_split_position();
        // left_path_.split_from = candidate_lanes_model->get_ref_lane_split_from();
        return true;
    } else if (ref_lane_idx == right_lane_idx) {
        return true;
    } else if (ref_lane_idx == left_left_lane_idx) {
        return true;
    } else if (ref_lane_idx == right_right_lane_idx) {
        return true;
    } else {
        ego_path_.is_ref_line = true;
        ego_path_.split_position_index = candidate_lanes_model->get_ref_lane_split_position();
        ego_path_.next_position_index = candidate_lanes_model->get_ref_lane_next_split_position();
        ego_path_.split_from = candidate_lanes_model->get_ref_lane_split_from();
    }
    return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool ExtractRefLine::GeneratePaths(int ego_lane_prior_index, int left_lane_prior_index, int left_left_lane_prior_index,
                                   int right_lane_prior_index, int right_right_lane_prior_index,int32_t emergency_left_index,int32_t emergency_right_index,
                                   EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path,
                                   EFMRefLine& right_path, EFMRefLine& right_right_path, EFMRefLine& emergency_left_path, EFMRefLine& emergency_right_path) {
    std::vector<uint8_t> ego_lane{};
    std::vector<uint8_t> left_lane{};
    std::vector<uint8_t> left_left_lane{};
    std::vector<uint8_t> right_lane{};
    std::vector<uint8_t> right_right_lane{};
// EFMRefLinePoints ego_path_utm, left_path_utm, right_path_utm;
// #undef ERL_COUT
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_lane_prior_index: " << ego_lane_prior_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_lane_prior_index: " << left_lane_prior_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_lane_prior_index: " << right_lane_prior_index << std::endl;
#endif
    double ego_utm_x, ego_utm_y;
    double ego_lon, ego_lat;
    ego_lon = map_position_->Lon.Lon * 360.0 / (4294967296);
    ego_lat = map_position_->Lat.Lat * 360.0 / (4294967296);
    auto instance = CommonTool::CoordinateTool::GetInstance();
    if (instance != nullptr) {
        instance->wgs84toUTM2(ego_lat, ego_lon, ego_utm_x, ego_utm_y, ego_lat, ego_lon);
    } else {
        LOGI("CommonTool::CoordinateTool::GetInstance(): nullptr");
    }

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_lane, GenerateOnePathLine" << std::endl;
#endif
// std::cout<<"########################################0000000000000000000000000000000000"<<std::endl;
    if (ego_lane_prior_index >= 0 && ego_lane_prior_index < all_lanes_vec_vec_.size()) {
        ego_lane = all_lanes_vec_vec_[ego_lane_prior_index];
        GenerateOnePathLine(ego_lane, link_id_vec_, back_ego_link_id_vec_, back_ego_lane_num_vec_, ego_path);
        GetLaneCurvature(ego_lane, link_id_vec_, ego_path);
        //RerangeLaneMarkerSection(ego_path);
        MakePath(ego_utm_x, ego_utm_y, ego_path);
        
    }
#ifdef SMOOTH
    PlotSmoothSplitInfo("ego_path",ego_path.smooth_split_info);
#endif
    //std::cout<<"########################################1111111111111111111"<<std::endl;
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_lane, GenerateOnePathLine" << std::endl;
#endif
    if (left_lane_prior_index >= 0 && left_lane_prior_index < all_lanes_vec_vec_.size()) {
        left_lane = all_lanes_vec_vec_[left_lane_prior_index];
        GenerateOnePathLine(left_lane, link_id_vec_, back_left_link_id_vec_, back_left_lane_num_vec_, left_path);
        GetLaneCurvature(left_lane, link_id_vec_, left_path);
        //RerangeLaneMarkerSection(left_path);
        MakePath(ego_utm_x, ego_utm_y, left_path);
        
    }

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_lane, GenerateOnePathLine" << std::endl;
#endif
    if (right_lane_prior_index >= 0 && right_lane_prior_index < all_lanes_vec_vec_.size()) {
        right_lane = all_lanes_vec_vec_[right_lane_prior_index];
        GenerateOnePathLine(right_lane, link_id_vec_, back_right_link_id_vec_, back_right_lane_num_vec_, right_path);
        GetLaneCurvature(right_lane, link_id_vec_, right_path);
        //RerangeLaneMarkerSection(right_path);
        MakePath(ego_utm_x, ego_utm_y, right_path);
        
    }
#ifdef SMOOTH
    PlotSmoothSplitInfo("right_path",right_path.smooth_split_info);
#endif
//std::cout<<"########################################33333333333333333333333333"<<std::endl;
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_left_lane, GenerateOnePathLine" << std::endl;
#endif
    if (left_left_lane_prior_index >= 0 && left_left_lane_prior_index < all_lanes_vec_vec_.size()) {
        left_left_lane = all_lanes_vec_vec_[left_left_lane_prior_index];
        GenerateOnePathLine(left_left_lane, link_id_vec_, back_left_left_link_id_vec_, back_left_left_lane_num_vec_, left_left_path);
        GetLaneCurvature(left_left_lane, link_id_vec_, left_left_path);
        //RerangeLaneMarkerSection(right_path);
        MakePath(ego_utm_x, ego_utm_y, left_left_path);
        
    }

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_right_lane, GenerateOnePathLine" << std::endl;
#endif
    if (right_right_lane_prior_index >= 0 && right_right_lane_prior_index < all_lanes_vec_vec_.size()) {
        right_right_lane = all_lanes_vec_vec_[right_right_lane_prior_index];
        GenerateOnePathLine(right_right_lane, link_id_vec_, back_right_right_link_id_vec_, back_right_right_lane_num_vec_, right_right_path);
        GetLaneCurvature(right_right_lane, link_id_vec_, right_right_path);
        //RerangeLaneMarkerSection(right_path);
        MakePath(ego_utm_x, ego_utm_y, right_right_path);
        
    }

    //emergency lanes
    if(emergency_left_index>=0 && emergency_left_index< all_emergency_lanes_vec_vec_.size()){
        std::vector<uint8_t> emergency_left_lane = all_emergency_lanes_vec_vec_[emergency_left_index];
        GenerateOnePathLine(emergency_left_lane, link_id_vec_, back_emergency_left_link_id_vec_, back_emergency_left_lane_num_vec_, emergency_left_path);
        MakePath(ego_utm_x, ego_utm_y, emergency_left_path);
    }
    if(emergency_right_index>=0 && emergency_right_index< all_emergency_lanes_vec_vec_.size()){
        std::vector<uint8_t> emergency_right_lane = all_emergency_lanes_vec_vec_[emergency_right_index];
        GenerateOnePathLine(emergency_right_lane, link_id_vec_, back_emergency_right_link_id_vec_, back_emergency_right_lane_num_vec_, emergency_right_path);
        MakePath(ego_utm_x, ego_utm_y, emergency_right_path);
    }

    return true;
}

bool ExtractRefLine::GenerateOnePathLine(const std::vector<uint8_t>& candidate_lane,
                                         const std::vector<uint32_t>& link_id_vec,
                                         const std::vector<uint32_t>& back_link_id_vec,
                                         const std::vector<uint8_t>& back_lane_num_vec, EFMRefLine& path) {
    if (link_id_vec.size() <= 0 || candidate_lane.size() <= 0 || candidate_lane.size() > link_id_vec.size()) {
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "GenerateOnePathLine start" << std::endl;
    EFMRefLinePoints& path_points = path.linePoints;
    EFMRefLineMarkr& left_mrk = path.leftMarkr;
    EFMRefLineMarkr& right_mrk = path.rightMarkr;
    std::vector<uint32_t>& path_points_offset = path.linePointsOffset;
    std::vector<uint32_t>& left_mrk_offset = path.leftMarkrOffset;
    std::vector<uint32_t>& right_mrk_offset = path.rightMarkrOffset;
    std::vector<LaneLineOffsetInfo_s>& line_type_offset = path.line_type_offset;
    // EFMRefLineMarkrSection& left_mrk_section = path.leftMarkrSection;
    // EFMRefLineMarkrSection& right_mrk_section = path.rightMarkrSection;
    path_points.clear();
    left_mrk.clear();
    right_mrk.clear();
    path_points_offset.clear();
    left_mrk_offset.clear();
    right_mrk_offset.clear();
    // left_mrk_section.clear();
    // right_mrk_section.clear();
    path.leftLineMkrInfos.clear();   // for smooth
    path.rightLineMkrInfos.clear();  // for smooth
    path.back_link_offsets.clear();
    path.back_link_left_line_infos.clear();
    path.back_link_right_line_infos.clear();
    path.back_lane_split_to = 0;
    path.back_lane_split_point_index = -1;
    path.back_lane_is_split = false;
    path.split_start_s_vec.clear();
    path.merge_end_s_vec.clear();
    path.smooth_split_info.clear();
    line_type_offset.clear();
    uint32_t ego_offset = map_position_->PathOffset;
    // fill back
    // back_link_id_vec[0] is nearest ego position!!
        std::vector<uint32_t> back_link_id_vec_in_loop{};//for smooth
        std::vector<uint8_t> back_lane_id_vec_in_loop{};//for smooth
        std::vector<uint32_t> foward_link_id_vec_in_loop{};//for smooth,还剩余的
        std::vector<uint8_t> foward_lane_id_vec_in_loop{};//for smooth
        // back front的拼起来
        foward_link_id_vec_in_loop =back_link_id_vec;
        std::reverse(foward_link_id_vec_in_loop.begin(),foward_link_id_vec_in_loop.end());
        foward_link_id_vec_in_loop.insert(foward_link_id_vec_in_loop.end(),link_id_vec.begin(),link_id_vec.end());

        foward_lane_id_vec_in_loop =back_lane_num_vec;
        std::reverse(foward_lane_id_vec_in_loop.begin(),foward_lane_id_vec_in_loop.end());
        foward_lane_id_vec_in_loop.insert(foward_lane_id_vec_in_loop.end(),candidate_lane.begin(),candidate_lane.end());   

    if (back_link_id_vec.size() != back_lane_num_vec.size()) {
        // not fill back
    } else {  
        for (int i = back_link_id_vec.size() - 1; i >= 0; i--) {
            uint32_t link_id = back_link_id_vec[i];
            uint32_t next_link_id = 0;
            if(i == 0 && (link_id_vec.empty() == false)){
                next_link_id = link_id_vec[0];
            }else{
                if(i>0){
                    next_link_id = back_link_id_vec[i-1];
                }              
            }
            uint8_t lane_num = back_lane_num_vec[i];
            message::map_map::s_LinkInfo_t link_infos{};
            if (!GetLinkInfos(link_id, link_infos)) {
#ifdef EXTRACTREFLINE_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "can't find link id: " << link_id << std::endl;
#endif
                break;
            }
            {  // center line
                uint32_t center_line_index = 0;
                if (!GetLaneCenterLine(link_infos, lane_num, center_line_index)) {
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "can't find lane center line!!! linkId: " << link_id << "lane_num: " << lane_num <<
                    //           std::endl;
                    break;
                }
                std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
                uint8_t center_type = 0;
                uint8_t center_color = 0;
                // if (!GetLine(center_line_index, geometry_points, center_type, center_color)) {
                if(!efm::MapCommonTool::GetInstance()->GetLine(center_line_index, *map_static_info_, *linear_obj_id_map_,
                             geometry_points, center_type,center_color)){
#ifdef EXTRACTREFLINE_COUT
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << "can't find center line!!! " << center_line_index << std::endl;
#endif
                    break;
                }
                path.back_lane_split_point_index = path_points.size();
                EFMRefLinePoints ref_line_points{};       
                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(geometry_points, ref_line_points, path_points_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));
                for (auto point : ref_line_points) {
                    path_points.push_back(point);
                }

                if (path.is_ref_line){
                    
                    double temp_offset = (link_infos.EndOffset.EndOffset - map_position_->PathOffset) / 100.0;
                    path.back_link_offsets.push_back(temp_offset > 0 ? temp_offset : 0);
                    uint8_t trans = 0;
                    GetLaneTrans(link_infos, lane_num, trans);
                    if (trans == 3){
                        path.back_lane_is_split = true;
                        GetSplitFrom(link_infos, lane_num, path.back_lane_split_to);
                    }
                    
                }
                //get split s for smooth, 
                // std::cout << __FILE__ << "," << __LINE__ << "," << "GetLaneSplitStartS back    "<< std::endl;
                GetLaneSplitStartS(ego_offset, link_id, lane_num,link_infos,next_link_id,path.split_start_s_vec);

                if(foward_link_id_vec_in_loop.size()>0){
                    foward_link_id_vec_in_loop.erase(foward_link_id_vec_in_loop.begin());
                }      
                if(foward_lane_id_vec_in_loop.size()>0){
                    foward_lane_id_vec_in_loop.erase(foward_lane_id_vec_in_loop.begin());
                }
                SmoothSplitInfo smooth_split_info{};          
                if(GetLaneSplitSmoothInfo(ego_offset, link_id, lane_num,foward_link_id_vec_in_loop, foward_lane_id_vec_in_loop, back_link_id_vec_in_loop, back_lane_id_vec_in_loop,smooth_split_info)){
                    path.smooth_split_info.push_back(smooth_split_info);
                }
                uint32_t last_link_id_temp = 0;
                if (i < back_link_id_vec.size()-1 ) {
                    last_link_id_temp = back_link_id_vec[i + 1];
                }
                GetLaneMergeEndS(ego_offset, last_link_id_temp, lane_num, link_infos, path.merge_end_s_vec);          
                
            }

            uint8_t left_type = 0;
            {  // left line
                uint32_t left_line_index = 0;
                if (!GetLaneLeftLine(link_infos, lane_num, left_line_index)) {
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "can't find lane left line!!! linkId: " << link_id << "lane_num: " << lane_num <<
                    //           std::endl;
                    break;
                }
                std::vector<message::map_map::s_GeometryPoint_t> left_geometry_points;
                uint8_t left_color = 0;
                // if (!GetLine(left_line_index, left_geometry_points, left_type, left_color)) {
                if(!efm::MapCommonTool::GetInstance()->GetLine(left_line_index, *map_static_info_, *linear_obj_id_map_,
                             left_geometry_points, left_type,left_color)){
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "can't find left line!!! " << center_line_index << std::endl;
                    break;
                }
                LineMkrInfo_s infos_tmp(link_infos.PathOffset.PathOffset,link_infos.EndOffset.EndOffset,left_type, left_color);
                path.back_link_left_line_infos.push_back(infos_tmp);
                EFMRefLinePoints left_line_points{};
                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(left_geometry_points, left_line_points, left_mrk_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));

                // if (left_type != 8) 
                {
                    EFMMarkr lane_markr{};
                    lane_markr.type = left_type;
                    lane_markr.color = left_color;
                    //std::vector<EFMMarkr> lane_markr_vec{};
                    for (auto point : left_line_points) {
                        lane_markr.x = point.x;
                        lane_markr.y = point.y;
                        //lane_markr_vec.push_back(lane_markr);
                        left_mrk.push_back(lane_markr);
                    }
                    // if (!lane_markr_vec.empty()) {
                    //     left_mrk_section.push_back(lane_markr_vec);
                    // }
                }
            }

            uint8_t right_type = 0;
            {  // right line
                uint32_t right_line_index = 0;
                if (!GetLaneRightLine(link_infos, lane_num, right_line_index)) {
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "can't find lane left line!!! linkId: " << link_id << "lane_num: " << lane_num <<
                    //           std::endl;
                    break;
                }
                std::vector<message::map_map::s_GeometryPoint_t> right_geometry_points;
                uint8_t right_color = 0;
                // if (!GetLine(right_line_index, right_geometry_points, right_type, right_color)) {
                if(!efm::MapCommonTool::GetInstance()->GetLine(right_line_index, *map_static_info_, *linear_obj_id_map_,
                             right_geometry_points, right_type,right_color)){
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "can't find left line!!! " << center_line_index << std::endl;
                    break;
                }
                LineMkrInfo_s infos_tmp(link_infos.PathOffset.PathOffset,link_infos.EndOffset.EndOffset,right_type, right_color);
                path.back_link_right_line_infos.push_back(infos_tmp);
                EFMRefLinePoints right_line_points{};
                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(right_geometry_points, right_line_points, right_mrk_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));

                // if (right_type != 8) 
                {
                    EFMMarkr lane_markr{};
                    lane_markr.type = right_type;
                    lane_markr.color = right_color;
                    //std::vector<EFMMarkr> lane_markr_vec{};
                    for (auto point : right_line_points) {
                        lane_markr.x = point.x;
                        lane_markr.y = point.y;
                        //lane_markr_vec.push_back(lane_markr);
                        right_mrk.push_back(lane_markr);
                    }
                    // if (!lane_markr_vec.empty()) {
                    //     right_mrk_section.push_back(lane_markr_vec);
                    // }
                }
            }

            {//save smooth used info
                LaneLineOffsetInfo_s line_type_offset_tmp{};
                line_type_offset_tmp.left_line_type = left_type;
                line_type_offset_tmp.right_line_type = right_type;
                line_type_offset_tmp.start_offset = link_infos.PathOffset.PathOffset;
                line_type_offset_tmp.end_offset = link_infos.EndOffset.EndOffset;
                line_type_offset.push_back(line_type_offset_tmp);
            }
            back_link_id_vec_in_loop.push_back(link_id);
            back_lane_id_vec_in_loop.push_back(lane_num);
        }
    }
    // fill front
    path.split_in_point_index = -1;
    path.next_in_point_index  = -1;
    for (int i = 0; i < candidate_lane.size(); i++) {
        uint8_t lane_num = candidate_lane[i];
        uint32_t link_id = link_id_vec[i];
        uint32_t next_link_id = 0;
        if((i+1)<candidate_lane.size()){
            next_link_id = link_id_vec[i+1];
        }
        message::map_map::s_LinkInfo_t link_infos;
        if (!GetLinkInfos(link_id, link_infos)) {
#ifdef EXTRACTREFLINE_COUT
            std::cout << __FILE__ << "," << __LINE__ << ","
                      << "can't find link id: " << link_id << std::endl;
#endif
            break;
        }
        uint32_t center_line_index = 0;
        if (!GetLaneCenterLine(link_infos, lane_num, center_line_index)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find lane center line!!! linkId: " << link_id << "lane_num: " << lane_num <<
            //           std::endl;
            break;
        }
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        uint8_t c_type = 0;
        uint8_t c_color = 0;
        // if (!GetLine(center_line_index, geometry_points, c_type, c_color)) {
        if(!efm::MapCommonTool::GetInstance()->GetLine(center_line_index, *map_static_info_, *linear_obj_id_map_,
                             geometry_points, c_type,c_color)){
#ifdef EXTRACTREFLINE_COUT
            std::cout << __FILE__ << "," << __LINE__ << ","
                      << "can't find center line!!! " << center_line_index << std::endl;
#endif
            break;
        }
        //get split point
        if (path.is_ref_line && path.split_position_index == i){
            path.split_in_point_index = path_points.size();
            //std::cout << __FILE__ << "," << __LINE__ << "," << "split_in_point_index:" << path.split_in_point_index<< std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "lane_num:" << lane_num << std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "link_id:" << link_id << std::endl;
        }

        if (path.is_ref_line && path.next_position_index == i && path.next_position_index > path.split_position_index){
            path.next_in_point_index = path_points.size();
            //std::cout << __FILE__ << "," << __LINE__ << "," << "split_in_point_index:" << path.split_in_point_index<< std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "lane_num:" << lane_num << std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "link_id:" << link_id << std::endl;
        }

        if (path.is_ref_line && 0 == i){
            uint8_t trans = 0;
            GetLaneTrans(link_infos, lane_num, trans);
            if (trans == 3){
                path.back_lane_is_split = true;
                GetSplitFrom(link_infos, lane_num, path.back_lane_split_to);
                path.back_lane_split_point_index = path_points.size();
            }
        }
        //get split s for smooth, 
        // std::cout << __FILE__ << "," << __LINE__ << "," << "GetLaneSplitStartS front    "<< std::endl;
        GetLaneSplitStartS(ego_offset, link_id, lane_num,link_infos,next_link_id,path.split_start_s_vec);
                if(foward_link_id_vec_in_loop.size()>0){
                    foward_link_id_vec_in_loop.erase(foward_link_id_vec_in_loop.begin());
                }      
                if(foward_lane_id_vec_in_loop.size()>0){
                    foward_lane_id_vec_in_loop.erase(foward_lane_id_vec_in_loop.begin());
                }
                SmoothSplitInfo smooth_split_info{};          
                if(GetLaneSplitSmoothInfo(ego_offset, link_id, lane_num,foward_link_id_vec_in_loop, foward_lane_id_vec_in_loop, back_link_id_vec_in_loop, back_lane_id_vec_in_loop,smooth_split_info)){
                    path.smooth_split_info.push_back(smooth_split_info);
                }
        uint32_t last_link_id_temp = 0;
        if(i == 0 && back_link_id_vec.size()>0){
              last_link_id_temp = back_link_id_vec[0];
        }else{
            if(i>0 && (i-1)<link_id_vec.size()){
               last_link_id_temp = link_id_vec[i-1];
            }
        }
        GetLaneMergeEndS(ego_offset, last_link_id_temp, lane_num,link_infos, path.merge_end_s_vec);     
        
        
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(geometry_points, ref_line_points, path_points_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));
        for (auto point : ref_line_points) {
            path_points.push_back(point);
        }

        // left line
        uint32_t left_line_index = 0;
        if (!GetLaneLeftLine(link_infos, lane_num, left_line_index)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find lane left line!!! linkId: " << link_id << "lane_num: " << lane_num <<
            //           std::endl;
            break;
        }
        std::vector<message::map_map::s_GeometryPoint_t> left_geometry_points;
        uint8_t left_type = 0;
        uint8_t left_color = 0;
        // if (!GetLine(left_line_index, left_geometry_points, left_type, left_color)) {
        if(!efm::MapCommonTool::GetInstance()->GetLine(left_line_index, *map_static_info_, *linear_obj_id_map_,
                             left_geometry_points, left_type,left_color)){
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find left line!!! " << left_line_index << std::endl;
            break;
        }
        EFMRefLinePoints left_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(left_geometry_points, left_line_points, left_mrk_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));

        // if (left_type != 8) {
        {
            EFMMarkr lane_markr{};
            lane_markr.type = left_type;
            lane_markr.color = left_color;
            //std::vector<EFMMarkr> lane_markr_vec{};
            for (auto point : left_line_points) {
                lane_markr.x = point.x;
                lane_markr.y = point.y;
                //lane_markr_vec.push_back(lane_markr);
                left_mrk.push_back(lane_markr);
            }
            // if (!lane_markr_vec.empty()) {
            //     left_mrk_section.push_back(lane_markr_vec);
            // }
        }
        LineMkrInfo_s infos_le(link_infos.PathOffset.PathOffset,link_infos.EndOffset.EndOffset,left_type, left_color);
        path.leftLineMkrInfos.push_back(infos_le);

        // right line
        uint32_t right_line_index = 0;
        if (!GetLaneRightLine(link_infos, lane_num, right_line_index)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find lane right line!!! linkId: " << link_id << "lane_num: " << lane_num <<
            //           std::endl;
            break;
        }
        std::vector<message::map_map::s_GeometryPoint_t> right_geometry_points;
        uint8_t right_type = 0;
        uint8_t right_color = 0;
        // if (!GetLine(right_line_index, right_geometry_points, right_type, right_color)) {
        if(!efm::MapCommonTool::GetInstance()->GetLine(right_line_index, *map_static_info_, *linear_obj_id_map_,
                             right_geometry_points, right_type,right_color)){
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find right line!!! " << right_line_index << std::endl;
            break;
        }
        EFMRefLinePoints right_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(right_geometry_points, right_line_points, right_mrk_offset,map_position_,link_infos.PathOffset.PathOffset,static_cast<double>(map_position_->Heading.Heading));

        // if (right_type != 8) 
        {
            EFMMarkr lane_markr{};
            lane_markr.type = right_type;
            lane_markr.color = right_color;
            //std::vector<EFMMarkr> lane_markr_vec{};
            for (auto point : right_line_points) {
                lane_markr.x = point.x;
                lane_markr.y = point.y;
                //lane_markr_vec.push_back(lane_markr);
                right_mrk.push_back(lane_markr);
            }
            // if (!lane_markr_vec.empty()) {
            //     right_mrk_section.push_back(lane_markr_vec);
            // }
        }
        {//save smooth used info
            LaneLineOffsetInfo_s line_type_offset_tmp{};
            line_type_offset_tmp.left_line_type = left_type;
            line_type_offset_tmp.right_line_type = right_type;
            line_type_offset_tmp.start_offset = link_infos.PathOffset.PathOffset;
            line_type_offset_tmp.end_offset = link_infos.EndOffset.EndOffset;
            line_type_offset.push_back(line_type_offset_tmp);
        }
        LineMkrInfo_s infos_ri(link_infos.PathOffset.PathOffset,link_infos.EndOffset.EndOffset,right_type, right_color);
        path.rightLineMkrInfos.push_back(infos_ri);

        back_link_id_vec_in_loop.push_back(link_id);
        back_lane_id_vec_in_loop.push_back(lane_num);
        if (link_infos.EndOffset.EndOffset - ego_offset >= 20000) {
            break;
        }
    }
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "GenerateOnePathLine end" << std::endl;
#endif

    return true;
}

/**
 * @brief get a link's link infos
 *
 * @return true
 * @return false
 */
bool ExtractRefLine::GetLinkInfos(uint32_t want_link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_->find(want_link_id) != link_id_index_lane_info_map_->end()) {
        int link_index = link_id_index_lane_info_map_->at(want_link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

bool ExtractRefLine::GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
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

bool ExtractRefLine::GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
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

bool ExtractRefLine::GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
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

bool ExtractRefLine::GetLine(const uint32_t line_index,
                             std::vector<message::map_map::s_GeometryPoint_t>& geometry_points, uint8_t& mrk_type,
                             uint8_t& mrk_color) {
    geometry_points.clear();
    for (int i = 0; i < map_static_info_->LinearObjects.LinearObjects.size(); i++) {
        if (map_static_info_->LinearObjects.LinearObjects[i].IDLinearObject.IDLinearObject == line_index) {
            geometry_points = map_static_info_->LinearObjects.LinearObjects[i].GeometryPoints.GeometryPoints;
            mrk_type = map_static_info_->LinearObjects.LinearObjects[i].LinearObjectMarking.data;
            mrk_color = map_static_info_->LinearObjects.LinearObjects[i].LinearObjectColour.data;
            return true;
        }
    }
    return false;
}

bool ExtractRefLine::FixPathDensity(const EFMRefLinePoints& raw_reference_line, double sample_distance,
                                    EFMRefLinePoints& fixed_reference_line) {
    if (raw_reference_line.size() <= 1 || sample_distance < 0.1) {
        // std::cout << "raw point num to less" << std::endl;
        return false;
    }

    // 路径密度优化
    fixed_reference_line.clear();
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

bool ExtractRefLine::FixMarkrDensity(const EFMRefLineMarkr& raw_reference_line, double sample_distance,
                                     EFMRefLineMarkr& fixed_reference_line) {
    if (raw_reference_line.size() <= 1) {
        // std::cout << "raw point num to less" << std::endl;
        return false;
    }

    // 路径密度优化
    fixed_reference_line.clear();
    if (sample_distance < 0.1) {
        return false;
    }  // unit m
    //
    double accumu_s = 0;
    EFMMarkr point0 = raw_reference_line[0];
    fixed_reference_line.emplace_back(point0);
    //
    const double accumu_s_threshold = 5000;
    // std::cout << "FixPathDensity, raw_reference_line.size(): " << raw_reference_line.size() << std::endl;
    for (int i = 1; i < raw_reference_line.size() && accumu_s < accumu_s_threshold; i++) {
        const EFMMarkr& point1 = raw_reference_line[i];
        double distance = sqrt(pow(point1.x - point0.x, 2) + pow(point1.y - point0.y, 2));
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
        point0.type = point1.type;
        point0.color = point1.color;
    }

    return true;
}

bool ExtractRefLine::GeneratePathsAttributeOne(std::vector<uint8_t>& lanes, EFMRefLine& path, int lane_prior_index){

   if (lane_prior_index >= 0) {
        lanes = all_lanes_vec_vec_[lane_prior_index];
        path.bIsAvailable = 1;
        uint32_t link_id = link_id_vec_[0];
        uint8_t lane_num = lanes[0];
        uint8_t lane_type = 0;
        if (GetLaneType(link_id, lane_num, lane_type)) {
            switch (lane_type) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                    path.LaneType = lane_type;
                    break;
                default:
                    path.LaneType = 8;
                    break;
            }
        } else {
            path.LaneType = 8;
        }
    } else {
        path.bIsAvailable = 0;
    }

    return true;
}

bool ExtractRefLine::GeneratePathsAttribute(EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path,
                                            EFMRefLine& right_path, EFMRefLine& right_right_path) {
    std::vector<uint8_t> ego_lane{};
    std::vector<uint8_t> left_lane{};
    std::vector<uint8_t> left_left_lane{};
    std::vector<uint8_t> right_lane{};
    std::vector<uint8_t> right_right_lane{};

    //ego
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePathsAttributeOne(ego_lane, ego_path, ego_lane_prior_index_);
    //left
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePathsAttributeOne(left_lane, left_path, left_lane_prior_index_);
    //left_left
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << " left_left_lane_prior_index: " << left_left_lane_prior_index_ <<std::endl;
    GeneratePathsAttributeOne(left_left_lane, left_left_path, left_left_lane_prior_index_);
    //right
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePathsAttributeOne(right_lane, right_path, right_lane_prior_index_);
    //right_right
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;
    GeneratePathsAttributeOne(right_right_lane, right_right_path, right_right_lane_prior_index_);
    // std::cout << __FILE__ << "," << __LINE__ << " EFM is running: " <<std::endl;

// #undef ERL_COUT
#ifdef ERL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_lane.size(): " << ego_lane.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_lane.size(): " << left_lane.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_lane.size(): " << right_lane.size() << std::endl;
#endif
    // if (ego_lane.size() > right_lane.size()) {
    //     right_path.bIsAvailable = 0;
    // }


    // merge type judge
    {
        ego_path.merge_type = MergeType_NONE;
        right_path.merge_type = MergeType_NONE;
        left_path.merge_type = MergeType_NONE;
        left_left_path.merge_type = MergeType_NONE;
        right_right_path.merge_type = MergeType_NONE;
        ego_path.merge_dist = -255;
        right_path.merge_dist = -255;
        right_right_path.merge_dist = -255;
        left_path.merge_dist = -255;
        left_left_path.merge_dist = -255;
        for (int i = 0; i < ego_lane.size() && i < left_lane.size() && i < link_id_vec_.size(); i++) {
            if (ego_lane[i] == left_lane[i] && i >= 1) {
                if (i > 0 && ego_lane[i - 1] < left_lane[i - 1]) {
                    uint32_t first_link = link_id_vec_[i - 1];
                    uint32_t second_link = link_id_vec_[i];
                    uint8_t first_ego_lane_num = ego_lane[i - 1];
                    uint8_t second_ego_lane_num = ego_lane[i];
                    uint8_t first_neighbor_lane_num = left_lane[i - 1];
                    uint8_t second_neighbor_lane_num = left_lane[i];
                    uint8_t first_ego_lane_transit = 0;
                    uint8_t first_neighbor_lane_transit = 0;
                    message::map_map::s_LinkInfo_t link_infos1;
                    if (!GetLinkInfos(first_link, link_infos1)) {
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "can't find link id: " << first_link << std::endl;
                        break;
                    }
                    for (auto& lane_info : link_infos1.LaneInfos.LaneInfos) {
                        if (lane_info.LaneNum.LaneNum == first_ego_lane_num) {
                            first_ego_lane_transit = int(lane_info.Transit.data);
                        }
                        if (lane_info.LaneNum.LaneNum == first_neighbor_lane_num) {
                            first_neighbor_lane_transit = int(lane_info.Transit.data);
                        }
                    }
                    if (first_ego_lane_transit != first_neighbor_lane_transit) {
                        if (first_ego_lane_transit == 2 && first_neighbor_lane_transit == 1) {
                            ego_path.merge_type = MergeType_MERGE_TO_LEFT;
                            left_path.merge_type = MergeType_MERGE_FROM_RIGHT;
                            ego_path.merge_dist = (static_cast<double>(link_infos1.EndOffset.EndOffset) -
                                                   static_cast<double>(map_position_->PathOffset)) /
                                                  100.0;
                            left_path.merge_dist = (static_cast<double>(link_infos1.EndOffset.EndOffset) -
                                                    static_cast<double>(map_position_->PathOffset)) /
                                                   100.0;
                            break;
                        } else {
                            continue;
                        }
                    }
                } else {
                    continue;
                }

            } else {
                continue;
            }
        }

        double ego_merge_dist1 = ego_path.merge_dist;
        uint8_t ego_merge_type1 = ego_path.merge_type;
        // ZTEXT("EFM_INFO", "ego_path_merge_dist1: ", 80, 24, "ego_path_merge_dist: {}", ego_merge_dist1);
        // ZTEXT("EFM_INFO", "ego_path_merge_type1: ", 80, 27, "ego_path_merge_type: {}", ego_merge_type1);
        for (int i = 0; i < ego_lane.size() && i < right_lane.size() && i < link_id_vec_.size(); i++) {
            if (ego_lane[i] == right_lane[i] && i >= 1) {
                if (i > 0 && ego_lane[i - 1] > right_lane[i - 1]) {
                    uint32_t first_link = link_id_vec_[i - 1];
                    uint32_t second_link = link_id_vec_[i];
                    uint8_t first_ego_lane_num = ego_lane[i - 1];
                    uint8_t second_ego_lane_num = ego_lane[i];
                    uint8_t first_neighbor_lane_num = right_lane[i - 1];
                    uint8_t second_neighbor_lane_num = right_lane[i];
                    uint8_t first_ego_lane_transit = 0;
                    uint8_t first_neighbor_lane_transit = 0;
                    message::map_map::s_LinkInfo_t link_infos1;
                    if (!GetLinkInfos(first_link, link_infos1)) {
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "can't find link id: " << first_link << std::endl;
                        break;
                    }
                    for (auto& lane_info : link_infos1.LaneInfos.LaneInfos) {
                        if (lane_info.LaneNum.LaneNum == first_ego_lane_num) {
                            first_ego_lane_transit = int(lane_info.Transit.data);
                        }
                        if (lane_info.LaneNum.LaneNum == first_neighbor_lane_num) {
                            first_neighbor_lane_transit = int(lane_info.Transit.data);
                        }
                    }
                    if (first_ego_lane_transit != first_neighbor_lane_transit) {
                        if (first_ego_lane_transit == 2 && first_neighbor_lane_transit == 1) {
                            ego_path.merge_type = MergeType_MERGE_TO_RIGHT;
                            right_path.merge_type = MergeType_MERGE_FROM_LEFT;
                            ego_path.merge_dist = (static_cast<double>(link_infos1.EndOffset.EndOffset) -
                                                   static_cast<double>(map_position_->PathOffset)) /
                                                  100.0;
                            right_path.merge_dist = (static_cast<double>(link_infos1.EndOffset.EndOffset) -
                                                     static_cast<double>(map_position_->PathOffset)) /
                                                    100.0;
                            break;
                        } else {
                            continue;
                        }
                    }
                } else {
                    continue;
                }

            } else {
                continue;
            }
        }
    }

    if (left_path.merge_type == 2 && left_path.merge_dist <= 1000) {
        left_path.bIsAvailable = 0;
    }
    if (right_path.merge_type == 1 && right_path.merge_dist <= 1000) {
        right_path.bIsAvailable = 0;
    }

    return true;
}

bool ExtractRefLine::GenerateOnePathAttribute(const std::vector<uint8_t>& candidate_lane,
                                              const std::vector<uint32_t>& link_id_vec, EFMRefLine& path) {
    // init
    // path.bIsAvailable = 1;

    return true;
}

bool ExtractRefLine::GetLaneType(uint32_t link_id, uint8_t lane_num, uint8_t& lane_type) {
    lane_type = 0;
    message::map_map::s_LinkInfo_t link_infos;
    if (!GetLinkInfos(link_id, link_infos)) {
#ifdef EXTRACTREFLINE_COUT
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "can't find link id: " << link_id << std::endl;
#endif
        return false;
    }
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            lane_type = lane.LaneType.data;
            return true;
        }
    }
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "can't find lane num: " << lane_num << " , link id: " << link_id << std::endl;
#endif
    return false;
}

void ExtractRefLine::CalPriorIndex(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                                   const EFMRefLine& ego_path) {
    // new code return
    int32_t ego_lane_idx = candidate_lanes_model->ego_lane_prior_index();
    int32_t left_lane_idx = candidate_lanes_model->left_lane_prior_index();
    int32_t left_left_lane_idx = candidate_lanes_model->left_left_lane_prior_index();
    int32_t right_lane_idx = candidate_lanes_model->right_lane_prior_index();
    int32_t right_right_lane_idx = candidate_lanes_model->right_right_lane_prior_index();
    int32_t ref_lane_idx = candidate_lanes_model->get_ref_lane_prior_index();
    if (ref_lane_idx == left_lane_idx && ref_lane_idx >= 0) {
        prior_path_index_ = 1;
    } else if (ref_lane_idx == left_left_lane_idx && ref_lane_idx >= 0) {
        prior_path_index_ = 3;
    } else if (ref_lane_idx == right_lane_idx && ref_lane_idx >= 0) {
        prior_path_index_ = 2;
    } else if (ref_lane_idx == right_right_lane_idx && ref_lane_idx >= 0) {
        prior_path_index_ = 4;
    } else {
        prior_path_index_ = 0;
    }
    // return ;

    int prior_index = prior_path_index_;  // 0-ego;1-left;2-right
    double& rest_dist = rest_dist_;
    auto& prior_path_change_type = prior_path_change_type_;
    rest_dist = 0;
    // prior_index = 0;
    prior_path_change_type = LaneChangeType_e::NONE;
    size_t ego_length = candidate_lanes_model->ego_lane_prior_length();
    size_t left_length = candidate_lanes_model->left_lane_prior_length();
    size_t right_length = candidate_lanes_model->right_lane_prior_length();
    uint8_t ego_lane_num = map_position_->LaneId;
    std::vector<std::vector<uint8_t>> all_lanes_vec_vec = candidate_lanes_model->all_lanes_vec_vec();

    // #undef ERL_COUT
    // #ifdef ERL_COUT
    //     std::cout << __FILE__ << "," << __LINE__ << ","
    //               << "ego_length: " << ego_length << std::endl;
    //     std::cout << __FILE__ << "," << __LINE__ << ","
    //               << "left_length: " << left_length << std::endl;
    //     std::cout << __FILE__ << "," << __LINE__ << ","
    //               << "right_length: " << right_length << std::endl;
    // #endif
    //     double entry_dist = scenario_judge_model->entry_start_dist();

    //     double exit_dist = scenario_judge_model->exit_start_dist();
    //     double p_EntryDistChangePrior = 300;
    //     if (entry_dist > 0 && entry_dist < p_EntryDistChangePrior && exit_dist > 0 && exit_dist < 2000) {
    //         if (entry_dist < exit_dist) {
    //             if (!left_path().linePoints.empty()) {
    //                 if (ego_length > left_length) {
    //                     prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                     std::cout << __FILE__ << "," << __LINE__ << ","
    //                               << "path_points is ego: " << std::endl;
    // #endif
    //                 } else {
    //                     prior_index = 1;
    // #ifdef EXTRACTREFLINE_COUT
    //                     std::cout << __FILE__ << "," << __LINE__ << ","
    //                               << "path_points is right: " << std::endl;
    // #endif
    //                 }
    //             } else {
    //                 prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is ego: " << std::endl;
    // #endif
    //             }
    //         } else {
    //             if (!right_path().linePoints.empty()) {
    //                 prior_index = 2;
    //                 if (ego_length >= right_length) {
    //                     prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                     std::cout << __FILE__ << "," << __LINE__ << ","
    //                               << "path_points is ego: " << std::endl;
    // #endif
    //                 } else {
    // #ifdef EXTRACTREFLINE_COUT
    //                     std::cout << __FILE__ << "," << __LINE__ << ","
    //                               << "path_points is right: " << std::endl;
    // #endif
    //                 }

    //             } else {
    //                 prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is ego: " << std::endl;
    // #endif
    //             }
    //         }
    //     } else if (entry_dist > 0 && entry_dist < p_EntryDistChangePrior) {
    //         if (!left_path().linePoints.empty()) {
    //             if (ego_length > left_length) {
    //                 prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is ego: " << std::endl;
    // #endif
    //             } else {
    //                 prior_index = 1;
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is left: " << std::endl;
    // #endif
    //             }

    //         } else {
    //             prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //             std::cout << __FILE__ << "," << __LINE__ << ","
    //                       << "path_points is ego: " << std::endl;
    // #endif
    //         }
    //     } else if (exit_dist > 0 && exit_dist < 2000) {
    //         if (!right_path().linePoints.empty()) {
    //             prior_index = 2;
    //             if (ego_length >= right_length) {
    //                 prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is ego: " << std::endl;
    // #endif
    //             } else {
    // #ifdef EXTRACTREFLINE_COUT
    //                 std::cout << __FILE__ << "," << __LINE__ << ","
    //                           << "path_points is right: " << std::endl;
    // #endif
    //             }

    //         } else {
    //             prior_index = 0;
    // #ifdef EXTRACTREFLINE_COUT
    //             std::cout << __FILE__ << "," << __LINE__ << ","
    //                       << "path_points is ego: " << std::endl;
    // #endif
    //         }
    //     } else {
    //         if (ego_length >= left_length && ego_length >= right_length) {
    // #ifdef EXTRACTREFLINE_COUT
    //             std::cout << __FILE__ << "," << __LINE__ << ","
    //                       << "path_points is ego: " << ego_length << std::endl;
    // #endif
    //             prior_index = 0;
    //         } else if ((left_length >= ego_length && left_length >= right_length)) {
    // #ifdef EXTRACTREFLINE_COUT
    //             std::cout << __FILE__ << "," << __LINE__ << ","
    //                       << "path_points is left: " << left_length << std::endl;
    // #endif
    //             prior_index = 1;
    //         } else if (right_length >= ego_length && right_length >= left_length) {
    // #ifdef EXTRACTREFLINE_COUT
    //             std::cout << __FILE__ << "," << __LINE__ << ","
    //                       << "path_points is right: " << right_length << std::endl;
    // #endif
    //             prior_index = 2;
    //         }
    //     }
    //     if (prior_index != 0) {
    //         prior_path_change_type = LaneChangeType_e::EXIT;
    //     }
    rest_dist = candidate_lanes_model->ego_lane_prior_dist();

    // merge info deal
    if (ego_path.merge_type == MergeType_MERGE_TO_LEFT && !(ego_length > left_length) &&
        (ego_path.merge_dist / 100.0) < 800) {
#ifdef EXTRACTREFLINE_COUT
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "path_points is left: " << left_length << std::endl;
#endif
        prior_index = 1;
        rest_dist = ego_path.merge_dist;
        prior_path_change_type = LaneChangeType_e::MERGE;
    }
    if (ego_path.merge_type == MergeType_MERGE_TO_RIGHT && !(ego_length > right_length) &&
        (ego_path.merge_dist / 100.0) < 800) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "path_points is right: " << right_length << std::endl;
        prior_index = 2;
        rest_dist = ego_path.merge_dist;
        prior_path_change_type = LaneChangeType_e::MERGE;
    }

    // if there's a lane size >ego,then get the nearest and longest first lane_num,
    // deal not on lane that can go to destination
    int max_length = 0;
    int ego_lane_length = candidate_lanes_model->ego_lane_prior_length();
    int max_length_index = -1;
    uint8_t max_length_lane_num = 0;
    for (int i = 0; i < all_lanes_vec_vec.size(); i++) {
        if (all_lanes_vec_vec[i].size() > max_length) {
            max_length = all_lanes_vec_vec[i].size();
            max_length_index = i;
        }
    }
    if (ego_lane_length > 0 && ego_lane_length < max_length) {
        if (max_length_index >= 0) {
            std::vector<uint8_t> lane = all_lanes_vec_vec[max_length_index];
            if (lane.size() > 0) {
                max_length_lane_num = lane[0];
            }
        }
        if (max_length_lane_num > ego_lane_num) {
            // left
            prior_index = 1;
            rest_dist = candidate_lanes_model->ego_lane_prior_dist();
            prior_path_change_type = LaneChangeType_e::LANE_END;
        } else if (max_length_lane_num < ego_lane_num) {
            prior_index = 2;
            rest_dist = candidate_lanes_model->ego_lane_prior_dist();
            prior_path_change_type = LaneChangeType_e::LANE_END;
        } else {
            // keep result
        }
    }

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "prior_index: " << prior_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "prior_path_change_type: " << prior_path_change_type << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "rest_dist: " << rest_dist << std::endl;
#endif
}

void ExtractRefLine::CutLineToFixRange(EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& right_path) {
    int ego_c_start_index = -1;
    int ego_c_end_index = -1;
    int ego_l_start_index = -1;
    int ego_l_end_index = -1;
    int ego_r_start_index = -1;
    int ego_r_end_index = -1;

    int left_c_start_index = -1;
    int left_c_end_index = -1;
    int left_l_start_index = -1;
    int left_l_end_index = -1;
    int left_r_start_index = -1;
    int left_r_end_index = -1;

    int right_c_start_index = -1;
    int right_c_end_index = -1;
    int right_l_start_index = -1;
    int right_l_end_index = -1;
    int right_r_start_index = -1;
    int right_r_end_index = -1;

    double p_start_s = -50;
    double p_end_s = 230;
#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_path.linePoints.size(): " << ego_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_path.leftMarkr.size(): " << ego_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_path.rightMarkr.size(): " << ego_path.rightMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_path.linePoints.size(): " << left_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_path.leftMarkr.size(): " << left_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_path.rightMarkr.size(): " << left_path.rightMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_path.linePoints.size(): " << right_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_path.leftMarkr.size(): " << right_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_path.rightMarkr.size(): " << right_path.rightMarkr.size() << std::endl;
#endif

    {  // ego
        if (GetStartEndIndexInRange(p_start_s, p_end_s, ego_path.linePoints, ego_c_start_index, ego_c_end_index)) {
            EFMRefLinePoints::const_iterator start_iter = ego_path.linePoints.begin() + ego_c_start_index;
            EFMRefLinePoints::const_iterator end_iter = ego_path.linePoints.begin() + ego_c_end_index;
            EFMRefLinePoints ego_center_line(start_iter, end_iter);
            ego_path.linePoints.clear();
            ego_path.linePoints = ego_center_line;
        } else {
            ego_path.linePoints.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, ego_path.leftMarkr, ego_l_start_index, ego_l_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = ego_path.leftMarkr.begin() + ego_l_start_index;
            EFMRefLineMarkr::const_iterator end_iter = ego_path.leftMarkr.begin() + ego_l_end_index;
            EFMRefLineMarkr ego_left_mkr(start_iter, end_iter);
            ego_path.leftMarkr.clear();
            ego_path.leftMarkr = ego_left_mkr;
        } else {
            ego_path.leftMarkr.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, ego_path.rightMarkr, ego_r_start_index, ego_r_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = ego_path.rightMarkr.begin() + ego_r_start_index;
            EFMRefLineMarkr::const_iterator end_iter = ego_path.rightMarkr.begin() + ego_r_end_index;
            EFMRefLineMarkr ego_right_mkr(start_iter, end_iter);
            ego_path.rightMarkr.clear();
            ego_path.rightMarkr = ego_right_mkr;
        } else {
            ego_path.rightMarkr.clear();
        }
    }

    {  // left lane
        if (GetStartEndIndexInRange(p_start_s, p_end_s, left_path.linePoints, left_c_start_index, left_c_end_index)) {
            EFMRefLinePoints::const_iterator start_iter = left_path.linePoints.begin() + left_c_start_index;
            EFMRefLinePoints::const_iterator end_iter = left_path.linePoints.begin() + left_c_end_index;
            EFMRefLinePoints left_center_line(start_iter, end_iter);
            left_path.linePoints.clear();
            left_path.linePoints = left_center_line;
        } else {
            left_path.linePoints.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, left_path.leftMarkr, left_l_start_index, left_l_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = left_path.leftMarkr.begin() + left_l_start_index;
            EFMRefLineMarkr::const_iterator end_iter = left_path.leftMarkr.begin() + left_l_end_index;
            EFMRefLineMarkr left_mkr(start_iter, end_iter);
            left_path.leftMarkr.clear();
            left_path.leftMarkr = left_mkr;
        } else {
            left_path.leftMarkr.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, left_path.rightMarkr, left_r_start_index, left_r_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = left_path.rightMarkr.begin() + left_r_start_index;
            EFMRefLineMarkr::const_iterator end_iter = left_path.rightMarkr.begin() + left_r_end_index;
            EFMRefLineMarkr right_mkr(start_iter, end_iter);
            left_path.rightMarkr.clear();
            left_path.rightMarkr = right_mkr;
        } else {
            left_path.rightMarkr.clear();
        }
    }

    {  // right lane
        if (GetStartEndIndexInRange(p_start_s, p_end_s, right_path.linePoints, right_c_start_index,
                                    right_c_end_index)) {
            EFMRefLinePoints::const_iterator start_iter = right_path.linePoints.begin() + right_c_start_index;
            EFMRefLinePoints::const_iterator end_iter = right_path.linePoints.begin() + right_c_end_index;
            EFMRefLinePoints center_line(start_iter, end_iter);
            right_path.linePoints.clear();
            right_path.linePoints = center_line;
        } else {
            right_path.linePoints.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, right_path.leftMarkr, right_l_start_index, right_l_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = right_path.leftMarkr.begin() + right_l_start_index;
            EFMRefLineMarkr::const_iterator end_iter = right_path.leftMarkr.begin() + right_l_end_index;
            EFMRefLineMarkr left_mkr(start_iter, end_iter);
            right_path.leftMarkr.clear();
            right_path.leftMarkr = left_mkr;
        } else {
            right_path.leftMarkr.clear();
        }
        if (GetStartEndIndexInRange(p_start_s, p_end_s, right_path.rightMarkr, right_r_start_index,
                                    right_r_end_index)) {
            EFMRefLineMarkr::const_iterator start_iter = right_path.rightMarkr.begin() + right_r_start_index;
            EFMRefLineMarkr::const_iterator end_iter = right_path.rightMarkr.begin() + right_r_end_index;
            EFMRefLineMarkr right_mkr(start_iter, end_iter);
            right_path.rightMarkr.clear();
            right_path.rightMarkr = right_mkr;
        } else {
            right_path.rightMarkr.clear();
        }
    }

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_c_start_index: " << ego_c_start_index << " ego_c_end_index: " << ego_c_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_l_start_index: " << ego_l_start_index << " ego_l_end_index: " << ego_l_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "ego_r_start_index: " << ego_r_start_index << " ego_r_end_index: " << ego_r_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_c_start_index: " << left_c_start_index << " left_c_end_index: " << left_c_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_l_start_index: " << left_l_start_index << " left_l_end_index: " << left_l_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "left_r_start_index: " << left_r_start_index << " left_r_end_index: " << left_r_end_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_c_start_index: " << right_c_start_index << " right_c_end_index: " << right_c_end_index
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_l_start_index: " << right_l_start_index << " right_l_end_index: " << right_l_end_index
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_r_start_index: " << right_r_start_index << " right_r_end_index: " << right_r_end_index
              << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: ego_path.linePoints.size(): " << ego_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: ego_path.leftMarkr.size(): " << ego_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: ego_path.rightMarkr.size(): " << ego_path.rightMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: left_path.linePoints.size(): " << left_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: left_path.leftMarkr.size(): " << left_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: left_path.rightMarkr.size(): " << left_path.rightMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: right_path.linePoints.size(): " << right_path.linePoints.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: right_path.leftMarkr.size(): " << right_path.leftMarkr.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "after: right_path.rightMarkr.size(): " << right_path.rightMarkr.size() << std::endl;

#endif
}

bool ExtractRefLine::GetStartEndIndexInRange(double start_s, double end_s, const EFMRefLinePoints& linePoints,
                                             int& start_index, int& end_index) {
    start_index = -1;
    end_index = -1;
    if (start_s > end_s) {
        return false;
    }
    int ego_index = -1;
    for (int i = 1; i < linePoints.size(); i++) {
        if (linePoints[i - 1].x < 0 && linePoints[i].x >= 0) {
            ego_index = i;
            break;
        }
    }
    if (ego_index < 0) {
        return false;
    }

    int back_point = static_cast<int>(std::abs(start_s) / 2.5);
    int front_point = static_cast<int>(end_s / 2.5);

    start_index = std::max(ego_index - back_point, 0);
    end_index = std::min(ego_index + front_point, static_cast<int>(linePoints.size() - 1));
    // for (int i = 0; i < linePoints.size(); i++) {
    //     if (start_index < 0 && linePoints[i].x >= start_s) {
    //         start_index = i;
    //     } else if (end_index < 0 && linePoints[i].x >= end_s) {
    //         end_index = i;
    //     }

    //     if (start_index >= 0 && end_index >= 0) {
    //         break;
    //     }
    //     if (i == (linePoints.size() - 1)) {
    //         if (start_index < 0) {
    //             // all < -50
    //             break;
    //         } else if (end_index < 0) {
    //             // end <150
    //             end_index = linePoints.size() - 1;
    //         }
    //     }
    // }
    if (start_index < 0 && end_index < 0) {
        return false;
    } else if (start_index > end_index) {
        return false;
    } else if (start_index >= linePoints.size() || end_index >= linePoints.size()) {
        return false;
    }
    return true;
}

bool ExtractRefLine::GetStartEndIndexInRange(double start_s, double end_s, const EFMRefLineMarkr& lane_markr,
                                             int& start_index, int& end_index) {
    start_index = -1;
    end_index = -1;
    if (start_s > end_s) {
        return false;
    }
    int ego_index = -1;
    for (int i = 1; i < lane_markr.size(); i++) {
        if (lane_markr[i - 1].x < 0 && lane_markr[i].x >= 0) {
            ego_index = i;
            break;
        }
    }
    if (ego_index < 0) {
        return false;
    }

    int back_point = static_cast<int>(std::abs(start_s) / 2.5);
    int front_point = static_cast<int>(end_s / 2.5);

    start_index = std::max(ego_index - back_point, 0);
    end_index = std::min(ego_index + front_point, static_cast<int>(lane_markr.size() - 1));
    // for (int i = 0; i < lane_markr.size(); i++) {
    //     if (start_index < 0 && lane_markr[i].x >= start_s) {
    //         start_index = i;
    //     } else if (end_index < 0 && lane_markr[i].x >= end_s) {
    //         end_index = i;
    //     }

    //     if (start_index >= 0 && end_index >= 0) {
    //         break;
    //     }
    //     if (i == (lane_markr.size() - 1)) {
    //         if (start_index < 0) {
    //             // all < -50
    //             break;
    //         } else if (end_index < 0) {
    //             // end <150
    //             end_index = lane_markr.size() - 1;
    //         }
    //     }
    // }
    if (start_index < 0 && end_index < 0) {
        return false;
    } else if (start_index > end_index) {
        return false;
    }
    return true;
}

// bool ExtractRefLine::RerangeLaneMarkerSection(EFMRefLine& path) {
// // generate serval contimuous line sections, input include virtual line
// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "path.leftMarkrSection.size()" << path.leftMarkrSection.size() << std::endl;
//     for (int i = 0; i < path.leftMarkrSection.size(); i++) {
//         std::cout << " [,type: " << (int)path.leftMarkrSection[i].front().type
//                   << " ,point size: " << path.leftMarkrSection[i].size() << "]";
//     }
//     std::cout << std::endl;
// #endif
//     if (path.leftMarkrSection.size() <= 1) {
//         // do nothing
//     } else {
//         EFMRefLineMarkrSection left_lane_mkr_section_raw = path.leftMarkrSection;
//         path.leftMarkrSection.clear();
//         for (int i = 0; i < left_lane_mkr_section_raw.size() - 1; i++) {
//             std::vector<EFMMarkr> back_section = left_lane_mkr_section_raw[i];
//             std::vector<EFMMarkr> front_section = left_lane_mkr_section_raw[i + 1];
//             if (i == 0) {
//                 path.leftMarkrSection.push_back(back_section);
//             }
//             double dist = sqrt(pow(back_section.back().x - front_section.front().x, 2) +
//                                pow(back_section.back().y - front_section.front().y, 2));
//             // two section is continous, connect into one section
//             if (dist < 0.5) {
//                 path.leftMarkrSection.back().insert(path.leftMarkrSection.back().end(), front_section.begin(),
//                                                     front_section.end());
//             } else {
//                 path.leftMarkrSection.push_back(front_section);
//             }
//         }
//     }
// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "after path.leftMarkrSection.size()" << path.leftMarkrSection.size() << std::endl;
//     for (int i = 0; i < path.leftMarkrSection.size(); i++) {
//         std::cout << " [,type: " << (int)path.leftMarkrSection[i].front().type
//                   << " ,point size: " << path.leftMarkrSection[i].size() << "]";
//     }
//     std::cout << std::endl;
// #endif

// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "path.rightMarkrSection.size()" << path.rightMarkrSection.size() << std::endl;
//     for (int i = 0; i < path.rightMarkrSection.size(); i++) {
//         std::cout << " [,type: " << (int)path.rightMarkrSection[i].front().type
//                   << " ,point size: " << path.rightMarkrSection[i].size() << "]";
//     }
//     std::cout << std::endl;
// #endif
//     if (path.rightMarkrSection.size() <= 1) {
//         // do nothing
//     } else {
//         EFMRefLineMarkrSection right_lane_mkr_section_raw = path.rightMarkrSection;
//         path.rightMarkrSection.clear();
//         for (int i = 0; i < right_lane_mkr_section_raw.size() - 1; i++) {
//             std::vector<EFMMarkr> back_section = right_lane_mkr_section_raw[i];
//             std::vector<EFMMarkr> front_section = right_lane_mkr_section_raw[i + 1];
//             if (i == 0) {
//                 path.rightMarkrSection.push_back(back_section);
//             }
//             double dist = sqrt(pow(back_section.back().x - front_section.front().x, 2) +
//                                pow(back_section.back().y - front_section.front().y, 2));
//             // two section is continous, connect into one section
//             if (dist < 0.5) {
//                 path.rightMarkrSection.back().insert(path.rightMarkrSection.back().end(), front_section.begin(),
//                                                      front_section.end());
//             } else {
//                 path.rightMarkrSection.push_back(front_section);
//             }
//         }
//     }
// #ifdef EXTRACTREFLINE_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << "after path.rightMarkrSection.size()" << path.rightMarkrSection.size() << std::endl;
//     for (int i = 0; i < path.rightMarkrSection.size(); i++) {
//         std::cout << " [,type: " << (int)path.rightMarkrSection[i].front().type
//                   << " ,point size: " << path.rightMarkrSection[i].size() << "]";
//     }
//     std::cout << std::endl;
// #endif

//     return true;
// }

bool ExtractRefLine::MakePath(double ego_utm_x, double ego_utm_y, EFMRefLine& path) {
    // 1.utm to body coordination, 2.fix dentisy
    // path.leftMarkr.clear();
    // path.rightMarkr.clear();
#ifdef EXTRACTREFLINE_COUT
            {
                std::stringstream ss2;
                for (auto point : path.linePoints) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " path.linePoints: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.linePoints) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "path.linePoints: point.y: " << ss.str() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.linePoints.size(): " << path.linePoints.size() << std::endl;
                std::stringstream ss3;
                for (auto point : path.linePointsOffset) {
                    ss3 << " " << point;
                }
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.linePointsOffset:  " << ss3.str() << std::endl;    
                 std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.linePointsOffset.size(): " << path.linePointsOffset.size() << std::endl;                      
            }
            {
                std::stringstream ss2;
                for (auto point : path.leftMarkr) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " path.leftMarkr: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.leftMarkr) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "path.leftMarkr: point.y: " << ss.str() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.leftMarkr.size(): " << path.leftMarkr.size() << std::endl;
                std::stringstream ss3;
                for (auto point : path.leftMarkrOffset) {
                    ss3 << " " << point;
                }
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.leftMarkrsOffset:  " << ss3.str() << std::endl;    
                 std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.leftMarkrOffset.size(): " << path.leftMarkrOffset.size() << std::endl;  
            }   
         {
                std::stringstream ss2;
                for (auto point : path.rightMarkr) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " path.rightMarkr: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.rightMarkr) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "path.rightMarkr: point.y: " << ss.str() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.rightMarkr.size(): " << path.rightMarkr.size() << std::endl;
                 std::stringstream ss3;
                for (auto point : path.rightMarkrOffset) {
                    ss3 << " " << point;
                }
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.rightMarkrOffset:  " << ss3.str() << std::endl;    
                 std::cout << __FILE__ << "," << __LINE__ << ","
                          << "raw path.rightMarkrOffset.size(): " << path.rightMarkrOffset.size() << std::endl;                            
            }                      
#endif    
    if(path.linePoints.size() != path.linePointsOffset.size() || 
       path.leftMarkr.size() != path.leftMarkrOffset.size() || 
       path.rightMarkr.size() != path.rightMarkrOffset.size()){
        return false;
    }
    path.linePointsSection[0].clear();
    path.linePointsSection[1].clear();
    path.leftMarkrSection[0].clear();
    path.leftMarkrSection[1].clear();
    path.rightMarkrSection[0].clear();
    path.rightMarkrSection[1].clear();
    std::vector<uint32_t> center_offset{};
    std::vector<uint32_t> left_offset{};
    std::vector<uint32_t> right_offset{};
    {  // center line
        if (!path.linePoints.empty()) {
            EFMRefLinePoints reference_line_points_bodyc{};
            for (int k = 0; k < path.linePoints.size(); k++) {
                // double x, y;
                EFMPoint point = path.linePoints[k];
                uint32_t offset = path.linePointsOffset[k];
                // auto instance2 = CommonTool::CoordinateTool::GetInstance();
                // if (instance2 != nullptr) {
                //     instance2->UTM2EgoVehicle(point.x, point.y, ego_utm_x, ego_utm_y, map_position_->Heading.Heading, x,
                //                               y);
                    if (reference_line_points_bodyc.size() == 0) {
                        reference_line_points_bodyc.push_back({point.x, point.y});
                        center_offset.push_back(offset);
                    } else {
                        double distance = sqrt(pow(point.x - reference_line_points_bodyc.back().x, 2) +
                                               pow(point.y - reference_line_points_bodyc.back().y, 2));
                        if (distance > 0.1) {
                            reference_line_points_bodyc.push_back({point.x, point.y});
                            center_offset.push_back(offset);
                        }
                    }
                // } else {
                //     LOGI("CommonTool::CoordinateTool::GetInstance(): nullptr");
                // }
            }
            path.linePoints.clear();
            path.linePoints = reference_line_points_bodyc;
            path.linePointsOffset.clear();
            path.linePointsOffset = center_offset;
#ifdef EXTRACTREFLINE_COUT
            {
                std::stringstream ss2;
                for (auto point : reference_line_points_bodyc) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " center line befor fix: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : reference_line_points_bodyc) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "center line befor fix: point.y: " << ss.str() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.linePoints.size(): " << path.linePoints.size() << std::endl;
            }
#endif
            // change point
         //   if (1 == p_efm_use_pointforsplit) {
         //       if (path.is_ref_line && path.split_in_point_index > 0) {
                    // std::cout << __FILE__ << "," << __LINE__ << "," << "path.split_in_point:" <<
                    // path.split_in_point_index<< std::endl;
             //       AlterPointYForSplit(reference_line_points_bodyc, path.split_in_point_index,
             //                           path.split_position_index, path.next_in_point_index, path.next_position_index,
             //                           path.leftLineMkrType, path.rightLineMkrType, path.sum_curvature_vec,
             //                           path.laneMarkSectionLength, path.split_from, path.back_lane_split_point_index,
             //                          path.back_lane_is_split);
             //  }
            //}
            EFMPoint ego_point(0.0, 0.0);
            if (SaperateLaneCenterLineIntoTwoPart(reference_line_points_bodyc, ego_point,path.linePointsOffset,map_position_->PathOffset ,path.linePointsSection) ==
                true) {
#ifdef EXTRACTREFLINE_COUT
                {
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << "path.linePointsSection[0].size(): " << path.linePointsSection[0].size() << std::endl;
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << "path.linePointsSection[1].size(): " << path.linePointsSection[1].size() << std::endl;
                    std::stringstream ss2;
                    for (auto point : path.linePointsSection[0]) {
                        ss2 << " " << point.x;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << " center line back part: point.x: " << ss2.str() << std::endl;
                    std::stringstream ss;
                    for (auto point : path.linePointsSection[0]) {
                        ss << " " << point.y;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "center line back part: point.y: " << ss.str() << std::endl;

                    std::stringstream ss3;
                    for (auto point : path.linePointsSection[1]) {
                        ss3 << " " << point.x;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << " center line front part: point.x: " << ss3.str() << std::endl;
                    std::stringstream ss4;
                    for (auto point : path.linePointsSection[1]) {
                        ss4 << " " << point.y;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "center line front part: point.y: " << ss4.str() << std::endl;
                }
#endif
                EFMRefLinePoints fixed_reference_line_back{};
                EFMRefLinePoints fixed_reference_line_front{};
                double back_sample_distance = p_back_center_line_sample_dist;
                double front_sample_distance = p_front_center_line_sample_dist;
                std::reverse(path.linePointsSection[0].begin(),
                             path.linePointsSection[0].end());  // change to first point is proj point
                if (FixPathDensity(path.linePointsSection[0], back_sample_distance, fixed_reference_line_back)) {
                    path.linePointsSection[0].clear();
                    path.linePointsSection[0] = fixed_reference_line_back;
                } else {
                    path.linePointsSection[0].clear();
#ifdef EXTRACTREFLINE_COUT
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << " FixPathDensity back section is falure!!!!!" << std::endl;
#endif
                }

                if (FixPathDensity(path.linePointsSection[1], front_sample_distance, fixed_reference_line_front)) {
                    path.linePointsSection[1].clear();
                    path.linePointsSection[1] = fixed_reference_line_front;
                } else {
                    path.linePointsSection[1].clear();
#ifdef EXTRACTREFLINE_COUT
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << " FixPathDensity front section is falure!!!!!" << std::endl;
#endif
                }

#ifdef EXTRACTREFLINE_COUT
                {
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << "after fix#### path.linePointsSection[0].size(): " << path.linePointsSection[0].size()
                              << std::endl;
                    std::cout << __FILE__ << "," << __LINE__ << ","
                              << "after fix#### path.linePointsSection[1].size(): " << path.linePointsSection[1].size()
                              << std::endl;
                    std::stringstream ss2;
                    for (auto point : path.linePointsSection[0]) {
                        ss2 << " " << point.x;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "after fix####  center line back part: point.x: " << ss2.str() << std::endl;
                    std::stringstream ss;
                    for (auto point : path.linePointsSection[0]) {
                        ss << " " << point.y;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "after fix#### center line back part: point.y: " << ss.str() << std::endl;

                    std::stringstream ss3;
                    for (auto point : path.linePointsSection[1]) {
                        ss3 << " " << point.x;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "after fix####  center line front part: point.x: " << ss3.str() << std::endl;
                    std::stringstream ss4;
                    for (auto point : path.linePointsSection[1]) {
                        ss4 << " " << point.y;
                    }
                    std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                              << "after fix#### center line front part: point.y: " << ss4.str() << std::endl;
                }
#endif
            } else {
                path.linePointsSection[0].clear();
                path.linePointsSection[1].clear();
            }
        }
    }

    {  // left lane markr
        EFMRefLineMarkr reference_line_points_bodyc{};
        // EFMMarkr lane_mkr{};
        // for (int k = 0; k < lane_mkr_vec.size(); k++) {
        for (int k =0;k< path.leftMarkr.size();k++) {
            EFMMarkr mkr = path.leftMarkr[k];
            uint32_t offset = path.leftMarkrOffset[k];
            // lane_mkr.type = mkr.type;
            // lane_mkr.color = mkr.color;
            // auto instance2 = CommonTool::CoordinateTool::GetInstance();
            // if (instance2 != nullptr) {
            //     instance2->UTM2EgoVehicle(mkr.x, mkr.y, ego_utm_x, ego_utm_y, map_position_->Heading.Heading,
            //                               lane_mkr.x, lane_mkr.y);
                if (reference_line_points_bodyc.size() == 0) {
                    reference_line_points_bodyc.push_back(mkr);
                    left_offset.push_back(offset);
                } else {
                    double distance = sqrt(pow(mkr.x - reference_line_points_bodyc.back().x, 2) +
                                           pow(mkr.y - reference_line_points_bodyc.back().y, 2));
                    if (distance > 0.1) {
                        reference_line_points_bodyc.push_back(mkr);
                        left_offset.push_back(offset);
                    }
                }
            // } else {
            //     LOGI("CommonTool::CoordinateTool::GetInstance(): nullptr");
            // }
        }
        path.leftMarkr.clear();
        path.leftMarkr = reference_line_points_bodyc;
        path.leftMarkrOffset.clear();
        path.leftMarkrOffset = left_offset;
#ifdef EXTRACTREFLINE_COUT
        {
            std::stringstream ss2;
            for (auto point : reference_line_points_bodyc) {
                ss2 << " " << point.x;
            }
            std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                      << " left line befor fix: point.x: " << ss2.str() << std::endl;
            std::stringstream ss;
            for (auto point : reference_line_points_bodyc) {
                ss << " " << point.y;
            }
            std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                      << "left line befor fix: point.y: " << ss.str() << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << ","
                      << "path.leftMarkr.size(): " << path.leftMarkr.size() << std::endl;
        }
#endif

        EFMPoint ego_point(0.0, 0.0);
        if (SaperateLaneMarkerIntoTwoPart(reference_line_points_bodyc, ego_point,path.leftMarkrOffset ,map_position_->PathOffset,path.leftMarkrSection) == true) {
#ifdef EXTRACTREFLINE_COUT
            {
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.leftMarkrSection[0].size(): " << path.leftMarkrSection[0].size() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.leftMarkrSection[1].size(): " << path.leftMarkrSection[1].size() << std::endl;
                std::stringstream ss2;
                for (auto point : path.leftMarkrSection[0]) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " left line back part: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.leftMarkrSection[0]) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "left line back part: point.y: " << ss.str() << std::endl;

                std::stringstream ss3;
                for (auto point : path.leftMarkrSection[1]) {
                    ss3 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " left line front part: point.x: " << ss3.str() << std::endl;
                std::stringstream ss4;
                for (auto point : path.leftMarkrSection[1]) {
                    ss4 << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "left line front part: point.y: " << ss4.str() << std::endl;
            }
#endif
            EFMRefLineMarkr fixed_reference_line_back{};
            EFMRefLineMarkr fixed_reference_line_front{};
            double back_sample_distance = p_back_side_line_sample_dist;
            double front_sample_distance = p_front_side_line_sample_dist;
            std::reverse(path.leftMarkrSection[0].begin(),
                         path.leftMarkrSection[0].end());  // change to first point is proj point
            if (FixMarkrDensity(path.leftMarkrSection[0], back_sample_distance, fixed_reference_line_back)) {
                path.leftMarkrSection[0].clear();
                path.leftMarkrSection[0] = fixed_reference_line_back;
            } else {
                path.leftMarkrSection[0].clear();
#ifdef EXTRACTREFLINE_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << " FixPathDensity back section is falure!!!!!" << std::endl;
#endif
            }

            if (FixMarkrDensity(path.leftMarkrSection[1], front_sample_distance, fixed_reference_line_front)) {
                path.leftMarkrSection[1].clear();
                path.leftMarkrSection[1] = fixed_reference_line_front;
            } else {
                path.leftMarkrSection[1].clear();
#ifdef EXTRACTREFLINE_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << " FixPathDensity front section is falure!!!!!" << std::endl;
#endif
            }
        } else {
            path.leftMarkrSection[0].clear();
            path.leftMarkrSection[1].clear();
        }
    }
#ifdef EXTRACTREFLINE_COUT
    {
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "after fix#### path.leftMarkrSection[0].size(): " << path.leftMarkrSection[0].size() << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "after fix#### path.leftMarkrSection[1].size(): " << path.leftMarkrSection[1].size() << std::endl;
        std::stringstream ss2;
        for (auto point : path.leftMarkrSection[0]) {
            ss2 << " " << point.x;
        }
        std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                  << "after fix####  left line back part: point.x: " << ss2.str() << std::endl;
        std::stringstream ss;
        for (auto point : path.leftMarkrSection[0]) {
            ss << " " << point.y;
        }
        std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                  << "after fix#### left line back part: point.y: " << ss.str() << std::endl;

        std::stringstream ss3;
        for (auto point : path.leftMarkrSection[1]) {
            ss3 << " " << point.x;
        }
        std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                  << "after fix####  left line front part: point.x: " << ss3.str() << std::endl;
        std::stringstream ss4;
        for (auto point : path.leftMarkrSection[1]) {
            ss4 << " " << point.y;
        }
        std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                  << "after fix#### left line front part: point.y: " << ss4.str() << std::endl;
    }
#endif
    {  // right lane markr
        EFMRefLineMarkr reference_line_points_bodyc{};
        // EFMMarkr lane_mkr{};
        for (int k =0;k <path.rightMarkr.size();k++) {
            EFMMarkr mkr = path.rightMarkr[k];
            uint32_t offset = path.rightMarkrOffset[k];
            // lane_mkr.type = mkr.type;
            // lane_mkr.color = mkr.color;
            // auto instance2 = CommonTool::CoordinateTool::GetInstance();
            // if (instance2 != nullptr) {
            //     instance2->UTM2EgoVehicle(mkr.x, mkr.y, ego_utm_x, ego_utm_y, map_position_->Heading.Heading,
            //                               lane_mkr.x, lane_mkr.y);
                if (reference_line_points_bodyc.size() == 0) {
                    reference_line_points_bodyc.push_back(mkr);
                    right_offset.push_back(offset);
                } else {
                    double distance = sqrt(pow(mkr.x - reference_line_points_bodyc.back().x, 2) +
                                           pow(mkr.y - reference_line_points_bodyc.back().y, 2));
                    if (distance > 0.1) {
                        reference_line_points_bodyc.push_back(mkr);
                        right_offset.push_back(offset);
                    }
                }
            // } else {
            //     LOGI("CommonTool::CoordinateTool::GetInstance(): nullptr");
            // }
        }
        path.rightMarkr.clear();
        path.rightMarkr = reference_line_points_bodyc;
        path.rightMarkrOffset.clear();
        path.rightMarkrOffset = right_offset;
#ifdef EXTRACTREFLINE_COUT
        {
            std::stringstream ss2;
            for (auto point : reference_line_points_bodyc) {
                ss2 << " " << point.x;
            }
            std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                      << " right line befor fix: point.x: " << ss2.str() << std::endl;
            std::stringstream ss;
            for (auto point : reference_line_points_bodyc) {
                ss << " " << point.y;
            }
            std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                      << "right line befor fix: point.y: " << ss.str() << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << ","
                      << "path.rightMarkr.size(): " << path.rightMarkr.size() << std::endl;
        }
#endif
        EFMPoint ego_point(0.0, 0.0);
        if (SaperateLaneMarkerIntoTwoPart(reference_line_points_bodyc, ego_point,path.rightMarkrOffset, map_position_->PathOffset,path.rightMarkrSection) == true) {
#ifdef EXTRACTREFLINE_COUT
            {
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.rightMarkrSection[0].size(): " << path.rightMarkrSection[0].size() << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "path.rightMarkrSection[1].size(): " << path.rightMarkrSection[1].size() << std::endl;
                std::stringstream ss2;
                for (auto point : path.rightMarkrSection[0]) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " right line back part: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.rightMarkrSection[0]) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "right line back part: point.y: " << ss.str() << std::endl;

                std::stringstream ss3;
                for (auto point : path.rightMarkrSection[1]) {
                    ss3 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << " right line front part: point.x: " << ss3.str() << std::endl;
                std::stringstream ss4;
                for (auto point : path.rightMarkrSection[1]) {
                    ss4 << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "right line front part: point.y: " << ss4.str() << std::endl;
            }
#endif
            EFMRefLineMarkr fixed_reference_line_back{};
            EFMRefLineMarkr fixed_reference_line_front{};
            double back_sample_distance = p_back_side_line_sample_dist;
            double front_sample_distance = p_front_side_line_sample_dist;
            std::reverse(path.rightMarkrSection[0].begin(),
                         path.rightMarkrSection[0].end());  // change to first point is proj point
            if (FixMarkrDensity(path.rightMarkrSection[0], back_sample_distance, fixed_reference_line_back)) {
                path.rightMarkrSection[0].clear();
                path.rightMarkrSection[0] = fixed_reference_line_back;
            } else {
                path.rightMarkrSection[0].clear();
#ifdef EXTRACTREFLINE_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << " FixPathDensity back section is falure!!!!!" << std::endl;
#endif
            }

            if (FixMarkrDensity(path.rightMarkrSection[1], front_sample_distance, fixed_reference_line_front)) {
                path.rightMarkrSection[1].clear();
                path.rightMarkrSection[1] = fixed_reference_line_front;
            } else {
                path.rightMarkrSection[1].clear();
#ifdef EXTRACTREFLINE_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << " FixPathDensity front section is falure!!!!!" << std::endl;
#endif
            }
#ifdef EXTRACTREFLINE_COUT
            {
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "after fix#### path.rightMarkrSection[0].size(): " << path.rightMarkrSection[0].size()
                          << std::endl;
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "after fix#### path.rightMarkrSection[1].size(): " << path.rightMarkrSection[1].size()
                          << std::endl;
                std::stringstream ss2;
                for (auto point : path.rightMarkrSection[0]) {
                    ss2 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "after fix####  right line back part: point.x: " << ss2.str() << std::endl;
                std::stringstream ss;
                for (auto point : path.rightMarkrSection[0]) {
                    ss << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "after fix#### right line back part: point.y: " << ss.str() << std::endl;

                std::stringstream ss3;
                for (auto point : path.rightMarkrSection[1]) {
                    ss3 << " " << point.x;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "after fix####  right line front part: point.x: " << ss3.str() << std::endl;
                std::stringstream ss4;
                for (auto point : path.rightMarkrSection[1]) {
                    ss4 << " " << point.y;
                }
                std::cout << std::cout.precision(12) << __FILE__ << "," << __LINE__ << ","
                          << "after fix#### right line front part: point.y: " << ss4.str() << std::endl;
            }
#endif
        } else {
            path.rightMarkrSection[0].clear();
            path.rightMarkrSection[1].clear();
        }
    }

#ifdef EXTRACTREFLINE_COUT
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "after fix: path.rightMarkr.size(): " << path.rightMarkr.size() << std::endl;

    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "after fix: path().rightMarkr: type: " << std::endl;
    // for (auto type : path.rightMarkr) {
    //     std::cout << ", " << (int)type.type;
    // }
    // std::cout << std::endl;
#endif
    return true;
}

std::pair<std::array<double, 5>, std::array<double, 5>> ExtractRefLine::rotatedRect(double x, double y,
                                                                                    double half_width,
                                                                                    double half_height, double angle) {
    double c = cos(angle);
    double s = sin(angle);
    double r1x = -half_width * c - half_height * s;
    double r1y = -half_width * s + half_height * c;
    double r2x = half_width * c - half_height * s;
    double r2y = half_width * s + half_height * c;
    return {{x + r1x, x + r2x, x - r1x, x - r2x, x + r1x}, {y + r1y, y + r2y, y - r1y, y - r2y, y + r1y}};
}

bool ExtractRefLine::SetPathAvailableOne(int32_t side_lane_idx, EFMRefLine& side_path, 
                                         LaneElementGroupSets lane_element_group_sets){
    if (side_lane_idx >= 0) {
        side_path.bIsAvailable = false;
        for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
            bool exits_flag = false;
            for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
                auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
                if (lane_element.candidate_index == side_lane_idx) {
                    exits_flag = true;
                    side_path.bIsAvailable = lane_element.is_dest ? true : false;
                    break;
                }
            }
            if (exits_flag) {
                break;
            }
        }
    } else {
        side_path.bIsAvailable = false;
    }

    return true;
}


bool ExtractRefLine::SetPathAvailable(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                                      EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path, 
                                      EFMRefLine& right_path, EFMRefLine& right_right_path) {
    int32_t ego_lane_idx = candidate_lanes_model->ego_lane_prior_index();
    int32_t left_lane_idx = candidate_lanes_model->left_lane_prior_index();
    int32_t left_left_lane_idx = candidate_lanes_model->left_left_lane_prior_index();
    int32_t right_lane_idx = candidate_lanes_model->right_lane_prior_index();
    int32_t right_right_lane_idx = candidate_lanes_model->right_right_lane_prior_index();
    int32_t ref_lane_idx = candidate_lanes_model->get_ref_lane_prior_index();
    auto lane_element_group_sets = candidate_lanes_model->get_lane_element_group_sets();

    ego_path.bIsAvailable = ego_lane_idx >= 0 ? true : false;
    if (ref_lane_idx == ego_lane_idx && ego_lane_idx >= 0){
        SetPathAvailableOne(left_lane_idx, left_path, lane_element_group_sets);
        SetPathAvailableOne(left_left_lane_idx, left_left_path, lane_element_group_sets);
        SetPathAvailableOne(right_lane_idx, right_path, lane_element_group_sets);
        SetPathAvailableOne(right_right_lane_idx, right_right_path, lane_element_group_sets);
    }else if (ref_lane_idx == left_lane_idx && left_lane_idx >= 0){
        left_path.bIsAvailable = true;
        SetPathAvailableOne(left_left_lane_idx, left_left_path, lane_element_group_sets);
        SetPathAvailableOne(right_lane_idx, right_path, lane_element_group_sets);
        SetPathAvailableOne(right_right_lane_idx, right_right_path, lane_element_group_sets);
    }else if (ref_lane_idx == left_left_lane_idx && left_left_lane_idx >= 0){
        left_path.bIsAvailable = true;
        left_left_path.bIsAvailable = true;
        SetPathAvailableOne(right_lane_idx, right_path, lane_element_group_sets);
        SetPathAvailableOne(right_right_lane_idx, right_right_path, lane_element_group_sets);
    }else if (ref_lane_idx == right_lane_idx && right_lane_idx >= 0){
        SetPathAvailableOne(left_lane_idx, left_path, lane_element_group_sets);
        SetPathAvailableOne(left_left_lane_idx, left_left_path, lane_element_group_sets);
        right_path.bIsAvailable = true;
        SetPathAvailableOne(right_right_lane_idx, right_right_path, lane_element_group_sets);
    }else if (ref_lane_idx == right_right_lane_idx && right_right_lane_idx >= 0){
        SetPathAvailableOne(left_lane_idx, left_path, lane_element_group_sets);
        SetPathAvailableOne(left_left_lane_idx, left_left_path, lane_element_group_sets);
        right_path.bIsAvailable = true;
        right_right_path.bIsAvailable = true;
    }else{
        left_path.bIsAvailable = false;
        left_left_path.bIsAvailable = false;
        right_path.bIsAvailable = false;
        right_right_path.bIsAvailable = false;
    }
    
    
    // // left
    // if (left_lane_idx >= 0) {
    //     if (ref_lane_idx == left_lane_idx) {
    //         left_path.bIsAvailable = true;
    //     } else {
    //         left_path.bIsAvailable = false;
    //         for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
    //             bool exits_flag = false;
    //             for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
    //                 auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
    //                 if (lane_element.candidate_index == left_lane_idx) {
    //                     exits_flag = true;
    //                     left_path.bIsAvailable = lane_element.is_dest ? true : false;
    //                     break;
    //                 }
    //             }
    //             if (exits_flag) {
    //                 break;
    //             }
    //         }
    //     }
    // } else {
    //     left_path.bIsAvailable = false;
    // }

    // // right
    // if (right_lane_idx >= 0) {
    //     if (ref_lane_idx == right_lane_idx) {
    //         right_path.bIsAvailable = true;
    //     } else {
    //         right_path.bIsAvailable = false;
    //         for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
    //             bool exits_flag = false;
    //             for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
    //                 auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
    //                 if (lane_element.candidate_index == right_lane_idx) {
    //                     exits_flag = true;
    //                     right_path.bIsAvailable = lane_element.is_dest ? true : false;
    //                     break;
    //                 }
    //             }
    //             if (exits_flag) {
    //                 break;
    //             }
    //         }
    //     }
    // } else {
    //     right_path.bIsAvailable = false;
    // }

    return true;
}

bool ExtractRefLine::GetEndAnchorPoint(EFMRefLinePoints& line_points, int32_t split_point_number, 
                                       double Virtually_line_length, int32_t& anchor_point_number){
    anchor_point_number = split_point_number;
    if (line_points.size() <= split_point_number + 1){
        return true;
    }

    double need_length = 0;                               
    if (Virtually_line_length > VIRTUAL_LINE_STEP + 100){
        need_length = 100;
    }
    else if (Virtually_line_length <= VIRTUAL_LINE_STEP + 10)
    {
        need_length = 10;
    }
    else{
        need_length = Virtually_line_length - VIRTUAL_LINE_STEP;
    }

    double split_point_x = line_points[split_point_number].x > 0? line_points[split_point_number].x : 0;
    double last_point_x = split_point_x;
    for (int idx = split_point_number + 1; idx < line_points.size(); idx++){
        if (line_points[idx].x < split_point_x || line_points[idx].x < last_point_x){
            continue;
        }
        last_point_x = line_points[idx].x;

        if (line_points[idx].x > split_point_x + need_length){
            break;
        }
        anchor_point_number = idx;
    }
    return true;
}

bool ExtractRefLine::GetVirtuallyLineLength(double& Virtually_line_length, 
                                            std::vector<uint8_t> markr_section,
                                            std::vector<double> line_offsets){
    Virtually_line_length = 0;
    if (markr_section.size() > line_offsets.size() || 0 == markr_section.size()){
        return false;
    }
    
    for (int idx = 0; idx < markr_section.size(); idx++){
        if (3 == markr_section[idx] || 8 == markr_section[idx]){
            Virtually_line_length += line_offsets[idx];
        }
        else{
            break;
        }
    }
    
    return true;
}

bool ExtractRefLine::GetSideLine(std::vector<uint8_t>& split_line, 
                                 std::vector<std::pair<double, int>> curvature_nums_, 
                                 std::vector<uint8_t> left_markr_section,
                                 std::vector<uint8_t> right_markr_section,
                                 int32_t split_lane_index,
                                 int32_t next_split_lane_index,
                                 std::vector<double> line_offsets,
                                 std::vector<double>& split_line_offsets,
                                 uint8_t split_from){
    split_line ={};
    if (split_lane_index + 1 > curvature_nums_.size() || 
        line_offsets.size() < curvature_nums_.size() ){
        return false;
    }
    
    double all_curvature = 0;
    double all_offset = 0;
    // for (size_t idx = split_lane_index; idx < curvature_nums_.size(); idx++){
    for (size_t idx = split_lane_index; idx < split_lane_index + 1; idx++){
        if (all_offset >= 100 || split_lane_index == next_split_lane_index){
            all_curvature += curvature_nums_[split_lane_index].first;
            break;
        }
        all_curvature += curvature_nums_[split_lane_index].first;
        all_offset    += line_offsets[split_lane_index];  
    }

    // std::cout << __FILE__ << "," << __LINE__ << "," << "all_curvature: " << all_curvature << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "all_offset: " << all_offset << std::endl;

    std::vector<uint8_t> temp_mark_section = {};
    if (0 == split_from){
        return false;
    }
    else if (1 == split_from){
        temp_mark_section = left_markr_section;
    }else{
        temp_mark_section = right_markr_section;
    }

    int32_t temp_split_lane_index = 0;
    if (next_split_lane_index > split_lane_index || next_split_lane_index > 0){
        temp_split_lane_index = next_split_lane_index;
    }else{
        temp_split_lane_index = split_lane_index;
    }
    
    if (temp_split_lane_index + 1 > temp_mark_section.size()){
        return false;
    }

    for (size_t idx = temp_split_lane_index; idx < temp_mark_section.size(); idx++){
        split_line.push_back(temp_mark_section[idx]);
        split_line_offsets.push_back(line_offsets[idx]);
    }
    
    return true;
}

bool ExtractRefLine::AlterPointYForSplitPart(EFMRefLinePoints& line_points, 
                                             int32_t s_point_number, 
                                             int32_t e_point_number){
    if (s_point_number <0 || e_point_number < 0 || 
        s_point_number >= e_point_number || e_point_number + 1 > line_points.size()){
        return false;
    }
    
    //back split
    double max_x = line_points[e_point_number].x;
    //double min_x = line_points[s_point_number].x < 0? 0 : line_points[s_point_number].x;
    double min_x = line_points[s_point_number].x;
    double max_x_length = line_points[e_point_number].x - min_x;
    if (max_x_length <= 0){
        return true;
    }

    //double min_y = line_points[s_point_number].x < 0 ? 0 : line_points[s_point_number].y;
    double min_y =  line_points[s_point_number].y;
    double max_y_length = line_points[e_point_number].y - min_y;

    // std::stringstream s_ss_x;
    // std::stringstream s_ss_y;
    // for(int idx = first_idx; idx < split_point_number + 2; idx ++){
    //     s_ss_x << "," << line_points[idx].x;
    //     s_ss_y << "," << line_points[idx].y;
    // }

    // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_x.str() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_y.str() << std::endl;

    //change point
    std::vector<double> temp_line;
    bool is_overrun = false;
    for(int idx = s_point_number + 1; idx < e_point_number; idx ++){
        double temp_x_length = line_points[idx].x - min_x;
        if (temp_x_length <= 0){
            is_overrun = true;
            continue;
        }

        // double ratio = 4* (std::atan(temp_x_length/max_x_length))/M_PI;
        double ratio = temp_x_length/max_x_length;
        double temp_y = min_y + ratio*max_y_length;
        if (fabs(line_points[idx].y - temp_y) > 1.3){
            is_overrun = true;
            break;
        }
        temp_line.push_back(temp_y);
        //line_points[idx].y = min_y + ratio*max_y_length;
    }
    if (false == is_overrun){
        for(int idx = s_point_number + 1; idx < e_point_number && (idx - s_point_number -1) < temp_line.size(); idx ++){
            line_points[idx].y = temp_line[idx - s_point_number -1];
        }
    }

    return true;
}

bool ExtractRefLine::AlterPointYForSplit(EFMRefLinePoints& line_points, 
                                         int32_t split_point_number, 
                                         int32_t split_lane_index,
                                         int32_t next_split_point_number,
                                         int32_t next_split_lane_index,
                                         std::vector<uint8_t> left_markr_section, 
                                         std::vector<uint8_t> right_markr_section,
                                         std::vector<std::pair<double, int>> curvature_nums_,
                                         std::vector<double> line_offsets,
                                         uint8_t split_from,
                                         int back_lane_split_point_index,
                                         bool back_lane_is_split){

    if (split_point_number <=1 || line_points.size() <= split_point_number + 1){
        return true;
    }

    if (line_points[split_point_number].x > 3 * SPLIT_BACK_LENGTH){
        return true;
    }

    if (split_lane_index + 1 < curvature_nums_.size()){
        // for (int i = split_lane_index; i >= 0; i--){
        //     std::cout << __FILE__ << "," << __LINE__ << "," << "i: " << i << std::endl;
        //     std::cout << __FILE__ << "," << __LINE__ << "," << "curvature: " << curvature_nums_[i].first << std::endl;
        //     std::cout << __FILE__ << "," << __LINE__ << "," << "nums: " << curvature_nums_[i].second << std::endl;
        // }
        if ((curvature_nums_[split_lane_index].first/(double)curvature_nums_[split_lane_index].second) > 0.0015){
           return true;
        }
        
        
    }
    
    int32_t split_setp = 0;
    for (int temp_i = split_point_number + 1; temp_i < line_points.size(); temp_i++){
        if ((line_points[temp_i].y - line_points[split_point_number].y) > 1){
            split_setp = temp_i - split_point_number - 1;
            break;
        }
    }
    //  split_setp = split_setp > 2 ? 2 : split_setp;
     split_setp = 1;
    

    //
    std::vector<uint8_t> split_line;
    std::vector<double> split_line_offsets;
    GetSideLine(split_line, curvature_nums_, left_markr_section, right_markr_section, split_lane_index, 
                next_split_lane_index, line_offsets, split_line_offsets, split_from);

    //
    double Virtually_line_length = 0.0;
    GetVirtuallyLineLength(Virtually_line_length, split_line, line_offsets);

    ///
    int32_t anchor_point_number = split_point_number;
    // GetEndAnchorPoint(line_points, split_point_number, Virtually_line_length, anchor_point_number);
    if (next_split_point_number > split_point_number || next_split_point_number > 0){
        anchor_point_number = next_split_point_number;
        GetEndAnchorPoint(line_points, next_split_point_number, Virtually_line_length, anchor_point_number);
    }
    else{
        GetEndAnchorPoint(line_points, split_point_number, Virtually_line_length, anchor_point_number);
    }
    
    if (anchor_point_number + 1 > line_points.size()){
        return true;
    }
    
    //get need point
    double max_x = line_points[split_point_number + split_setp].x;
    int32_t first_idx = -1;
    int32_t min_split_point_idx = 0;
    if (back_lane_is_split && back_lane_split_point_index >= 0){
        min_split_point_idx = back_lane_split_point_index;
    }
    
    for(int idx = split_point_number - 1; idx >=0; idx--){
        if (line_points[idx].x + SPLIT_BACK_LENGTH <= max_x || idx <= min_split_point_idx){
            first_idx = idx;
            break;
        }
    }
    if (-1 == first_idx){
        return false;
    }
    // for (size_t i = 0; i < left_markr_section.size(); i++)
    // {
    //     std::cout << __FILE__ << "," << __LINE__ << "," << "i: " << i << ";marking:" << int(left_markr_section[i]) << std::endl;
    // }
    
    // std::cout << __FILE__ << "," << __LINE__ << "," << "split_point_number: " << split_point_number << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "next_split_point_number: " << next_split_point_number << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "anchor_point_number: " << anchor_point_number << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "split_lane_index: " << split_lane_index << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "next_split_lane_index: " << next_split_lane_index << std::endl;

    // std::stringstream s_ss_x;
    // std::stringstream s_ss_y;
    // for(int idx = first_idx; idx < anchor_point_number+1; idx ++){
    //     s_ss_x << "," << line_points[idx].x;
    //     s_ss_y << "," << line_points[idx].y;
    // }

    // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_x.str() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_y.str() << std::endl;

    uint8_t split_marking = 0;
    if (split_lane_index + 1 < curvature_nums_.size() && 
        split_lane_index + 1 < left_markr_section.size() && 
        split_lane_index + 1 < right_markr_section.size() &&
        split_lane_index + 1 < line_offsets.size()){
        if (curvature_nums_[split_lane_index].first > 0){
            split_marking = right_markr_section[split_lane_index];
        }else{
            split_marking = left_markr_section[split_lane_index];
        }
        
    }
    
    // if (8 == split_marking || 3 == split_marking){
    //     // std::cout << __FILE__ << "," << __LINE__ << "," << "split_marking: " << int(split_marking) << std::endl;
    //     // std::cout << __FILE__ << "," << __LINE__ << "," << "anchor_point_number: " << anchor_point_number << std::endl;
    //     AlterPointYForSplitPart(line_points, first_idx, anchor_point_number);
    // }else{
    //     // std::cout << __FILE__ << "," << __LINE__ << "," << "split_marking: " << int(split_marking) << std::endl;
    //     AlterPointYForSplitPart(line_points, first_idx, split_point_number + 2);
    //     if ( 0 == split_setp){
    //         AlterPointYForSplitPart(line_points, split_point_number, anchor_point_number); 
    //     }
    //     else{
    //         AlterPointYForSplitPart(line_points, split_point_number + split_setp - 1, anchor_point_number); 
    //     }
    // }
    AlterPointYForSplitPart(line_points, first_idx, split_point_number + split_setp);

    
    // if (next_split_lane_index < line_offsets.size() && next_split_lane_index > split_lane_index){
    // AlterPointYForSplitPart(line_points, first_idx, split_point_number + 2);
    // AlterPointYForSplitPart(line_points, split_point_number + 1, anchor_point_number);
    // }
    // else{
    //     AlterPointYForSplitPart(line_points, first_idx, anchor_point_number);
    // }

    // std::stringstream e_ss_x;
    // std::stringstream e_ss_y;
    // for(int idx = first_idx; idx < anchor_point_number+1; idx ++){
    //     e_ss_x << "," << line_points[idx].x;
    //     e_ss_y << "," << line_points[idx].y;
    // }

    // std::cout << __FILE__ << "," << __LINE__ << "," << e_ss_x.str() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << e_ss_y.str() << std::endl;
    

    // AlterPointYForSplitPart(line_points, first_idx, split_point_number + 2);

    // double min_x = line_points[first_idx].x < 0? 0 : line_points[first_idx].x;
    // double max_x_length = line_points[anchor_point_number].x - min_x;
    // if (max_x_length <= 0){
    //     return true;
    // }

    // double min_y = line_points[first_idx].y < 0 ? 0 : line_points[first_idx].y;
    // double max_y_length = line_points[anchor_point_number].y - min_y;

    // // std::stringstream s_ss_x;
    // // std::stringstream s_ss_y;
    // // for(int idx = first_idx; idx < split_point_number + 2; idx ++){
    // //     s_ss_x << "," << line_points[idx].x;
    // //     s_ss_y << "," << line_points[idx].y;
    // // }

    // // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_x.str() << std::endl;
    // // std::cout << __FILE__ << "," << __LINE__ << "," << s_ss_y.str() << std::endl;

    // //change point
    // for(int idx = first_idx + 1; idx < anchor_point_number; idx ++){
    //     double temp_x_length = line_points[idx].x - min_x;
    //     if (temp_x_length <= 0){
    //         continue;
    //     }

    //     double ratio = 4* (std::atan(temp_x_length/max_x_length))/M_PI;
    //     line_points[idx].y = min_y + ratio*max_y_length;
    // }

    double min_x = line_points[first_idx].x < 0? 0 : line_points[first_idx].x;
    double max_x_length = max_x - min_x;
    int32_t min_idx = -1;
    for (int32_t idx = first_idx; idx >=0; idx--){
        if (line_points[idx].x + SPLIT_BACK_LENGTH + 10 < max_x_length|| line_points[idx].x <=0){
                min_idx = idx;
            break;
        }
    }
    if (min_idx >= 0){
        double min_x = line_points[min_idx].x < 0? 0 : line_points[min_idx].x;
        double min_y = line_points[min_idx].x < 0? 0 : line_points[min_idx].y;
        max_x_length = line_points[first_idx+1].x - min_x;
        double max_y = line_points[first_idx+1].y - min_y;
        std::vector<double> temp_line;
        bool is_overrun = false;
        if (max_x_length > 0){
            for (int32_t idx = min_idx; idx <= first_idx; idx++){
                double temp_length = line_points[min_idx].x - min_x;
                if (temp_length <= 0){
                    is_overrun = true;
                    break;
                }

                double temp_y = line_points[idx].y + (temp_length/max_x_length)*max_y;
                if (fabs(line_points[idx].y - temp_y) > 0.8){
                    is_overrun = true;
                    break;
                }
                temp_line.push_back(temp_y);
                // line_points[idx].y = line_points[idx].y + (temp_length/max_x_length)*max_y;
            }

            if (false == is_overrun){
                for (int32_t idx = min_idx; idx <= first_idx && (idx - min_idx) < temp_line.size(); idx++){
                    line_points[idx].y = temp_line[idx-min_idx];
                }
            }
        }
        
    }
    
    

    

    // std::stringstream e_ss_x;
    // std::stringstream e_ss_y;
    // for(int idx = first_idx; idx < split_point_number + 2; idx ++){
    //     e_ss_x << "," << line_points[idx].x;
    //     e_ss_y << "," << line_points[idx].y;
    // }

    // std::cout << __FILE__ << "," << __LINE__ << "," << e_ss_x.str() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << e_ss_y.str() << std::endl;

    return true;
}

bool ExtractRefLine::GetLaneCurvature(const std::vector<uint8_t>& candidate_lane,
                                      const std::vector<uint32_t>& link_id_vec, EFMRefLine& path) {
    if (link_id_vec.size() < candidate_lane.size()) {
        return false;
    }
    for (int i = 0; i < candidate_lane.size(); i++) {
        uint32_t link_id = link_id_vec[i];
        uint8_t lane_num = candidate_lane[i];

        auto iter = std::find_if(map_static_info_->LinkCurvatures.LinkCurvatures.begin(),
                                 map_static_info_->LinkCurvatures.LinkCurvatures.end(),
                                 [&](const message::map_map::s_LinkCurvature_t& it) {
                                     return (it.InstanceId.InstanceId == link_id && it.LaneNum.LaneNum == lane_num);
                                 });

        double curvature_val = 0;
        int number = 0;
        if (iter != map_static_info_->LinkCurvatures.LinkCurvatures.end()) {
            for (auto CurvPoint : (*iter).CurvPoints.CurvPoints) {
                curvature_val += static_cast<double>(CurvPoint.CurvPointValue.CurvPointValue / 100000);
                number++;
                // break;
            }
        } else {
            // do nothing
        }
        path.sum_curvature_vec.push_back(std::make_pair(curvature_val, number));
    }
    path.laneMarkSectionLength = link_length_vec_;

#ifdef EXTRACTREFLINE_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.laneMarkSectionLength.size(): " << path.laneMarkSectionLength.size() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.laneMarkSectionLength: " << std::endl;
    for (auto type : path.laneMarkSectionLength) {
        std::cout << ", " << type;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.sum_curvature_vec.size(): " << path.sum_curvature_vec.size() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.sum_curvature_vec: " << std::endl;
    for (auto type : path.sum_curvature_vec) {
        std::cout << ", <" << type.first << "," << type.second<<">";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.leftLineMkrType.size(): " << path.leftLineMkrType.size() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.leftLineMkrType: " << std::endl;
    for (auto type : path.leftLineMkrType) {
        std::cout << ", " << (int)type;
    }
    std::cout << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.rightLineMkrType.size(): " << path.rightLineMkrType.size() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "path.rightLineMkrType: " << std::endl;
    for (auto type : path.rightLineMkrType) {
        std::cout << ", " << (int)type;
    }
    std::cout << std::endl;
#endif
    return true;
}

bool ExtractRefLine::GetLaneTrans(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans) {
    trans = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (trans == 0 || trans == 99) {
            trans = 2;
            }
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            trans = laneinfo.Transit.data;
        }
    }

    return true;
}

bool ExtractRefLine::GetSplitFrom(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, int& split_from) {
    // split_from = 0;
    // for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
    //     if ((lane_id + 1) == laneinfo.LaneNum.LaneNum && 3 == laneinfo.Transit.data) {
    //         split_from = 2;

    //     } else if (lane_id == (laneinfo.LaneNum.LaneNum + 1) && 3 == laneinfo.Transit.data) {
    //          split_from = 1;
    //     }
    // }

    return true;
}

bool ExtractRefLine::SaperateLaneMarkerIntoTwoPart(const EFMRefLineMarkr& marker_raw, EFMPoint tar_point,
                                                   const std::vector<uint32_t>& mkr_offset,uint32_t ego_pos_offset,
                                                   EFMRefLineMarkrSection& marker_section) {
    if(mkr_offset.size()!=marker_raw.size()){
       return false;
    }                                                
    marker_section[0].clear();
    marker_section[1].clear();
    EFMPoint proj_point;
    bool is_inside = false;
    int nearest_index = 0;
    EFMRefLinePoints line_points{};
    for (int i = 0; i < marker_raw.size(); i++) {
        line_points.push_back({marker_raw[i].x, marker_raw[i].y});
    }
    if (CommonMathMethod::DiscretePointsMath::GetProjectPointBodyCoordinate(tar_point, line_points,ego_pos_offset,mkr_offset, proj_point,
                                                                            is_inside, nearest_index) == false) {
        return false;
    }

    if (is_inside == false || nearest_index >= marker_raw.size() || nearest_index < 0) {
        // project point not inside line
        return false;
    }

    EFMMarkr proj_mkr(proj_point.x, proj_point.y, marker_raw[nearest_index].type, marker_raw[nearest_index].color);
    if (nearest_index < marker_raw.size() && marker_raw.size() > 0) {
        if (proj_point.x > marker_raw[nearest_index].x) {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index + 1; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }

        } else {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index - 1; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "nearest_index: " << nearest_index << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "is_inside: " << is_inside << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "proj_point.x: " << proj_point.x << " ,proj_point.y: " << proj_point.y << std::endl;

    return true;
}

bool ExtractRefLine::SaperateLaneCenterLineIntoTwoPart(const EFMRefLinePoints& marker_raw, EFMPoint tar_point,
                                                       const std::vector<uint32_t>& pts_offset,uint32_t ego_pos_offset,
                                                       EFMRefLinePointsSection& marker_section) {
    if(pts_offset.size()!=marker_raw.size()){
       return false;
    }                                                        
    marker_section[0].clear();
    marker_section[1].clear();
    EFMPoint proj_point;
    bool is_inside = false;
    int nearest_index = 0;
    EFMRefLinePoints line_points{};
    for (int i = 0; i < marker_raw.size(); i++) {
        line_points.push_back({marker_raw[i].x, marker_raw[i].y});
    }
    if (CommonMathMethod::DiscretePointsMath::GetProjectPointBodyCoordinate(tar_point, line_points, ego_pos_offset,pts_offset,proj_point,
                                                                            is_inside, nearest_index) == false) {
        return false;
    }

    if (is_inside == false || nearest_index >= marker_raw.size() || nearest_index < 0) {
        // project point not inside line
        return false;
    }

    EFMPoint proj_mkr(proj_point.x, proj_point.y);
    if (nearest_index < marker_raw.size() && marker_raw.size() > 0) {
        if (proj_point.x > marker_raw[nearest_index].x) {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index + 1; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }

        } else {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index - 1; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "nearest_index: " << nearest_index << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "is_inside: " << is_inside << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "proj_point.x: " << proj_point.x << " ,proj_point.y: " << proj_point.y << std::endl;

    return true;
}

bool ExtractRefLine::GetLaneSplitSmoothInfo(uint32_t ego_offset, uint32_t link_id, uint8_t lane_num,
                                        std::vector<uint32_t> next_link_id_vec, std::vector<uint8_t> next_lane_id_vec, const std::vector<uint32_t>& last_link_id_vec, const std::vector<uint8_t>& last_lane_id_vec,
                                        SmoothSplitInfo& smooth_split_info){
    // std::cout << __FILE__ << "," << __LINE__ << ","<< "GetLaneSplitSmoothInfo##############   " <<"link_id: "<<link_id<<" ,lane_num: "<<(int)lane_num<< std::endl; 
    // std::cout <<"next_link_id_vec: " ; 
    // for(auto link:next_link_id_vec){
    //      std::cout<<" ,"<<link;
    // }   
    // std::cout<<std::endl;
    // std::cout <<"next_lane_id_vec: " ; 
    // for(auto link:next_lane_id_vec){
    //      std::cout<<" ,"<<(int)link;
    // }   
    // std::cout<<std::endl;
    // std::cout <<"last_link_id_vec: " ; 
    // for(auto link:last_link_id_vec){
    //      std::cout<<" ,"<<link;
    // }   
    // std::cout<<std::endl;
    // std::cout <<"last_lane_id_vec: " ; 
    // for(auto link:last_lane_id_vec){
    //      std::cout<<" ,"<<(int)link;
    // }   
    // std::cout<<std::endl;
    //保存smooth要的split附近的信息, 只保存要平滑选出的车道，所以输入std::vector<uint32_t> next_link_id, std::vector<uint8_t> next_lane_id, const std::deque<uint32_t>& last_link_id, const std::deque<uint8_t>& last_lane_id,
    //last_lane_id_vec ,last_link_id_vec的【0】是离split最近
    if(next_link_id_vec.size()<next_lane_id_vec.size() || last_link_id_vec.size()!=last_lane_id_vec.size()||
        next_link_id_vec.empty()||next_lane_id_vec.empty()){
        return false;
    }
    bool get_split_f = false;
    double p_split_take_length = 100;//split分歧后的取的信息的距离
    
    std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_raw{};
    if(efm::MapCommonTool::GetInstance()->GetLaneFromConnectivitys(map_static_info_,link_id_index_lane_connect_map_,link_id,lane_num,lane_Connectivitys_raw)){
        int connect_size = 0;
        int forward_index = 0;
        message::map_map::s_LaneInfo_t lane_info_raw{};
        lane_info_raw.LaneNum.LaneNum = 0;
        message::map_map::s_LaneInfo_t lane_info_side{};
        lane_info_side.LaneNum.LaneNum = 0;
        uint32_t side_lane_link_id = 0;
        double side_link_length = 0;
        double link_length = 0;
        for(auto connect:lane_Connectivitys_raw){
            message::map_map::s_LinkInfo_t link_infos{};
            if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, connect.ToLinkId.ToLink, link_infos)){
                message::map_map::s_LaneInfo_t lane_info{};
                if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.NewLaneNum.NewLaneNum, lane_info)){
                    connect_size++;
                    if(connect_size == 1){
                        //判断是不是选出的车道，是那么放入lane_info_raw
                        if(forward_index<next_lane_id_vec.size()&& forward_index<next_link_id_vec.size()){
                            if(connect.NewLaneNum.NewLaneNum == next_lane_id_vec[forward_index] && connect.ToLinkId.ToLink == next_link_id_vec[forward_index]){
                                link_length = (static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0;
                                lane_info_raw = lane_info;
                            }else{
                                side_lane_link_id = connect.ToLinkId.ToLink;
                                side_link_length = (static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0;
                                lane_info_side = lane_info;
                            } 
                        }
                    }else{
                        //是split, 记录信息, 先记录split处第一段
                        // std::cout << __FILE__ << "," << __LINE__ << ","<< "get_split" << std::endl;                          
                        get_split_f = true;
                        if(lane_info_raw.LaneNum.LaneNum ==0 ){
                            link_length = (static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0;
                            lane_info_raw = lane_info;
                        }else{
                            side_link_length = (static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0;
                            side_lane_link_id = connect.ToLinkId.ToLink;
                            lane_info_side = lane_info;
                        }
                        smooth_split_info.dist = (static_cast<double>(link_infos.PathOffset.PathOffset) - static_cast<double>(ego_offset))/100.0;
                        smooth_split_info.road_class = GetRoadClass(map_static_info_, link_id);
                        smooth_split_info.link_id.push_back(next_link_id_vec[forward_index]);
                        smooth_split_info.lane_id.push_back(next_lane_id_vec[forward_index]);
                        smooth_split_info.side_link_id.push_back(side_lane_link_id);
                        smooth_split_info.side_lane_id.push_back(lane_info_side.LaneNum.LaneNum);  
                        smooth_split_info.side_link_length.push_back(side_link_length); 
                        smooth_split_info.link_length.push_back(link_length);                        
                        message::map_map::s_LaneWidth_t lane_width{};
                        if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_, next_link_id_vec[forward_index],next_lane_id_vec[forward_index], lane_width)){
                            smooth_split_info.max_width.push_back(lane_width.MaxWidth.MaxWidth);
                            smooth_split_info.min_width.push_back(lane_width.MinWidth.MinWidth);
                        }else{
                            smooth_split_info.max_width.push_back(0);
                            smooth_split_info.min_width.push_back(0);                            
                        }
                        if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_, side_lane_link_id,lane_info_side.LaneNum.LaneNum, lane_width)){
                            smooth_split_info.side_max_width.push_back(lane_width.MaxWidth.MaxWidth);
                            smooth_split_info.side_min_width.push_back(lane_width.MinWidth.MinWidth);
                        }else{
                            smooth_split_info.side_max_width.push_back(0);
                            smooth_split_info.side_min_width.push_back(0);                            
                        }
                        //曲率
                        std::vector<CurvPoint> curvpoints{};
                        efm::MapCommonTool::GetInstance()->SaveLaneCurvInfo(next_lane_id_vec[forward_index], next_link_id_vec[forward_index], *curve_index_map_, *map_static_info_, curvpoints);
                        std::vector<std::pair<double,double>> curvpoints_ego_pos{};
                        for(auto curvp:curvpoints){
                            curvpoints_ego_pos.push_back(std::make_pair(
                                (static_cast<double>(curvp.CurvPointPathOffset)-static_cast<double>(ego_offset))/100.0, curvp.CurvPointValue));
                        }
                        smooth_split_info.lane_curvpoints.push_back(curvpoints_ego_pos);
                        //边线
                        std::vector<message::map_map::s_GeometryPoint_t> geometry_points{};
                        uint8_t mrk_type =0;
                        uint8_t mrk_color  = 0;                      
                        efm::MapCommonTool::GetInstance()->GetLine(lane_info_raw.LBound.LBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                        smooth_split_info.left_line.push_back(mrk_type);
                        efm::MapCommonTool::GetInstance()->GetLine(lane_info_raw.RBound.RBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                        smooth_split_info.right_line.push_back(mrk_type);
                        efm::MapCommonTool::GetInstance()->GetLine(lane_info_side.LBound.LBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                        smooth_split_info.side_left_line.push_back(mrk_type);
                        efm::MapCommonTool::GetInstance()->GetLine(lane_info_side.RBound.RBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                        smooth_split_info.side_right_line.push_back(mrk_type);
                        //限速
                        message::map_map::s_LaneSpeedLimit_t lane_speed_limit{};                  
                        if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, next_link_id_vec[forward_index],next_lane_id_vec[forward_index], lane_speed_limit)){
                            smooth_split_info.speed_limit.push_back(lane_speed_limit.ValueH.ValueH);
                        }else{
                            smooth_split_info.speed_limit.push_back(0);
                        }
                        if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, side_lane_link_id, lane_info_side.LaneNum.LaneNum, lane_speed_limit)){
                            smooth_split_info.side_speed_limit.push_back(lane_speed_limit.ValueH.ValueH);
                        }else{
                            smooth_split_info.side_speed_limit.push_back(0);
                        }
                        //lane type
                        smooth_split_info.lane_type.push_back(lane_info_raw.LaneType.data);
                        smooth_split_info.side_lane_type.push_back(lane_info_side.LaneType.data);
                        
                        //step 2，向自车行驶方向找连接的第二段
                        forward_index++;
                        uint32_t while_side_link_id = 0;
                        uint8_t while_side_lane_id = 0;
                        double split_forward_length = link_length;
                        while(forward_index<6 && smooth_split_info.side_link_id.size()>0 && smooth_split_info.side_lane_id.size()>0 && split_forward_length<=p_split_take_length){
                            //raw的用 next_lane_id_vec, next_link_id_vec找，side用连接关系找
                            if(forward_index<next_lane_id_vec.size()&& forward_index<next_link_id_vec.size()){
                                if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, next_link_id_vec[forward_index], link_infos)){
                                    if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, next_lane_id_vec[forward_index], lane_info)){
                                        smooth_split_info.link_id.push_back(next_link_id_vec[forward_index]);
                                        smooth_split_info.lane_id.push_back(next_lane_id_vec[forward_index]);
                                        smooth_split_info.link_length.push_back((static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0);
                                        //曲率
                                        efm::MapCommonTool::GetInstance()->SaveLaneCurvInfo(next_lane_id_vec[forward_index], next_link_id_vec[forward_index], *curve_index_map_, *map_static_info_, curvpoints);
                                        curvpoints_ego_pos.clear();
                                        for(auto curvp:curvpoints){
                                            curvpoints_ego_pos.push_back(std::make_pair(
                                                (static_cast<double>(curvp.CurvPointPathOffset)-static_cast<double>(ego_offset))/100.0, curvp.CurvPointValue));
                                        }
                                        smooth_split_info.lane_curvpoints.push_back(curvpoints_ego_pos);
                                        //边线
                                        efm::MapCommonTool::GetInstance()->GetLine(lane_info.LBound.LBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                                        smooth_split_info.left_line.push_back(mrk_type);
                                        efm::MapCommonTool::GetInstance()->GetLine(lane_info.RBound.RBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                                        smooth_split_info.right_line.push_back(mrk_type);                                        
                                    }
                                    if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_, next_link_id_vec[forward_index],next_lane_id_vec[forward_index], lane_width)){
                                        smooth_split_info.max_width.push_back(lane_width.MaxWidth.MaxWidth);
                                        smooth_split_info.min_width.push_back(lane_width.MinWidth.MinWidth);
                                    }else{
                                        smooth_split_info.max_width.push_back(0);
                                        smooth_split_info.min_width.push_back(0);                            
                                    }
                                    //限速
                                    message::map_map::s_LaneSpeedLimit_t lane_speed_limit_tmp{};                  
                                    if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, next_link_id_vec[forward_index],next_lane_id_vec[forward_index], lane_speed_limit_tmp)){
                                        smooth_split_info.speed_limit.push_back(lane_speed_limit_tmp.ValueH.ValueH);
                                    }else{
                                        smooth_split_info.speed_limit.push_back(0);
                                    }
                                    //lane type
                                    smooth_split_info.lane_type.push_back(lane_info.LaneType.data);

                                    split_forward_length += (static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0;
                                }else{
                                    break;
                                }
                            }else{

                                break;
                            }

                            //side 
                            while_side_link_id = smooth_split_info.side_link_id.back();
                            while_side_lane_id = smooth_split_info.side_lane_id.back();
                            // std::cout << __FILE__ << "," << __LINE__ << ","<< "forward_index" <<forward_index<< std::endl; 
                            // std::cout << __FILE__ << "," << __LINE__ << ","<< "while_side_link_id " << while_side_link_id<<"  while_side_lane_id: "<<(int)while_side_lane_id<<std::endl; 
                            std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_side{};
                            bool find_side_lane = false;
                            if(efm::MapCommonTool::GetInstance()->GetLaneFromConnectivitys(map_static_info_,link_id_index_lane_connect_map_,while_side_link_id,while_side_lane_id,lane_Connectivitys_side)){
                                //相同的link 只保存相邻的，不相同link就不放了
                                for(auto connect:lane_Connectivitys_side){
                                    if(connect.ToLinkId.ToLink == next_link_id_vec[forward_index] 
                                    && efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, next_link_id_vec[forward_index], link_infos)){
                                        // if((connect.NewLaneNum.NewLaneNum == next_lane_id_vec[forward_index]+1) ||(connect.NewLaneNum.NewLaneNum == next_lane_id_vec[forward_index]-1)){
                                            smooth_split_info.side_link_id.push_back(connect.ToLinkId.ToLink);
                                            smooth_split_info.side_lane_id.push_back(connect.NewLaneNum.NewLaneNum); 
                                            smooth_split_info.side_link_length.push_back((static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0);
                                            if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.NewLaneNum.NewLaneNum, lane_info)){
                                                //边线
                                                efm::MapCommonTool::GetInstance()->GetLine(lane_info.LBound.LBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                                                smooth_split_info.side_left_line.push_back(mrk_type);
                                                efm::MapCommonTool::GetInstance()->GetLine(lane_info.RBound.RBound, *map_static_info_, *linear_obj_id_map_,geometry_points, mrk_type,mrk_color);
                                                smooth_split_info.side_right_line.push_back(mrk_type);  
                                            }
                                            if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_, connect.ToLinkId.ToLink,connect.NewLaneNum.NewLaneNum, lane_width)){
                                                smooth_split_info.side_max_width.push_back(lane_width.MaxWidth.MaxWidth);
                                                smooth_split_info.side_min_width.push_back(lane_width.MinWidth.MinWidth);
                                            }else{
                                                smooth_split_info.side_max_width.push_back(0);
                                                smooth_split_info.side_min_width.push_back(0);                            
                                            }
                                            //限速
                                            message::map_map::s_LaneSpeedLimit_t lane_speed_limit_tmp{};                  
                                            if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, while_side_link_id,while_side_lane_id, lane_speed_limit_tmp)){
                                                smooth_split_info.side_speed_limit.push_back(lane_speed_limit_tmp.ValueH.ValueH);
                                            }else{
                                                smooth_split_info.side_speed_limit.push_back(0);
                                            }
                                            //lane type
                                            smooth_split_info.side_lane_type.push_back(lane_info.LaneType.data);
                                            find_side_lane = true;
                                            break;                                            
                                        // }
                                    }
                                }
                            }
                            forward_index++;
                        }

                        //保存split前的信息
                        // std::cout << __FILE__ << "," << __LINE__ << ","<< "last_link_id_vec.size() " <<last_link_id_vec.size()<<" last_lane_id_vec.size():"<<last_lane_id_vec.size() <<std::endl;
                        int for_times = 0; 
                        if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, link_id, link_infos)){
                            smooth_split_info.before_link_length.push_back((static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0);
                        }
                        if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_,link_id,lane_num, lane_width)){
                            smooth_split_info.before_split_max_width.push_back(lane_width.MaxWidth.MaxWidth);
                            smooth_split_info.before_split_min_width.push_back(lane_width.MinWidth.MinWidth);
                        }else{
                            smooth_split_info.before_split_max_width.push_back(0);
                            smooth_split_info.before_split_min_width.push_back(0);                            
                        }
                        //曲率
                        efm::MapCommonTool::GetInstance()->SaveLaneCurvInfo(lane_num, link_id, *curve_index_map_, *map_static_info_, curvpoints);
                        curvpoints_ego_pos.clear();
                        for(auto curvp:curvpoints){
                            curvpoints_ego_pos.push_back(std::make_pair(
                                (static_cast<double>(curvp.CurvPointPathOffset)-static_cast<double>(ego_offset))/100.0, curvp.CurvPointValue));
                        }
                        smooth_split_info.before_lane_curvpoints.push_back(curvpoints_ego_pos);
                        //限速
                        message::map_map::s_LaneSpeedLimit_t lane_speed_limit_tmp{}; 
                        // std::cout << __FILE__ << "," << __LINE__ << ","<< "link_id " <<link_id<<" lane_num:"<<(int)lane_num<<std::endl;
                        
                        if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, link_id,lane_num, lane_speed_limit_tmp)){
                            smooth_split_info.before_speed_limit.push_back(lane_speed_limit_tmp.ValueH.ValueH);
                        }else{
                            smooth_split_info.before_speed_limit.push_back(0);
                        } 
                        // std::cout << __FILE__ << "," << __LINE__ << ","<< "lane_speed_limit link: " <<lane_speed_limit.InstanceId.InstanceId
                        // <<" lane_num:"<<(int)lane_speed_limit.LaneNum.LaneNum<<" valueH:"<<(int)lane_speed_limit.ValueH.ValueH<<std::endl;
                        //lane type
                        efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, lane_num, lane_info);
                        smooth_split_info.before_lane_type.push_back(lane_info.LaneType.data);
                        
                        for(int i =last_link_id_vec.size()-1;i<last_link_id_vec.size() && i<last_lane_id_vec.size() && i>=0 && for_times<2;i--){
                            if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, last_link_id_vec[i], link_infos)){
                                smooth_split_info.before_link_length.push_back((static_cast<double>(link_infos.EndOffset.EndOffset)-static_cast<double>(link_infos.PathOffset.PathOffset))/100.0);
                            }
                            // std::cout << __FILE__ << "," << __LINE__ << ","<<"i:"<<i<< " last_link_id_vec[i] " <<last_link_id_vec[i]<<" ,last_lane_id_vec[i]:"<<(int)last_lane_id_vec[i] <<std::endl; 
                            if(efm::MapCommonTool::GetInstance()->GetLaneWidth(map_static_info_,last_link_id_vec[i],last_lane_id_vec[i], lane_width)){
                                smooth_split_info.before_split_max_width.push_back(lane_width.MaxWidth.MaxWidth);
                                smooth_split_info.before_split_min_width.push_back(lane_width.MinWidth.MinWidth);
                            }else{
                                smooth_split_info.before_split_max_width.push_back(0);
                                smooth_split_info.before_split_min_width.push_back(0);                            
                            }
                            //曲率
                            efm::MapCommonTool::GetInstance()->SaveLaneCurvInfo(last_lane_id_vec[i], last_link_id_vec[i], *curve_index_map_, *map_static_info_, curvpoints);
                            curvpoints_ego_pos.clear();
                            for(auto curvp:curvpoints){
                                curvpoints_ego_pos.push_back(std::make_pair(
                                    (static_cast<double>(curvp.CurvPointPathOffset)-static_cast<double>(ego_offset))/100.0, curvp.CurvPointValue));
                            }
                            smooth_split_info.before_lane_curvpoints.push_back(curvpoints_ego_pos);
                            //限速
                            message::map_map::s_LaneSpeedLimit_t lane_speed_limit_tmp{};                  
                            if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, last_link_id_vec[i],last_lane_id_vec[i], lane_speed_limit_tmp)){
                                smooth_split_info.before_speed_limit.push_back(lane_speed_limit_tmp.ValueH.ValueH);
                            }else{
                                smooth_split_info.before_speed_limit.push_back(0);
                            } 
                            //lane type
                            efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, last_lane_id_vec[i], lane_info);
                            smooth_split_info.before_lane_type.push_back(lane_info.LaneType.data);
                            for_times++;
                        }                       

                        break;
                    }

                }
            }
        } 
    }   
    return get_split_f;
}

void ExtractRefLine::GetLaneSplitStartS(uint32_t ego_offset, uint32_t link_id, uint8_t lane_num,
                                        const message::map_map::s_LinkInfo_t& link_infos,uint32_t next_link_id,
                                        std::vector<std::pair<double,double>>& split_start_s_vec) {
    // 如果trans ==3， 那么记录s， 自车位置为0点
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "GetLaneSplitStartS: start" << std::endl;    
    bool get_split_f = false;
    // 如果 split 属性找不到， 还要判断 是不是一条lane连接多条lane
    int connect_size = 0;
    if (get_split_f == false) {
        // 当前link在静态地图中connect中的的索引
        std::vector<int> link_staticmap_indices{};
        if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
            link_staticmap_indices = link_id_index_lane_connect_map_->at(link_id);
        } else {
            // std::cout << " link can't find connect : " << last_link_id << std::endl;
        }
        for (int index : link_staticmap_indices) {
            uint8_t NewLaneNum = map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum;
            uint8_t InitLaneNum = map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;
            if (InitLaneNum == lane_num) {
                // 找有没有连接多个车道
                connect_size++;
            }
            if (connect_size > 1) {
                double s_temp = static_cast<double>(link_infos.EndOffset.EndOffset) / 100.0 -
                                static_cast<double>(ego_offset) / 100.0;
                //需要分歧点之后的一段link 长度
                double link_length = 0;
                message::map_map::s_LinkInfo_t next_link_infos;
                if (!GetLinkInfos(next_link_id, next_link_infos)){ 
                    split_start_s_vec.push_back(std::make_pair(s_temp,link_length));
                    get_split_f = true;                    
                }else{
                    link_length = static_cast<double>(next_link_infos.EndOffset.EndOffset) / 100.0 - static_cast<double>(next_link_infos.PathOffset.PathOffset) / 100.0; 
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    // << "s_temp:" <<s_temp <<" ,link_length:"<<next_link_length<<" ,split_link:"<<link_infos.InstanceId.InstanceId<<" ,lane_num:"<<(int)lane_num<<" , next_link_id:"<<next_link_id<<std::endl;                        
                    split_start_s_vec.push_back(std::make_pair(s_temp,link_length));
                    get_split_f = true;                     
                }              

                break;
            }
        }
    }

    // if(get_split_f){
    //     std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "connect_size: " <<connect_size<< std::endl;   
    //                   std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "link_id: " <<(int)link_id<< " lane_num: " <<(int)lane_num<<" ego_offset: " <<(int)ego_offset<<" link_infos.PathOffset: " <<(int)link_infos.EndOffset.EndOffset<< std::endl;   

    // }  
    return;
}

void ExtractRefLine::GetLaneMergeEndS(uint32_t ego_offset, uint32_t last_link_id, uint8_t cur_lane_num,
                                        const message::map_map::s_LinkInfo_t& cur_link_infos,
                                        std::vector<std::pair<double,double>>& merge_end_s_vec) {
    //是不是多条lane连接1条lane,last link里有多条lane 连接这个link的lane_num号车道, 
    bool get_merge_f =false;
    int connect_size = 0;
    uint32_t cur_link_id = cur_link_infos.InstanceId.InstanceId;
    if (get_merge_f == false) {
        // 当前link在静态地图中connect中的的索引
        std::vector<int> link_staticmap_indices{};
        if (link_id_index_lane_connect_map_->find(last_link_id) != link_id_index_lane_connect_map_->end()) {
            link_staticmap_indices = link_id_index_lane_connect_map_->at(last_link_id);
        } else {
            // std::cout << " link can't find connect : " << last_link_id << std::endl;
        }
        for (int index : link_staticmap_indices) {
            uint8_t NewLaneNum = map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum;
            // uint8_t InitLaneNum = map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;
            uint32_t next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
            if (NewLaneNum == cur_lane_num && next_link_id == cur_link_id) {
                // 找有没有连接多个车道
                connect_size++;
            }
            if (connect_size > 1) {
                double s_temp = static_cast<double>(cur_link_infos.PathOffset.PathOffset) / 100.0 -
                                static_cast<double>(ego_offset) / 100.0;
                double link_length = 0;
                message::map_map::s_LinkInfo_t last_link_infos;
                if (!GetLinkInfos(last_link_id, last_link_infos)){
                    //
                    merge_end_s_vec.push_back(std::make_pair(s_temp,link_length));
                    get_merge_f = true;
                }else{
                    //需要 merge点前的一段link的长度
                    link_length = static_cast<double>(last_link_infos.EndOffset.EndOffset) / 100.0 - static_cast<double>(last_link_infos.PathOffset.PathOffset) / 100.0;
                    merge_end_s_vec.push_back(std::make_pair(s_temp,link_length));
                    // std::cout<< "cur_link_id : " << cur_link_infos.InstanceId.InstanceId << " ,cur_lane_num : " << (int)cur_lane_num<<" ,s_temp: "<<s_temp 
                    // <<" ,last_link_id"<<last_link_id<<" ,last_link_length: "<<link_length<< std::endl;
                    get_merge_f = true;                    
                }

                break;
            }
        }
    }

    // if(get_merge_f){
    //     std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "connect_size: " <<connect_size<< std::endl;   
    //                   std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "last_link_id: " <<(int)last_link_id<< "cur_lane_num: " <<(int)cur_lane_num<<" ego_offset: " <<(int)ego_offset<<" link_infos.PathOffset: " <<(int)cur_link_infos.PathOffset.PathOffset<< std::endl;   

    // }  

    return;
}

uint8_t ExtractRefLine::GetRoadClass(std::shared_ptr<const TopicTrait::MapMapMsg> map_msg, uint32_t link_id){
    uint8_t road_class = 0;
        auto iter = std::find_if(
            map_msg->FormOfWays.FormOfWays.begin(), map_msg->FormOfWays.FormOfWays.end(),
            [&](const message::map_map::s_FormOfWay_t& it) { return it.InstanceId.InstanceId == link_id; });

        if (iter != map_msg->FormOfWays.FormOfWays.end()) {
            road_class = (*iter).FunctionRoadClass.data;
        }
    return road_class;
}

void ExtractRefLine::PlotSmoothSplitInfo(std::string path_name, const std::vector<SmoothSplitInfo>& smooth_split_info){
        std::cout << __FILE__ << "," << __LINE__ << ","<<path_name<<'.'
              << "smooth_split_info.size(): " <<smooth_split_info.size()<< std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5, ss6,ss7, ss8, ss9, ss10,ss11,ss12,ss13,ss14,ss15,ss16,ss17,ss18,ss19,ss20,ss21,ss22,ss23,ss24,ss25,ss26;
    ss<<path_name<<'.' << "smooth_split_info.dist: ";
    ss26<<path_name<<'.' << "smooth_split_info.road_class: ";
    ss1 <<path_name<<'.'<< "smooth_split_info.link_id: ";
    ss2<<path_name<<'.' << "smooth_split_info.lane_id: ";
    ss3<<path_name<<'.' << "smooth_split_info.side_link_id: ";
    ss4<<path_name<<'.' << "smooth_split_info.side_lane_id: ";
    ss5<<path_name<<'.'<< "smooth_split_info.max_width: ";
    ss6<<path_name<<'.' << "smooth_split_info.side_max_width: ";
    ss7<<path_name<<'.'<< "smooth_split_info.min_width: ";
    ss8<<path_name<<'.' << "smooth_split_info.side_min_width: ";
    ss9<<path_name<<'.'<< "smooth_split_info.before_split_max_width: ";
    ss10 <<path_name<<'.'<< "smooth_split_info.before_split_min_width: ";
    ss11<<path_name<<'.'<< "smooth_split_info.link_lenth: ";
    ss12 <<path_name<<'.'<< "smooth_split_info.side_link_length: ";
    ss13 <<path_name<<'.'<< "smooth_split_info.before_link_length: ";
    ss14 <<path_name<<'.'<< "smooth_split_info.left_line: ";
    ss15<<path_name<<'.'<< "smooth_split_info.right_line: ";
    ss16<<path_name<<'.' << "smooth_split_info.side_left_line: ";
    ss17 <<path_name<<'.'<< "smooth_split_info.side_right_line: ";
    ss18 <<path_name<<'.'<< "smooth_split_info.lane_curvpoints: ";
    ss19 <<path_name<<'.'<< "smooth_split_info.before_lane_curvpoints: ";
    ss20 <<path_name<<'.'<< "smooth_split_info.speed_limit: ";
    ss21 <<path_name<<'.'<< "smooth_split_info.side_speed_limit: ";
    ss22 <<path_name<<'.'<< "smooth_split_info.before_speed_limit: ";
    ss23 <<path_name<<'.'<< "smooth_split_info.lane_type: ";
    ss24 <<path_name<<'.'<< "smooth_split_info.side_lane_type: ";
    ss25 <<path_name<<'.'<< "smooth_split_info.before_lane_type: ";
    for(auto info : smooth_split_info){
        ss<<" ,"<< info.dist;
        ss26<<" ,"<< (int)info.road_class;
        for(auto id:info.link_id){
            ss1<<" ,"<<id;
        }
        ss1<<" ||||";
        for(auto id:info.lane_id){
            ss2<<" ,"<<(int)id;
        }
        ss2<<" ||||";
        for(auto id:info.side_link_id){
            ss3<<" ,"<<id;
        }
        ss3<<" ||||";
        for(auto id:info.side_lane_id){
            ss4<<" ,"<<(int)id;
        }
        ss4<<" ||||";
        for(auto id:info.max_width){
            ss5<<" ,"<<id;
        }
        ss5<<" ||||";
        for(auto id:info.side_max_width){
            ss6<<" ,"<<id;
        }
        ss6<<" ||||";
        for(auto id:info.min_width){
            ss7<<" ,"<<id;
        }
        ss7<<" ||||";
        for(auto id:info.side_min_width){
            ss8<<" ,"<<id;
        }
        ss8<<" ||||";
        for(auto id:info.before_split_max_width){
            ss9<<" ,"<<id;
        }
        ss9<<" ||||";
        for(auto id:info.before_split_min_width){
            ss10<<" ,"<<id;
        }
        ss10<<" ||||";
        for(auto id:info.link_length){
            ss11<<" ,"<<id;
        }
        ss11<<" ||||";
        for(auto id:info.side_link_length){
            ss12<<" ,"<<id;
        }
        ss12<<" ||||";
        for(auto id:info.before_link_length){
            ss13<<" ,"<<id;
        }
        ss13<<" ||||";
        for(auto id:info.left_line){
            ss14<<" ,"<<(int)id;
        }
        ss14<<" ||||";
        for(auto id:info.right_line){
            ss15<<" ,"<<(int)id;
        }
        ss15<<" ||||";
        for(auto id:info.side_left_line){
            ss16<<" ,"<<(int)id;
        }
        ss16<<" ||||";
        for(auto id:info.side_right_line){
            ss17<<" ,"<<(int)id;
        }
        ss17<<" ||||";
        for(auto id:info.lane_curvpoints){
            for(auto cuve:id){
                ss18<<" ,<"<<cuve.second<<" ,"<<cuve.first<<">";
            }
            ss18<< "||";
        }
        ss18<<" ||||";
        for(auto id:info.before_lane_curvpoints){
            for(auto cuve:id){
                ss19<<" ,<"<<cuve.second<<" ,"<<cuve.first<<">";
            }
            ss19<< "||";
        }
        for(auto id:info.speed_limit){
            ss20<<" ,"<<(int)id;
        }
        ss20<<" ||||";
        for(auto id:info.side_speed_limit){
            ss21<<" ,"<<(int)id;
        }
        ss21<<" ||||";
        for(auto id:info.before_speed_limit){
            ss22<<" ,"<<(int)id;
        }
        ss22<<" ||||";
        for(auto id:info.lane_type){
            ss23<<" ,"<<(int)id;
        }
        ss23<<" ||||";
        for(auto id:info.side_lane_type){
            ss24<<" ,"<<(int)id;
        }
        ss24<<" ||||";
        for(auto id:info.before_lane_type){
            ss25<<" ,"<<(int)id;
        }
        ss25<<" ||||";
    }
    std::cout << ss.str() << std::endl;
    std::cout << ss26.str() << std::endl;
    std::cout << ss1.str() << std::endl;
    std::cout << ss2.str() << std::endl;
    std::cout << ss3.str() << std::endl;
    std::cout << ss4.str() << std::endl;
    std::cout << ss5.str() << std::endl;
    std::cout << ss6.str() << std::endl;
    std::cout << ss7.str() << std::endl;
    std::cout << ss8.str() << std::endl;
    std::cout << ss9.str() << std::endl;
    std::cout << ss10.str() << std::endl;
    std::cout << ss11.str() << std::endl;
    std::cout << ss12.str() << std::endl;
    std::cout << ss13.str() << std::endl;
    std::cout << ss14.str() << std::endl;
    std::cout << ss15.str() << std::endl;
    std::cout << ss16.str() << std::endl;
    std::cout << ss17.str() << std::endl;
    std::cout << ss18.str() << std::endl;
    std::cout << ss19.str() << std::endl;
    std::cout << ss20.str() << std::endl;
    std::cout << ss21.str() << std::endl;
    std::cout << ss22.str() << std::endl;
    std::cout << ss23.str() << std::endl;
    std::cout << ss24.str() << std::endl;
    std::cout << ss25.str() << std::endl;
    return;
}
}  // namespace framework
}  // namespace shell
}  // namespace earth
