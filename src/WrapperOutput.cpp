#include "WrapperOutput.h"

namespace NoMapEFM{

bool WrapperOutput::Execute(const BevLaneElementGroupSet& bev_lane_group_set, const SDLaneElementGroupSet& EHP_ele_group, const SEhpOutputLinkList& link_list, 
                            const LaneIdx& EgoLaneIdx,const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx,
                            const LaneIdx& RightRightLaneIdx, const SEhpOutputLoc& loc, const LaneIdx& RefLaneIdx, const NodeInfo& node, const SEhpOutputPathList& paths,
                            LocJudgeInfo loc_res_info, datatype_efm::s_MapLane_t& map_lane) {
    map_lane.efm_info.mRampInfo.startPoint.dX = 10000;
    map_lane.efm_info.mRampInfo.endPoint.dX = 10000;
    InitMapLane(map_lane);
    MakeMapLane(bev_lane_group_set, EHP_ele_group, EgoLaneIdx, LeftLaneIdx, RightLaneIdx, LeftLeftLaneIdx, RightRightLaneIdx, loc,
                RefLaneIdx, link_list, paths, map_lane);
    //std::cout << __FILE__ << "," << __LINE__ << "," << "map_lane.Array_lanes_5[2]:enable_flag: " << (int)map_lane.Array_lanes_5[2].enable_flag<<std::endl; 
    //PlotOneLane("final_ego_lane", map_lane.Array_lanes_5[2]);
    MakeLppInfo(EHP_ele_group,paths,loc, link_list, EgoLaneIdx,LeftLaneIdx, RightLaneIdx, node, loc_res_info, map_lane);
    
    SEhpOutputLoc loc_tmp=loc;
    // PostProcess(map_lane, loc_tmp);
    return true;
}

bool WrapperOutput::MakeMapLane(const BevLaneElementGroupSet& bev_lane_group_set, const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx,
                                const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx,
                                const LaneIdx& RightRightLaneIdx, const SEhpOutputLoc& loc, const LaneIdx& RefLaneIdx,const SEhpOutputLinkList& link_list,
                                const SEhpOutputPathList& paths, datatype_efm::s_MapLane_t& map_lane) {
#ifdef WOUT 
    // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ids_map.size(): " << lane_ids_map.size()<<std::endl;    
    // for(auto & iter: lane_ids_map){
    //     std::cout<<" lane_iter ,";
    //     for(auto & iter_sub: iter.second){
    //         std::cout<<",||,"<<iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
    //         <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;            
    //     }
    // }
    std::cout << __FILE__ << "," << __LINE__ << "," << "EgoLaneIdx.bev_lane_idx_.first: " << (int)EgoLaneIdx.bev_lane_idx_.first <<" ,bev_lane_group_set.size():"<<bev_lane_group_set.size()<<std::endl;    
    // std::cout << __FILE__ << "," << __LINE__ << "," << "last_loc_lane_id: "<<last_loc_lane_id <<" ,current ego lane id: "<< loc.id_<<std::endl;                               
#endif
    // ego_lane
    // std::cout << __FILE__ << "," << __LINE__ << "," << "make ego lane: "<<std::endl;
    if (EgoLaneIdx.bev_lane_idx_.first >= 0 && EgoLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        const BevLaneElementGroup& bev_ele_group = bev_lane_group_set[EgoLaneIdx.bev_lane_idx_.first].second;
        if (EgoLaneIdx.bev_lane_idx_.second >= 0 && EgoLaneIdx.bev_lane_idx_.second < bev_ele_group.size()) {
            const BevLaneElement& lane_ele = bev_ele_group[EgoLaneIdx.bev_lane_idx_.second];
#ifdef WOUT 
    // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ids_map.size(): " << lane_ids_map.size()<<std::endl;    
    // for(auto & iter: lane_ids_map){
    //     std::cout<<" lane_iter ,";
    //     for(auto & iter_sub: iter.second){
    //         std::cout<<",||,"<<iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
    //         <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;            
    //     }
    // }
    std::cout << __FILE__ << "," << __LINE__ << "," << "EgoLaneIds left: " <<" ,"<<lane_ele.left_back_connect_id<<" ,"<<lane_ele.left_front_connect_id<<" ,"<<lane_ele.left_line_base_id<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "EgoLaneIds right: " <<" ,"<<" ,"<<lane_ele.right_back_connect_id<<" ,"<<lane_ele.right_front_connect_id<<" ,"<<lane_ele.right_line_base_id<<std::endl;   
#endif
            MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[2]);
            MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[2]);
            MakeLaneId(lane_ele,loc, map_lane.Array_lanes_5[2]);
            BevLaneIds lane_ids(lane_ele.left_line_base_id, lane_ele.left_back_connect_id, lane_ele.left_front_connect_id,
                                lane_ele.right_line_base_id, lane_ele.right_back_connect_id, lane_ele.right_front_connect_id);            
            last_ego_lane_info = lane_ids;
        }
    }

//    if (0 < bev_lane_group_set.size()) {
//        for(auto & group:bev_lane_group_set){
//            if(group.first == 0){
//                std::cout << __FILE__ << "," << __LINE__ << "," << "get ego lane group"<<std::endl;
//                const BevLaneElementGroup& bev_ele_group = group.second;
//                if (0 < bev_ele_group.size()) {
//                    std::cout << __FILE__ << "," << __LINE__ << "," << "get ego lane!!!"<<std::endl;
//                    const BevLaneElement& lane_ele = bev_ele_group[0];
//                    MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[2]);
//                    MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[2]);
//                }
//            }
//        }
//    }
//    std::cout << __FILE__ << "," << __LINE__ << "," << "end ego lane!!!"<<std::endl;
#ifdef WOUT 
    std::cout << __FILE__ << "," << __LINE__ << "," << "LeftLaneIdx.bev_lane_idx_.first: " << (int)LeftLaneIdx.bev_lane_idx_.first <<std::endl; 
#endif
    // left_lane
    if (LeftLaneIdx.bev_lane_idx_.first >= 0 && LeftLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        const BevLaneElementGroup& bev_ele_group = bev_lane_group_set[LeftLaneIdx.bev_lane_idx_.first].second;
        if (LeftLaneIdx.bev_lane_idx_.second >= 0 && LeftLaneIdx.bev_lane_idx_.second < bev_ele_group.size()) {
            const BevLaneElement& lane_ele = bev_ele_group[LeftLaneIdx.bev_lane_idx_.second];
            MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[1]);
            MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[1]);
            MakeLaneId(lane_ele, loc,map_lane.Array_lanes_5[1]);
        }
    }
#ifdef WOUT 
    std::cout << __FILE__ << "," << __LINE__ << "," << "RightLaneIdx.bev_lane_idx_.first: " << (int)RightLaneIdx.bev_lane_idx_.first <<std::endl; 
#endif
    // right_lane
    if (RightLaneIdx.bev_lane_idx_.first >= 0 && RightLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        const BevLaneElementGroup& bev_ele_group = bev_lane_group_set[RightLaneIdx.bev_lane_idx_.first].second;
        if (RightLaneIdx.bev_lane_idx_.second >= 0 && RightLaneIdx.bev_lane_idx_.second < bev_ele_group.size()) {
            const BevLaneElement& lane_ele = bev_ele_group[RightLaneIdx.bev_lane_idx_.second];
            MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[3]);
            MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[3]);
            MakeLaneId(lane_ele, loc,map_lane.Array_lanes_5[3]);
        }
    }
#ifdef WOUT 
    std::cout << __FILE__ << "," << __LINE__ << "," << "LeftLeftLaneIdx.bev_lane_idx_.first: " << (int)LeftLeftLaneIdx.bev_lane_idx_.first <<std::endl; 
#endif
    // left left_lane
    if (LeftLeftLaneIdx.bev_lane_idx_.first >= 0 && LeftLeftLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        const BevLaneElementGroup& bev_ele_group = bev_lane_group_set[LeftLeftLaneIdx.bev_lane_idx_.first].second;
        if (LeftLeftLaneIdx.bev_lane_idx_.second >= 0 && LeftLeftLaneIdx.bev_lane_idx_.second < bev_ele_group.size()) {
            const BevLaneElement& lane_ele = bev_ele_group[LeftLeftLaneIdx.bev_lane_idx_.second];
            MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[0]);
            MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[0]);
            MakeLaneId(lane_ele, loc,map_lane.Array_lanes_5[0]);
        }
    }
#ifdef WOUT 
    std::cout << __FILE__ << "," << __LINE__ << "," << "RightRightLaneIdx.bev_lane_idx_.first: " << (int)RightRightLaneIdx.bev_lane_idx_.first <<std::endl; 
#endif
    // right right_lane
    if (RightRightLaneIdx.bev_lane_idx_.first >= 0 &&
        RightRightLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        const BevLaneElementGroup& bev_ele_group = bev_lane_group_set[RightRightLaneIdx.bev_lane_idx_.first].second;
        if (RightRightLaneIdx.bev_lane_idx_.second >= 0 &&
            RightRightLaneIdx.bev_lane_idx_.second < bev_ele_group.size()) {
            const BevLaneElement& lane_ele = bev_ele_group[RightRightLaneIdx.bev_lane_idx_.second];
            MakeLaneCenterLine(lane_ele, map_lane.Array_lanes_5[4]);
            MakeLaneSideLine(lane_ele, map_lane.Array_lanes_5[4]);
            MakeLaneId(lane_ele, loc,map_lane.Array_lanes_5[4]);
        }
    }

    // isactive
    if (loc.link_id_ != 0 && loc.offset_ != 0 && loc.path_id_ != 0 && loc.lane_id_ != 0 &&
        map_lane.Array_lanes_5[2].enable_flag == true) {
        map_lane.bIsActive = true;
    } else {
        map_lane.bIsActive = false;
    }
    map_lane.efm_info.bIsActive = map_lane.bIsActive; // for EFMInfoMsg

    //ref 
    if (RefLaneIdx.bev_lane_idx_.first >= 0 && RefLaneIdx.bev_lane_idx_.first < bev_lane_group_set.size()) {
        if(bev_lane_group_set[RefLaneIdx.bev_lane_idx_.first].first == 1){
            map_lane.priorIndex = 1; // left
        }else if(bev_lane_group_set[RefLaneIdx.bev_lane_idx_.first].first == 2){
            map_lane.priorIndex = 2; 
        }else{
             map_lane.priorIndex = 0; 
        }
    }
    map_lane.efm_info.priorIndex = map_lane.priorIndex; // for EFMInfoMsg
    if(map_lane.priorIndex ==1){
        map_lane.efm_info.priorRefLine.pntSize= map_lane.Array_lanes_5[1].center_line.pntSize;
        for(int i= 0; i< map_lane.Array_lanes_5[1].center_line.pntSize; i++){
            map_lane.efm_info.priorRefLine.Array_PriorRefLine_121[i].dX = map_lane.Array_lanes_5[1].center_line.Array_linePnt_121[i].dX;
            map_lane.efm_info.priorRefLine.Array_PriorRefLine_121[i].dY = map_lane.Array_lanes_5[1].center_line.Array_linePnt_121[i].dY;
        }
    }

    MakePosition(loc, map_lane.Position);
    MakeMergeSplit(EHP_ele_group, EgoLaneIdx, LeftLaneIdx, RightLaneIdx, LeftLeftLaneIdx, RightRightLaneIdx, map_lane);
    MakeLaneType(EHP_ele_group, EgoLaneIdx, LeftLaneIdx, RightLaneIdx, LeftLeftLaneIdx, RightRightLaneIdx, map_lane);
    MakeCurvature(link_list, paths, loc, map_lane);
    return true;
}

bool WrapperOutput::MakeCurvature(const SEhpOutputLinkList& link_list, const SEhpOutputPathList& paths, const SEhpOutputLoc& loc, datatype_efm::s_MapLane_t& map_lane){
    const SEhpOutputPath *ego_path_ptr = nullptr;
    for(auto& path : paths.ehp_output_path_list){
        if(path.path_id_ == loc.path_id_ ){
            ego_path_ptr = &path;
        }
    }
    if(ego_path_ptr == nullptr){
        return false; // no path found
    }
    // curvature
    map_lane.Array_lanes_5[2].lane_attrs.curveSize = 0;
    map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200 = {};
    //
    for (const auto& link : link_list.ehp_output_link_list) {
        for(const auto& curv:link.curvs_){
            if(curv.offset_>=loc.offset_ && curv.offset_<=loc.offset_+30000 && map_lane.Array_lanes_5[2].lane_attrs.curveSize<200){
                map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200[map_lane.Array_lanes_5[2].lane_attrs.curveSize].offset = static_cast<double>(curv.offset_)/100.0 - static_cast<double>(loc.offset_)/100.0;
                map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200[map_lane.Array_lanes_5[2].lane_attrs.curveSize].curvature_value = curv.value_;
                map_lane.Array_lanes_5[2].lane_attrs.curveSize++;
            }
        }
    }

    map_lane.Array_lanes_5[1].lane_attrs.curveSize = map_lane.Array_lanes_5[2].lane_attrs.curveSize;
    map_lane.Array_lanes_5[1].lane_attrs.Array_curves_200 = map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200;

    map_lane.Array_lanes_5[0].lane_attrs.curveSize = map_lane.Array_lanes_5[2].lane_attrs.curveSize;
    map_lane.Array_lanes_5[0].lane_attrs.Array_curves_200 = map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200; 

    map_lane.Array_lanes_5[3].lane_attrs.curveSize = map_lane.Array_lanes_5[2].lane_attrs.curveSize;
    map_lane.Array_lanes_5[3].lane_attrs.Array_curves_200 = map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200;   

    map_lane.Array_lanes_5[4].lane_attrs.curveSize = map_lane.Array_lanes_5[2].lane_attrs.curveSize;
    map_lane.Array_lanes_5[4].lane_attrs.Array_curves_200 = map_lane.Array_lanes_5[2].lane_attrs.Array_curves_200;  
    return true;
}

bool WrapperOutput::MakeLaneType(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx,
                            const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx,
                            const LaneIdx& RightRightLaneIdx, datatype_efm::s_MapLane_t& map_lane){
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "EgoLaneIdx.sd_lane_idx_.first : "<<EgoLaneIdx.sd_lane_idx_.first<<" ,EgoLaneIdx.sd_lane_idx_.second:"<<EgoLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //ego
    MakeOneLaneType(EHP_ele_group, EgoLaneIdx, map_lane.Array_lanes_5[2]);
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "LeftLaneIdx.sd_lane_idx_.first : "<<LeftLaneIdx.sd_lane_idx_.first<<" ,LeftLaneIdx.sd_lane_idx_.second:"<<LeftLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //left
    MakeOneLaneType(EHP_ele_group, LeftLaneIdx, map_lane.Array_lanes_5[1]);
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "LeftLeftLaneIdx.sd_lane_idx_.first : "<<LeftLeftLaneIdx.sd_lane_idx_.first<<" ,LeftLeftLaneIdx.sd_lane_idx_.second:"<<LeftLeftLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //left left
    MakeOneLaneType(EHP_ele_group, LeftLeftLaneIdx, map_lane.Array_lanes_5[0]);
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "RightLaneIdx.sd_lane_idx_.first : "<<RightLaneIdx.sd_lane_idx_.first<<" ,RightLaneIdx.sd_lane_idx_.second:"<<RightLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //right
    MakeOneLaneType(EHP_ele_group, RightLaneIdx, map_lane.Array_lanes_5[3]);
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "RightRightLaneIdx.sd_lane_idx_.first : "<<RightRightLaneIdx.sd_lane_idx_.first<<" ,RightRightLaneIdx.sd_lane_idx_.second:"<<RightRightLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //right right
    MakeOneLaneType(EHP_ele_group, RightRightLaneIdx, map_lane.Array_lanes_5[4]);
    return true;
}

bool WrapperOutput::MakeOneLaneType(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& LaneIdx, datatype_efm::s_Lane_t& lane){
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," << "MakeOneLaneType : "<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," <<"LaneIdx.sd_lane_idx_.first:"<<LaneIdx.sd_lane_idx_.first<< " ,EHP_ele_group.size() : "<<EHP_ele_group.size()<<std::endl;
#endif
    if (LaneIdx.sd_lane_idx_.first >= 0 &&
        LaneIdx.sd_lane_idx_.first < EHP_ele_group.size()) {
        const SDLaneElementGroup& ele_group = EHP_ele_group[LaneIdx.sd_lane_idx_.first].second;
#ifdef MAKE_LANE_TYPE
    std::cout << __FILE__ << "," << __LINE__ << "," <<"LaneIdx.sd_lane_idx_.second:"<<LaneIdx.sd_lane_idx_.second<< " ,ele_group.size() : "<<ele_group.size()<<std::endl;
#endif
        if (LaneIdx.sd_lane_idx_.second >= 0 &&
            LaneIdx.sd_lane_idx_.second < ele_group.size()) {

            const SDLaneElement& lane_ele = ele_group[LaneIdx.sd_lane_idx_.second];
#ifdef MAKE_LANE_TYPE
                std::cout << __FILE__ << "," << __LINE__ << "," <<"lane num: ";
                for(auto& num: lane_ele.lane_nums){
                    std::cout<<(int)num<<",";
                }
        std::stringstream ss, ss1, ss2, ss3;
            ss<<"  lane_type: ";
            ss1<<" lane_merge: ";
            ss2<<" lane_split: ";
            ss3<<" lane_s_offset: ";
            for(auto & iter:lane_ele.lane_types){
                ss<<" ,"<<(int)iter;
            }
            for(auto & iter:lane_ele.lane_merges){
                ss1<<" ,"<<(int)iter;
            }
            for(auto & iter:lane_ele.lane_splits){
                ss2<<" ,"<<(int)iter;
            }
            for(auto & iter:lane_ele.lane_s_offsets){
                ss3<<" ,"<<(int)iter;
            }
            std::cout <<std::endl;
            std::cout<<ss.str()<<std::endl;
            std::cout<<ss1.str()<<std::endl;
            std::cout<<ss2.str()<<std::endl;
            std::cout<<ss3.str()<<std::endl;
                std::cout<<std::endl;
#endif
            // merge,split
            int lane_type_count = 0;
            for(int i = 0; i< lane_ele.lane_types.size() && i<lane_ele.lane_s_offsets.size(); i++){
                if(i==0){
                    lane.Array_types_3[lane_type_count].valid = true;
                    lane.Array_types_3[lane_type_count].type = static_cast<datatype_efm::e_EfmLaneType_t_ref>(LaneTypeMatch(lane_ele.lane_types[i]));
                    lane.Array_types_3[lane_type_count].s = static_cast<float>(lane_ele.lane_s_offsets[i]);
                    lane_type_count++;
                }else if(lane_type_count<3 && lane_type_count>0){
                    uint8_t cur_type = LaneTypeMatch(lane_ele.lane_types[i]);
                    if(cur_type != static_cast<uint8_t>(lane.Array_types_3[lane_type_count-1].type)){
                        lane.Array_types_3[lane_type_count].valid = true;
                        lane.Array_types_3[lane_type_count].type = static_cast<datatype_efm::e_EfmLaneType_t_ref>(cur_type);
                        lane.Array_types_3[lane_type_count].s = static_cast<float>(lane_ele.lane_s_offsets[i]);
                        lane_type_count++;
                    }
                }
            }
        }
    } 
    return true; 

}

bool WrapperOutput::MakePosition(const SEhpOutputLoc& loc, datatype_efm::s_EfmPosition_t& position) {
    position.dLat = static_cast<float>(loc.lat_);
    position.dLon = static_cast<float>(loc.lon_);
    position.dSpeed = static_cast<float>(loc.speed_);
    position.heading = static_cast<float>(loc.heading_);
    position.timeStamp = loc.time_stmp_;
    position.mapIsValid = true;
    return true;
}

bool WrapperOutput::MakeMergeSplit(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx,
                            const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx,
                            const LaneIdx& RightRightLaneIdx, datatype_efm::s_MapLane_t& map_lane){
    //ego
    MakeOneMergeSplit(EHP_ele_group, EgoLaneIdx, map_lane.Array_lanes_5[2]);
    //left
    MakeOneMergeSplit(EHP_ele_group, LeftLaneIdx, map_lane.Array_lanes_5[1]);
    //left left
    MakeOneMergeSplit(EHP_ele_group, LeftLeftLaneIdx, map_lane.Array_lanes_5[0]);
    //right
    MakeOneMergeSplit(EHP_ele_group, RightLaneIdx, map_lane.Array_lanes_5[3]);
    //right right
    MakeOneMergeSplit(EHP_ele_group, RightRightLaneIdx, map_lane.Array_lanes_5[4]);
    return true;
}

bool WrapperOutput::MakeOneMergeSplit(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& LaneIdx, datatype_efm::s_Lane_t& lane){
     if (LaneIdx.sd_lane_idx_.first >= 0 &&
        LaneIdx.sd_lane_idx_.first < EHP_ele_group.size()) {
        const SDLaneElementGroup& ele_group = EHP_ele_group[LaneIdx.sd_lane_idx_.first].second;
        if (LaneIdx.sd_lane_idx_.second >= 0 &&
            LaneIdx.sd_lane_idx_.second < ele_group.size()) {
            const SDLaneElement& lane_ele = ele_group[LaneIdx.sd_lane_idx_.second];
            // merge,split
            int merge_count = 0;
            int split_count = 0;
            for(int i = 0; i< lane_ele.lane_merges.size() && i<lane_ele.lane_splits.size() && i<lane_ele.lane_s_offsets.size() && i<lane_ele.lane_e_offsets.size(); i++){
                if(lane_ele.lane_merges[i] != EfmMergeType::EFM_MergeType_NONE && merge_count <3){
                    lane.Array_lane_merge_3[merge_count].dir = static_cast<datatype_efm::e_ChangeDirection_t_ref>(MergeTypeMatch(lane_ele.lane_merges[i]));
                    lane.Array_lane_merge_3[merge_count].valid = true;
                    lane.Array_lane_merge_3[merge_count].s_start = static_cast<float>(lane_ele.lane_s_offsets[i]);
                    lane.Array_lane_merge_3[merge_count].s_end = static_cast<float>(lane_ele.lane_e_offsets[i]);
                    merge_count++;
                }
                if(lane_ele.lane_splits[i] != EfmSplitType::EFM_SplitType_NONE && split_count <3){
                    lane.Array_lane_split_3[split_count].dir = static_cast<datatype_efm::e_ChangeDirection_t_ref>(MergeTypeMatch(lane_ele.lane_merges[i]));
                    lane.Array_lane_split_3[split_count].valid = true;
                    lane.Array_lane_split_3[split_count].s_start = static_cast<float>(lane_ele.lane_s_offsets[i]);
                    lane.Array_lane_split_3[split_count].s_end = static_cast<float>(lane_ele.lane_e_offsets[i]);
                    split_count++;
                }

            }
        }
    }   
    return true;
}

uint8_t WrapperOutput::MergeTypeMatch(uint8_t type_in) {
// enum EfmMergeType : uint8_t {
//     EFM_MergeType_NONE = 0,
//     EFM_MergeType_TO_LEFT = 1,
//     EFM_MergeType_FROM_LEFT = 2,
//     EFM_MergeType_LEFT_TO_MIDDLE = 3,
//     EFM_MergeType_TO_RIGHT = 4,
//     EFM_MergeType_FROM_RIGHT = 5,
//     EFM_MergeType_RIGHT_TO_MIDDLE = 6,
//     EFM_MergeType_TO_MIDDLE = 7
// };

//out
// const u8 DirUnknown_@1 = 0
// const u8 DirToLeft_@2 = 1
// const u8 DirFromLeft_@3 = 2
// const u8 DirMiddle_@4 = 3
// const u8 DirToRight_@5 = 4
// const u8 DirFromRight_@6 = 5
    switch (type_in) {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
            return type_in;
        case 3:
        case 6:
        case 7:
            return 3;
        default:
            return 0;  // LINE_UNKNOWN
    }
    return 0;
}

uint8_t WrapperOutput::SplitTypeMatch(uint8_t type_in) {
// enum EfmSplitType : uint8_t {
//     EFM_SplitType_NONE = 0,
//     EFM_SplitType_TO_LEFT = 1,
//     EFM_SplitType_FROM_LEFT = 2,
//     EFM_SplitType_CONTIUE_FROM_LEFT = 3,
//     EFM_SplitType_TO_RIGHT = 4,
//     EFM_SplitType_FROM_RIGHT = 5,
//     EFM_SplitType_SPLIT_FROM_LEFT = 6,
//     EFM_SplitType_CONTIUE_FROM_RIGHT = 7,
//     EFM_SplitType_SPLIT_FROM_RIGHT = 8
// } ;

//out
// const u8 DirUnknown_@1 = 0
// const u8 DirToLeft_@2 = 1
// const u8 DirFromLeft_@3 = 2
// const u8 DirMiddle_@4 = 3
// const u8 DirToRight_@5 = 4
// const u8 DirFromRight_@6 = 5
    switch (type_in) {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
            return type_in;
        case 3:
        case 6:
        case 7:
        case 8:
            return 3;
        default:
            return 0;  // LINE_UNKNOWN
    }
    return 0;
}

uint8_t WrapperOutput::LaneTypeMatch(uint8_t type_in) {
// enum LaneType : uint8_t {
//     EnumLaneType_NORMAL_ = 0,
//     EnumLaneType_ENTRY_ = 1,
//     EnumLaneType_EXIT_ = 2,
//     EnumLaneType_EMERGENCY_ = 3,
//     EnumLaneType_ON_RAMP_ = 4,
//     EnumLaneType_OFF_RAMP_ = 5,
//     EnumLaneType_CONNECT_RAMP_ = 6,
//     EnumLaneType_ACCELERATE_ = 7,
//     EnumLaneType_DECELERATE_ = 8,
//     EnumLaneType_EMERGENCY_PARKING_STRIP_ = 9,
//     EnumLaneType_RESERVE0_ = 10,
//     EnumLaneType_RESERVE1_ = 11,
//     EnumLaneType_RESERVE2_ = 12,
//     EnumLaneType_RESERVE3_ = 13,
//     EnumLaneType_RESERVE4_ = 14,
//     EnumLaneType_DIVERSION_ = 15,
//     EnumLaneType_RESERVE5_ = 16,
//     EnumLaneType_RESERVE6_ = 17,
//     EnumLaneType_RESERVE7_ = 18,
//     EnumLaneType_RESERVE8_ = 19,
//     EnumLaneType_RESERVE9_ = 20,
//     EnumLaneType_RESERVE10_ = 21
// } ;

//out
// const u8 Normal_@1 = 0
// const u8 Entra_@2 = 1
// const u8 Exit_@3 = 2
// const u8 Emergency_@4 = 3
// const u8 OnRamp_@5 = 4
// const u8 OffRamp_@6 = 5
// const u8 ConnectRamp_@7 = 6
// const u8 Accelerate_@8 = 7
// const u8 Decelerate_@9 = 8
// const u8 EmergemcyParkingStrip_@10 = 9
// const u8 Unknown_@11 = 10
// const u8 Passing_@12 = 11
// const u8 Bus_@13 = 12
// const u8 Hov_@14 = 13
// const u8 Uturn@15 = 14
// const u8 Crossing@16 = 15
// const u8 Mix_@17 = 16
// const u8 Danger_@18 = 17
// const u8 TurnWaiting_@19 = 18
// const u8 StraightWaiting_@20 = 19
// const u8 Bicycle_@21 = 20
// const u8 Sidewalk_@22 = 21
// const u8 Parking_@23 = 22
// const u8 Diversion_@24 = 23
// const u8 Shoulder_@25 = 24
// const u8 Supervising_@26 = 25
    switch (type_in) {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            return type_in;
        case 15:
            return 23;
        default:
            return 0;  // LINE_UNKNOWN
    }
    return 0;
}

bool WrapperOutput::MakeLaneCenterLine(const BevLaneElement& lane_ele, datatype_efm::s_Lane_t& lane) {
    EFMPoint ego_point(0.0, 0.0);
    EFMRefLinePointsSection marker_section{};
#ifdef WOUT
    std::cout << __FILE__ << "," << __LINE__ << "," << " lane_ele.center_line_points.size: " << lane_ele.center_line_points.size() << std::endl;
#endif

//     if (CommonTool::DiscretePointsMath::GetInstance()->SaperateLineIntoTwoPart(lane_ele.center_line_points, ego_point,
//                                                                                marker_section) == true) {
//         std::vector<EFMPoint> front_line = marker_section[1];
//         std::vector<EFMPoint> back_line = marker_section[0];
//         int line_pnt_num = 0;
// #ifdef WOUT
//         {
//         std::stringstream ssx, ssy;
//         ssx<<"center front_linex = [";
//         ssy<<"center front_liney = [";
//         for(int i=0;i< front_line.size();i++){
//             ssx<< front_line[i].x<<" ";
//             ssy<< front_line[i].y<<" ";
//         }
//         ssx<<"]"<<std::endl;
//         ssy<<"]"<<std::endl; 
//         std::cout<<ssx.str();   
//         std::cout<<ssy.str();    
//         }
//         {
//         std::stringstream ssx, ssy;
//         ssx<<"center back_linex = [";
//         ssy<<"center back_liney = [";
//         for(int i=0;i< back_line.size();i++){
//             ssx<< back_line[i].x<<" ";
//             ssy<< back_line[i].y<<" ";
//         }
//         ssx<<"]"<<std::endl;
//         ssy<<"]"<<std::endl; 
//         std::cout<<ssx.str();   
//         std::cout<<ssy.str();    
//         }
// #endif
//         // fill back line
//         int back_size = std::min(static_cast<int>(back_line.size()), 41);
//         if (back_size <= 0) {
//             // no bake line
//         } else {
//             // not include ego point !!!! 所以不用[0]点
//             std::vector<EFMPoint> line_tmp{};
//             std::reverse(back_line.begin(), back_line.end());
//             line_tmp.assign(back_line.begin(), back_line.begin()+back_size);
//             std::reverse(line_tmp.begin(), line_tmp.end());
//             for (int i = 0; i < line_tmp.size() && line_pnt_num < lane.center_line.Array_linePnt_121.size();i++) {
//                 lane.center_line.Array_linePnt_121[line_pnt_num].dX = static_cast<float>(line_tmp[i].x);
//                 lane.center_line.Array_linePnt_121[line_pnt_num].dY = static_cast<float>(line_tmp[i].y);
//                 line_pnt_num++;
//             }
//         }
//         if (line_pnt_num > 0) {
//             lane.center_line.egoIndex = line_pnt_num - 1;
//         }
//         int back_point_num = line_pnt_num;
//         // std::cout << __FILE__ << "," << __LINE__ << "," << " back side_line_pnt_num: " << line_pnt_num << std::endl;
//         // std::cout << __FILE__ << "," << __LINE__ << "," << " side_line.egoIndex: " <<
//         // 前后两段线的[0]点都是一个点
//         for (int i = 1; i < front_line.size() && line_pnt_num < lane.center_line.Array_linePnt_121.size(); i++) {
//             lane.center_line.Array_linePnt_121[line_pnt_num].dX = static_cast<float>(front_line[i].x);
//             lane.center_line.Array_linePnt_121[line_pnt_num].dY = static_cast<float>(front_line[i].y);
//             line_pnt_num++;
//             if ((line_pnt_num - back_point_num) >= 61) {
//                 break;
//             }
//         }

//         lane.center_line.pntSize = line_pnt_num;
//         if (lane.center_line.pntSize > 0) {
//             lane.enable_flag = true;
//         }
// #ifdef WOUT
//         {
//         std::stringstream ssx, ssy;
//         ssx<<"center linex = [";
//         ssy<<"center liney = [";
//         for(int i=0;i< lane.center_line.pntSize;i++){
//             ssx<< lane.center_line.Array_linePnt_121[i].dX<<" ";
//             ssy<< lane.center_line.Array_linePnt_121[i].dY<<" ";
//         }
//         ssx<<"]"<<std::endl;
//         ssy<<"]"<<std::endl; 
//         std::cout<<ssx.str();   
//         std::cout<<ssy.str();    
//         }
// #endif
//     }else{
        int line_pnt_num = 0;
        for (int i = 0; i < lane_ele.center_line_points.size() && line_pnt_num < lane.center_line.Array_linePnt_121.size();i++) {
            lane.center_line.Array_linePnt_121[line_pnt_num].dX = static_cast<float>(lane_ele.center_line_points[i].x);
            lane.center_line.Array_linePnt_121[line_pnt_num].dY = static_cast<float>(lane_ele.center_line_points[i].y);
            line_pnt_num++;
        } 
        lane.center_line.pntSize = line_pnt_num;
        if (lane.center_line.pntSize > 0) {
            lane.enable_flag = true;
        }  
#ifdef WOUT
        {
        std::stringstream ssx, ssy;
        ssx<<"center linex = [";
        ssy<<"center liney = [";
        for(int i=0;i< lane.center_line.pntSize;i++){
            ssx<< lane.center_line.Array_linePnt_121[i].dX<<" ";
            ssy<< lane.center_line.Array_linePnt_121[i].dY<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }
#endif     
    // }
    return true;
}

bool WrapperOutput::MakeSideLineTypeSinglePart(const std::vector<LineTypeInfo>& line_types, datatype_efm::s_LineMkr_t& side_line){
    side_line.Array_etype_10.front().valid = true;
    side_line.Array_etype_10.front().s = 0.0f;
    side_line.Array_etype_10.front().type = static_cast<datatype_efm::e_LineType_t_ref>(0u);
#ifdef WOUT
    for(auto& sec: line_types){
        std::cout << __FILE__ << "," << __LINE__ << "," << ".line_types: " 
           << " ,[ valid: " << sec.is_valid
           << " , start_p:" << sec.start_point
           << " ,type: " << (int)sec.line_type
           << " ,change p: " << sec.typ_chg_point 
           <<" , type_af_change: "<< (int)sec.typ_aft_chg_point <<" ]"<<std::endl;
    }
#endif
    std::vector<SingleLineTypeInfoS> line_type_sections{};
    for(int i =0; i< line_types.size(); i++){
        if(line_types[i].is_valid == true ){
            SingleLineTypeInfoS section;
            if(line_type_sections.size()>0){
                line_type_sections.back().end_point = static_cast<float>(line_types[i].start_point);
            }
            section.start_point = static_cast<float>(line_types[i].start_point);
            section.line_type = line_types[i].line_type;
            if(line_types[i].typ_aft_chg_point == line_types[i].line_type){
                section.end_point = INFINITY;
                line_type_sections.push_back(section);
            }else{
                section.end_point = static_cast<float>(line_types[i].typ_chg_point);
                line_type_sections.push_back(section);
                section.start_point = static_cast<float>(line_types[i].typ_chg_point);
                section.line_type = line_types[i].typ_aft_chg_point;
                section.end_point = INFINITY;
                line_type_sections.push_back(section);
            }   
        }
    }
#ifdef WOUT
    for(auto& sec: line_type_sections){
        std::cout << __FILE__ << "," << __LINE__ << "," << ".line_type_sections: " 
           << " ,[ start_p:" << sec.start_point
           << " , end_p:" << sec.end_point
           << " ,type: " << (int)sec.line_type
            <<" ]"<<std::endl; 
    }
#endif
    //找到10m处的type
    float query_s = 10.0f;
    for(int i =0; i< line_type_sections.size(); i++){
        if(query_s >= line_type_sections[i].start_point && query_s < line_type_sections[i].end_point){
            side_line.Array_etype_10.front().type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(line_type_sections[i].line_type));
            break;
        }
    }
    return true;
}

bool WrapperOutput::MakeLaneSideLine(const BevLaneElement& lane_ele, datatype_efm::s_Lane_t& lane) {
    // EFMPoint ego_point(0.0, 0.0);
    // EFMRefLinePointsSection marker_section{};
    // if (CommonTool::DiscretePointsMath::GetInstance()->SaperateLineIntoTwoPart(lane_ele.left_line_points, ego_point,
    //                                                                            marker_section) == true) {                                                                        
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ele.left_types.size(): "<<lane_ele.left_types.size()<<std::endl;
            std::stringstream ss;
            ss <<".left_types: ";
            // ss1 << "map_lane.lanes[1].left_line.eColor: ";
            // ss2 << "map_lane.lanes[1].left_line.virtual_line_s: ";
            for (int i = 0; i < lane_ele.left_types.size(); i++) {
                ss << " ,[valid: " << lane_ele.left_types[i].is_valid
                << " , start_p:" << lane_ele.left_types[i].start_point
                << " ,type: " << (int)lane_ele.left_types[i].line_type
                << " ,change p: " << lane_ele.left_types[i].typ_chg_point 
                <<" , type_af_change: "<< (int)lane_ele.left_types[i].typ_aft_chg_point <<" ]";
            }
            std::cout<<ss.str()<<std::endl;
#endif
    //     MakeSideLinePoint(marker_section, lane.left_line);
    // }else{
        MakeSideLinePointBeyondEgo(lane_ele.left_line_points, lane.left_line);       
    // }

    {//type
        if(lane_ele.left_base_line_ptr!= nullptr){
            lane.left_line.MdlQly = lane_ele.left_base_line_ptr->md_qly;
        }
        //只做自车前方的10m处的type， 放到[0]处
        MakeSideLineTypeSinglePart(lane_ele.left_types, lane.left_line);
        // LineTypeInfo left_type_first, left_type_sec;
        // if(lane_ele.left_types.size() ==3){
        //     left_type_first = lane_ele.left_types[1];
        //     left_type_sec = lane_ele.left_types[2];
        // }else if(lane_ele.left_types.size() ==2){
        //     left_type_first = lane_ele.left_types[0];
        //     left_type_sec = lane_ele.left_types[1];
        // }else if(lane_ele.left_types.size() ==1){
        //     left_type_first = lane_ele.left_types[0];
        //     left_type_sec.is_valid = false; // no second type
        // }
        // MakeSideLineType(left_type_first, left_type_sec, lane.left_line);
    }

//     if (CommonTool::DiscretePointsMath::GetInstance()->SaperateLineIntoTwoPart(lane_ele.right_line_points, ego_point,
//                                                                               marker_section) == true) {
// #ifdef WOUT
//             std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ele.right_types.size(): "<<lane_ele.right_types.size()<<std::endl;
//             std::stringstream ss;
//             ss <<".right_types: ";
//             // ss1 << "map_lane.lanes[1].left_line.eColor: ";
//             // ss2 << "map_lane.lanes[1].left_line.virtual_line_s: ";
//             for (int i = 0; i < lane_ele.right_types.size(); i++) {
//                 ss << " ,[valid: " << lane_ele.right_types[i].is_valid
//                 << " , start_p:" << lane_ele.right_types[i].start_point
//                 << " ,type: " << (int)lane_ele.right_types[i].line_type
//                 << " ,change p: " << lane_ele.right_types[i].typ_chg_point 
//                 <<" , type_af_change: "<< (int)lane_ele.right_types[i].typ_aft_chg_point <<" ]";
//             }
//             std::cout<<ss.str()<<std::endl;
// #endif
                                                                                
//         MakeSideLinePoint(marker_section, lane.right_line);
        
//     }else{
        MakeSideLinePointBeyondEgo(lane_ele.right_line_points, lane.right_line);       
    // }
    
    {//type
        if(lane_ele.right_base_line_ptr!= nullptr){
            lane.right_line.MdlQly = lane_ele.right_base_line_ptr->md_qly;
        } 
        MakeSideLineTypeSinglePart(lane_ele.right_types, lane.right_line);
        // LineTypeInfo right_type_first, right_type_sec;
        // if(lane_ele.right_types.size() ==3){
        //     right_type_first = lane_ele.right_types[1];
        //     right_type_sec = lane_ele.right_types[2];
        // }else if(lane_ele.right_types.size() ==2){
        //     right_type_first = lane_ele.right_types[0];
        //     right_type_sec = lane_ele.right_types[1];
        // }else if(lane_ele.right_types.size() ==1){
        //     right_type_first = lane_ele.right_types[0];
        //     right_type_sec.is_valid = false; // no second type
        // } 
        // MakeSideLineType(right_type_first, right_type_sec, lane.right_line);
    }

    if(lane.right_line.bIsAvailable|| lane.left_line.bIsAvailable){
        lane.enable_flag = true;
    }

    return true;
}

bool WrapperOutput::MakeSideLinePointBeyondEgo(const EFMRefLinePoints& marker_points,
                                      datatype_efm::s_LineMkr_t& side_line){
    int pnt_num = 0;
    for (int i = 0; i < marker_points.size() && pnt_num < side_line.Array_linePoints_121.size(); i++) {
        side_line.Array_linePoints_121[pnt_num].dX = static_cast<float>(marker_points[i].x);
        side_line.Array_linePoints_121[pnt_num].dY = static_cast<float>(marker_points[i].y);
        pnt_num++;
    }
    side_line.pntSize = pnt_num;
    if (side_line.pntSize > 0) {
        side_line.bIsAvailable = true;
    }
// #ifdef WOUT
//         {
//         std::stringstream ssx, ssy;
//         ssx<<"side linex = [";
//         ssy<<"side liney = [";
//         for(int i=0;i< side_line.pntSize;i++){
//             if(i == side_line.pntSize.size()-1){
//                 ssx<< side_line.Array_linePoints_121[i].dX;
//                 ssy<< side_line.Array_linePoints_121[i].dY
//             }else{
//                 ssx<< side_line.Array_linePoints_121[i].dX<<", ";
//                 ssy<< side_line.Array_linePoints_121[i].dY<<", ";
//             } 
//         }
//         ssx<<"]"<<std::endl;
//         ssy<<"]"<<std::endl; 
//         std::cout<<ssx.str();   
//         std::cout<<ssy.str();    
//         }
// #endif  
    return true;
}

bool WrapperOutput::MakeSideLinePoint(const EFMRefLinePointsSection& marker_section,
                                      datatype_efm::s_LineMkr_t& side_line) {
    std::vector<EFMPoint> front_line = marker_section[1];
    std::vector<EFMPoint> back_line = marker_section[0];
#ifdef WOUT
        {
        std::stringstream ssx, ssy;
        ssx<<"side_front_linex = [";
        ssy<<"side_front_liney = [";
        for(int i=0;i< front_line.size();i++){
            ssx<< front_line[i].x<<" ";
            ssy<< front_line[i].y<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }
        {
        std::stringstream ssx, ssy;
        ssx<<"side_back_linex = [";
        ssy<<"side_back_liney = [";
        for(int i=0;i< back_line.size();i++){
            ssx<< back_line[i].x<<" ";
            ssy<< back_line[i].y<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }
#endif
    int line_pnt_num = 0;

    // fill back line
    int back_size = std::min(static_cast<int>(back_line.size()), 41);
    if (back_size <= 0) {
        // no bake line
    } else {
        // not include ego point !!!! 所以不用[0]点
        //for (int i = back_size - 1; i > 0 && i < back_line.size() && line_pnt_num < side_line.Array_linePoints_121.size(); i--) {
        std::vector<EFMPoint> line_tmp{};
        std::reverse(back_line.begin(), back_line.end());
        line_tmp.assign(back_line.begin(), back_line.begin()+back_size);
        std::reverse(line_tmp.begin(), line_tmp.end());
        for(int i =0; i < line_tmp.size() && line_pnt_num < side_line.Array_linePoints_121.size(); i++){
            side_line.Array_linePoints_121[line_pnt_num].dX = static_cast<float>(line_tmp[i].x);
            side_line.Array_linePoints_121[line_pnt_num].dY = static_cast<float>(line_tmp[i].y);
            line_pnt_num++;
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
    for (int i = 1; i < front_line.size() && line_pnt_num < side_line.Array_linePoints_121.size(); i++) {
        side_line.Array_linePoints_121[line_pnt_num].dX = static_cast<float>(front_line[i].x);
        side_line.Array_linePoints_121[line_pnt_num].dY = static_cast<float>(front_line[i].y);
        line_pnt_num++;
        if ((line_pnt_num - back_point_num) >= 61) {
            break;
        }
    }

    side_line.pntSize = line_pnt_num;
    if (side_line.pntSize > 0) {
        side_line.bIsAvailable = true;
    }

#ifdef WOUT
        {
        std::stringstream ssx, ssy;
        ssx<<"side linex = [";
        ssy<<"side liney = [";
        for(int i=0;i< side_line.pntSize;i++){
            ssx<< side_line.Array_linePoints_121[i].dX<<" ";
            ssy<< side_line.Array_linePoints_121[i].dY<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }
#endif
    return true;
}

bool WrapperOutput::MakeSideLineType(const LineTypeInfo& type_first, const LineTypeInfo& type_sec,
                                     datatype_efm::s_LineMkr_t& side_line) {
#ifdef WOUT
    std::stringstream ss, ss2;
    ss <<".type_first: ";
        ss << " ,[valid: " << type_first.is_valid
           << " , start_p:" << type_first.start_point
           << " ,type: " << (int)type_first.line_type
           << " ,change p: " << type_first.typ_chg_point 
           <<" , type_af_change: "<< (int)type_first.typ_aft_chg_point <<" ]"; 
           if(type_first.typ_chg_point>10000){
            std::cout << __FILE__ << "," << __LINE__ << "," << "type_first.typ_chg_point: "<<type_first.typ_chg_point<<" >10000"<<std::endl;
           } else{
            std::cout << __FILE__ << "," << __LINE__ << "," << "type_first.typ_chg_point: "<<type_first.typ_chg_point<<" <=10000"<<std::endl;
           } 
    std::cout<<ss.str()<<std::endl;     
    ss2 <<".type_sec: ";
        ss2 << " ,[valid: " << type_sec.is_valid
           << " , start_p:" << type_sec.start_point
           << " ,type: " << (int)type_sec.line_type
           << " ,change p: " << type_sec.typ_chg_point 
           <<" , type_af_change: "<< (int)type_sec.typ_aft_chg_point <<" ]";   
    std::cout<<ss2.str()<<std::endl; 
#endif
    if (type_first.is_valid && !type_sec.is_valid) {
        if (type_first.typ_chg_point > 10000) {  // 只有一段
#ifdef WOUT
	    std::cout << __FILE__ << "," << __LINE__ << "," << "type_first 只有一段case1: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = 0;
        } else {  // 两段
#ifdef WOUT
        std::cout << __FILE__ << "," << __LINE__ << "," << "type_first 两段 case1: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_first.typ_chg_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.typ_aft_chg_point));
            side_line.Array_etype_10[1].s = 200;
        }
    } else if (type_first.is_valid && type_sec.is_valid) {
        if (type_first.typ_chg_point > 10000 && type_sec.typ_chg_point > 10000 &&
            type_first.line_type == type_sec.line_type) {  // 只有一段
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "," << "整个 一段case2: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = 0;
        }else if (type_first.typ_chg_point > 10000 && type_sec.typ_chg_point < 10000 &&
                   type_first.line_type == type_sec.line_type) {  // 两段
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "," << "整个 2段: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_sec.typ_chg_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.typ_aft_chg_point));
            side_line.Array_etype_10[1].s = 200;
        }else if (type_first.typ_chg_point > 10000 && type_sec.typ_chg_point > 10000 &&
                   type_first.line_type != type_sec.line_type){
#ifdef WOUT
                   std::cout << __FILE__ << "," << __LINE__ << "," << "整个 2段 case2: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_sec.start_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.typ_aft_chg_point));
            side_line.Array_etype_10[1].s = 200;
        }else if(type_first.typ_chg_point < 10000 && type_sec.typ_chg_point > 10000 &&
                   type_first.typ_aft_chg_point == type_sec.line_type){
#ifdef WOUT
                std::cout << __FILE__ << "," << __LINE__ << "," << "整个 2段 case3: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_first.typ_chg_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.line_type));
            side_line.Array_etype_10[1].s = 200;
        }else if (type_first.typ_chg_point > 10000 && type_sec.typ_chg_point < 10000 &&
                   type_first.line_type != type_sec.line_type) {  // 三段
#ifdef WOUT
                   std::cout << __FILE__ << "," << __LINE__ << "," << "整个 3段 case1: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_sec.start_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.line_type));
            side_line.Array_etype_10[1].s = type_sec.typ_chg_point;

            side_line.Array_etype_10[2].valid = true;
            side_line.Array_etype_10[2].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.typ_aft_chg_point));
            side_line.Array_etype_10[2].s = 200;
        }else if (type_first.typ_chg_point < 10000 && type_sec.typ_chg_point > 10000 &&
                   type_first.typ_aft_chg_point != type_sec.line_type) {  // 三段
#ifdef WOUT
                   std::cout << __FILE__ << "," << __LINE__ << "," << "整个 3段 case2: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_first.typ_chg_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.typ_aft_chg_point));
            side_line.Array_etype_10[1].s = type_sec.typ_chg_point;

            side_line.Array_etype_10[2].valid = true;
            side_line.Array_etype_10[2].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.line_type));
            side_line.Array_etype_10[2].s = 200;
        } else {
            // 四段
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "," << "整个 4段: "<<std::endl;
#endif
            side_line.Array_etype_10[0].valid = true;
            side_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.line_type));
            side_line.Array_etype_10[0].s = type_first.typ_chg_point;

            side_line.Array_etype_10[1].valid = true;
            side_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_first.typ_aft_chg_point));
            side_line.Array_etype_10[1].s = type_sec.start_point;

            side_line.Array_etype_10[2].valid = true;
            side_line.Array_etype_10[2].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.line_type));
            side_line.Array_etype_10[2].s = type_sec.typ_chg_point;

            side_line.Array_etype_10[3].valid = true;
            side_line.Array_etype_10[3].type = static_cast<datatype_efm::e_LineType_t_ref>(LineTypeMatch(type_sec.typ_aft_chg_point));
            side_line.Array_etype_10[3].s = 200;
        }
    }
    return true;
}

uint8_t WrapperOutput::LineTypeMatch(int bev_line_type) {
    // enum BstdsLineType :uint8_t{
    //   LINE_UNKNOWN = 0,
    //   LINE_SOLID = 1,
    //   LINE_DASHED = 2,
    //   LINE_DOUBLESOLID = 3,
    //   LINE_DOUBLEDASHED = 4,
    //   LINE_SOLIDDASHED = 5, /**< zuo实you虚 */
    //   LINE_DASHEDSOLID = 6, /**< zuo虚you实 */
    //   LINE_FISHBONE = 7,
    //   LINE_TEMPORARY = 8,
    //   LINE_CURB = 9,
    //   LINE_CONE = 10
    // };

    // struct e_LineType_t {
    // using MantlePro = ::std::true_type;
    // uint8_t data_;
    // static constexpr uint8_t LineType_Unknow_ = 0;
    // static constexpr uint8_t LineType_None_ = 1;
    // static constexpr uint8_t LineType_SolidLine_ = 2;
    // static constexpr uint8_t LineType_DashedLine_ = 3;
    // static constexpr uint8_t LineType_DoubleSoildLine_ = 4;
    // static constexpr uint8_t LineType_DoubleDashedLine_ = 5;
    // static constexpr uint8_t LineType_LeftSolidRightDashed_ = 6;
    // static constexpr uint8_t LineType_RightSoildLeftDashed_ = 7;
    // static constexpr uint8_t LineType_Virtual_ = 8;
    switch (bev_line_type) {
        case 1:
            return 2;
        case 2:
            return 3;
        case 3:
            return 4;
        case 4:
            return 5;
        case 5:
            return 6;
        case 6:
            return 7;
        default:
            return 0;  // LINE_UNKNOWN
    }
    return 0;
}

bool WrapperOutput::MakeLppInfo(const SDLaneElementGroupSet& EHP_ele_group, const SEhpOutputPathList& paths,const SEhpOutputLoc& loc, const SEhpOutputLinkList& link_list,
                            const LaneIdx& EgoLaneIdx,const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const NodeInfo& node,  LocJudgeInfo loc_res_info, datatype_efm::s_MapLane_t& map_lane) {
    // mNavInfo
    datatype_efm::s_MapLppInfo_t& map_lpp_info = map_lane.efm_info;
    map_lpp_info.mNavInfo.bNavIsYaw = false;
    const SEhpOutputPath *ego_path_ptr = nullptr;
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<<"loc.path_id_:"<< loc.path_id_ <<",loc.offset_:"<<loc.offset_<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<" paths.ehp_output_path_list.size():"<< paths.ehp_output_path_list.size()<<std::endl;
    for(int i =0; i< paths.ehp_output_path_list.size();i++){
        std::cout<<" ,< path_id:"<< (int)paths.ehp_output_path_list[i].path_id_<<">";
    }
    std::cout<<std::endl;
#endif
    for(auto& path : paths.ehp_output_path_list){
        if(path.path_id_ == loc.path_id_ ){
            map_lpp_info.mNavInfo.bIsSetNav = true;
            ego_path_ptr = &path;
        }
    }

    //odd 
    MakeOdd(ego_path_ptr, map_lpp_info);
    MakeLaneDecision(EHP_ele_group,EgoLaneIdx, LeftLaneIdx, RightLaneIdx,loc, loc_res_info,map_lane);
    MakeLaneSize(EHP_ele_group,loc,EgoLaneIdx, map_lpp_info.laneNumber, map_lpp_info.egoLaneIndex);
    map_lpp_info.laneIndex =map_lpp_info.laneNumber;
    MakeRampInfo(ego_path_ptr, loc, map_lpp_info);

    map_lpp_info.PositionCounter = loc.id_;
    map_lpp_info.PositionTimeStamp = loc.time_stmp_;

    MakeTunnelCount(ego_path_ptr, map_lpp_info);
    MakeSpeedLimit(ego_path_ptr, link_list, loc,loc_res_info.loc_lane_num_, map_lpp_info.mRampInfo.startPoint.dX, map_lpp_info.mRampInfo.endPoint.dX, map_lpp_info);
    MakeRoadType(link_list, loc, map_lpp_info);
    MakeNode(node, map_lpp_info);
    MakeDistanceByPassMerge(ego_path_ptr, map_lpp_info);

    return true;
}

bool WrapperOutput::MakeOdd(const SEhpOutputPath * ego_path,  datatype_efm::s_MapLppInfo_t& map_lpp_info){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "MakeOdd: "<<std::endl;
#endif
    if(ego_path == nullptr){
        return false;
    }
    std::vector<OddDis> Odds{};
    // TollBooth_ = 1; ServiceArea_ = 2;
    for(auto& special_sit: ego_path->special_datas_){
        if(special_sit.type_ == 3){
            map_lpp_info.mSpecialSit.eSpecSitType = static_cast<datatype_efm::e_SpecialSituation_t_ref>(SpecialSitMatch(3));
            map_lpp_info.mSpecialSit.dStartDistance = static_cast<double>(special_sit.s_offset_)/100;
            map_lpp_info.mSpecialSit.dEndDistance = static_cast<double>(special_sit.e_offset_)/100;
            Odds.push_back(OddDis(SpecialSitMatch(3), special_sit.s_offset_));
            break;
        }
    }

    if(ego_path->link_offsets_.size()>0){
        Odds.push_back(OddDis(0, ego_path->link_offsets_.back().e_offset_));
    }

    auto compareOddDis = [](const OddDis& a, const OddDis& b) { 
        return a.odddis < b.odddis; 
    };
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "Odds.size: "<<Odds.size();
    for(auto &odd: Odds){
        std::cout<<"<dist:"<<odd.odddis<<",type:"<<(int)odd.oddtype<<">,";
    }
    std::cout<<std::endl;
#endif
    if (false == Odds.empty()) {
        std::sort(Odds.begin(), Odds.end(), compareOddDis);
        map_lpp_info.mMapOdd.dDisToOdd = Odds[0].odddis/ 100; //输出m

        if (Odds[0].odddis > 150000) {//判断时候offset用cm
            map_lpp_info.mMapOdd.eInOdd = static_cast<datatype_mcu::e_InOdd_t_ref>(1);
            map_lpp_info.mMapOdd.eOddReason = static_cast<datatype_efm::e_OutODDReason1_t_ref>(0);
        } else if (Odds[0].odddis > 0) {
            if (Odds[0].oddtype == 1) {
                map_lpp_info.mMapOdd.eInOdd = static_cast<datatype_mcu::e_InOdd_t_ref>(2);
                map_lpp_info.mMapOdd.eOddReason = static_cast<datatype_efm::e_OutODDReason1_t_ref>(2);
            } else {
                map_lpp_info.mMapOdd.eInOdd = static_cast<datatype_mcu::e_InOdd_t_ref>(2);
                map_lpp_info.mMapOdd.eOddReason = static_cast<datatype_efm::e_OutODDReason1_t_ref>(0);
            }

        } else {
            if (Odds[0].oddtype == 1) {
                map_lpp_info.mMapOdd.eInOdd = static_cast<datatype_mcu::e_InOdd_t_ref>(0);
                map_lpp_info.mMapOdd.eOddReason = static_cast<datatype_efm::e_OutODDReason1_t_ref>(2);
            } else {
                map_lpp_info.mMapOdd.eInOdd = static_cast<datatype_mcu::e_InOdd_t_ref>(0);
                map_lpp_info.mMapOdd.eOddReason = static_cast<datatype_efm::e_OutODDReason1_t_ref>(1);
            }
        }
    }

    return true;
}

uint8_t WrapperOutput::SpecialSitMatch(uint8_t type_in){
// enum PATH_SPECIAL_TYPE {
//     // TODO(lxf)  这里面是道路级别的数据
//     PATH_SPECIAL_TYPE_UNKNOWN = 0,     //   未知，
//     // TODO(lxf)  后续自己判断
//     PATH_SPECIAL_TYPE_TO_RAMP = 1,     //   上匝道
//     PATH_SPECIAL_TYPE_TO_MAIN,         //   上主路
//     // 可以透传
//     PATH_SPECIAL_TYPE_TOLL_BOOTH,      //   收费站
//     // TODO(lxf)  需要拼接下
//     PATH_SPECIAL_TYPE_TUNNEL,          //   隧道
//     // TODO(lxf) 后续自己判断
//     PATH_SPECIAL_TYPE_RAMP_ROADMERGE,  //   匝道道路merge
//     PATH_SPECIAL_TYPE_RAMP_ROADSPLIT,  //   匝道道路split
//     PATH_SPECIAL_TYPE_MAIN_ROADMERGE,  //   主路道路merge
//     PATH_SPECIAL_TYPE_MAIN_ROADSPLIT,  //   主路道路split
// };

//out
// static constexpr uint8_t None_ = 0;
// static constexpr uint8_t TollBooth_ = 1;
// static constexpr uint8_t ServiceArea_ = 2;
// static constexpr uint8_t Tunnel_ = 3;

    switch (type_in) {
        case 0:
            return 0;  // None
        case 3:
            return 1;  // TollBooth
        case 4:
            return 3;  // tunnel
        default:
            return 0;  // None
    }
    return 0;
}

bool WrapperOutput::MakeLaneDecision(const SDLaneElementGroupSet& EHP_ele_group,const LaneIdx& EgoLaneIdx,
                            const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const SEhpOutputLoc& loc, LocJudgeInfo loc_res_info,datatype_efm::s_MapLane_t& map_lane){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "MakeLaneDecision: "<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "EgoLaneIdx.sd_lane_idx_.first : "<<EgoLaneIdx.sd_lane_idx_.first<<" ,EgoLaneIdx.sd_lane_idx_.second:"<<EgoLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;

#endif
    datatype_efm::s_MapLppInfo_t& map_lpp_info= map_lane.efm_info;
    //ego
    map_lpp_info.Array_laneDecision_3[0].isDriveable = true;
    uint32_t ego_rest_dist = std::numeric_limits<uint32_t>::max();
    int ego_lane_num_size = 0;
    if(EgoLaneIdx.sd_lane_idx_.first >= 0 &&
            EgoLaneIdx.sd_lane_idx_.first < EHP_ele_group.size()){
        const SDLaneElementGroup& ele_group = EHP_ele_group[EgoLaneIdx.sd_lane_idx_.first].second;
        if (EgoLaneIdx.sd_lane_idx_.second >= 0 &&
            EgoLaneIdx.sd_lane_idx_.second < ele_group.size()) {
            const SDLaneElement& lane_ele = ele_group[EgoLaneIdx.sd_lane_idx_.second];    
            ego_rest_dist =lane_ele.remain_dist;
            ego_lane_num_size = lane_ele.lane_nums.size();
        }        
    }
    map_lane.Array_lanes_5[2].ref_line.Array_linePnt_121[0].dX = static_cast<float>(ego_rest_dist)/100.0;

    //left
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_rest_dist : "<<ego_rest_dist<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "LeftLaneIdx.sd_lane_idx_.first : "<<LeftLaneIdx.sd_lane_idx_.first<<" ,LeftLaneIdx.sd_lane_idx_.second:"<<LeftLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    map_lpp_info.Array_laneDecision_3[1].isDriveable = false;
    if(map_lane.Array_lanes_5[1].enable_flag == true){
        if (LeftLaneIdx.sd_lane_idx_.first >= 0 &&
            LeftLaneIdx.sd_lane_idx_.first < EHP_ele_group.size()) {
            const SDLaneElementGroup& ele_group = EHP_ele_group[LeftLaneIdx.sd_lane_idx_.first].second;
            if (LeftLaneIdx.sd_lane_idx_.second >= 0 &&
                LeftLaneIdx.sd_lane_idx_.second < ele_group.size()) {
                map_lpp_info.Array_laneDecision_3[1].isDriveable = true;    
                const SDLaneElement& lane_ele = ele_group[LeftLaneIdx.sd_lane_idx_.second];
#ifdef MAKE_LPP_INFO
                std::cout << __FILE__ << "," << __LINE__ << "," <<"left lane num: ";
                for(auto& num: lane_ele.lane_nums){
                    std::cout<<(int)num<<",";
                }
                std::cout<<std::endl;
#endif
                //rest dist
                map_lane.Array_lanes_5[1].ref_line.Array_linePnt_121[0].dX = static_cast<float>(lane_ele.remain_dist)/100.0;
                if(lane_ele.remain_dist <50000 && lane_ele.remain_dist<ego_rest_dist){
#ifdef MAKE_LPP_INFO
                    std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ele.remain_dist <20000 : "<<lane_ele.remain_dist<<std::endl;
#endif
                    map_lpp_info.Array_laneDecision_3[1].isDriveable = false; 
                }            
                // merge,
                for(int i = 0; i< lane_ele.lane_merges.size() && i<lane_ele.lane_splits.size() && i<lane_ele.lane_s_offsets.size() && i<lane_ele.lane_e_offsets.size(); i++){
                    if(lane_ele.lane_merges[i] == EfmMergeType::EFM_MergeType_TO_RIGHT){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "i : "<<i<<",lane_ele.lane_e_offsets[i]: "<<lane_ele.lane_e_offsets[i]<<std::endl;
#endif
                        if(lane_ele.lane_e_offsets[i]<50000){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "merge to right < 20000 : "<<lane_ele.lane_e_offsets[i]<<std::endl;
#endif
                            map_lpp_info.Array_laneDecision_3[1].isDriveable = false; 
                        }
                    }

                }
            }
        }        
    }
   

#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "RightLaneIdx.sd_lane_idx_.first : "<<RightLaneIdx.sd_lane_idx_.first<<" ,RightLaneIdx.sd_lane_idx_.second:"<<RightLaneIdx.sd_lane_idx_.second<<" ,EHP_ele_group.size: " <<EHP_ele_group.size()<<std::endl;
#endif
    //right
    map_lpp_info.Array_laneDecision_3[2].isDriveable = false;
    if(map_lane.Array_lanes_5[3].enable_flag == true ){
        if (RightLaneIdx.sd_lane_idx_.first >= 0 &&
            RightLaneIdx.sd_lane_idx_.first < EHP_ele_group.size()) {
            const SDLaneElementGroup& ele_group = EHP_ele_group[RightLaneIdx.sd_lane_idx_.first].second;
            if (RightLaneIdx.sd_lane_idx_.second >= 0 &&
                RightLaneIdx.sd_lane_idx_.second < ele_group.size()) {
                map_lpp_info.Array_laneDecision_3[2].isDriveable = true;    
                const SDLaneElement& lane_ele = ele_group[RightLaneIdx.sd_lane_idx_.second];
#ifdef MAKE_LPP_INFO
                std::cout << __FILE__ << "," << __LINE__ << "," <<"right lane num: ";
                for(auto& num: lane_ele.lane_nums){
                    std::cout<<(int)num<<",";
                }
                std::cout<<std::endl;
#endif
                //rest dist
                map_lane.Array_lanes_5[3].ref_line.Array_linePnt_121[0].dX = static_cast<float>(lane_ele.remain_dist)/100.0;
                if(lane_ele.remain_dist <50000 && lane_ele.remain_dist<ego_rest_dist && lane_ele.lane_nums.size()<ego_lane_num_size){
#ifdef MAKE_LPP_INFO
                    std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ele.remain_dist <20000 : "<<lane_ele.remain_dist<<std::endl;
#endif
                    map_lpp_info.Array_laneDecision_3[2].isDriveable = false; 
                }            
                // merge,
                for(int i = 0; i< lane_ele.lane_merges.size() && i<lane_ele.lane_splits.size() && i<lane_ele.lane_s_offsets.size() && i<lane_ele.lane_e_offsets.size(); i++){
                    if(lane_ele.lane_merges[i] == EfmMergeType::EFM_MergeType_TO_LEFT){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "i : "<<i<<",lane_ele.lane_e_offsets[i]: "<<lane_ele.lane_e_offsets[i]<<std::endl;
#endif
                        if(lane_ele.lane_e_offsets[i]<50000){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "merge to left < 20000 : "<<lane_ele.lane_e_offsets[i]<<std::endl;
#endif
                            map_lpp_info.Array_laneDecision_3[2].isDriveable = false; 
                        }
                    }

                }
            }
        }  
    }
    for(auto& link_id : sd_links_for_lane_decision_loc_lane_num2){
        if(link_id == loc.link_id_ &&  loc_res_info.loc_lane_num_ ==2){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "loc_lane_num : "<<(int)loc_res_info.loc_lane_num_<<" ,loc.link_id_:"<<loc.link_id_<<std::endl;
#endif
            map_lpp_info.Array_laneDecision_3[2].isDriveable = false;
            break;
        }
    }
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << " map_lpp_info.Array_laneDecision_3[0].isDriveable : "<<(int) map_lpp_info.Array_laneDecision_3[0].isDriveable
    <<" ,map_lpp_info.Array_laneDecision_3[1].isDriveable: " <<(int)map_lpp_info.Array_laneDecision_3[1].isDriveable
    <<" ,map_lpp_info.Array_laneDecision_3[2].isDriveable: " <<(int)map_lpp_info.Array_laneDecision_3[2].isDriveable<<std::endl;

        std::cout << __FILE__ << "," << __LINE__ << "," << " map_lane.Array_lanes_5[2].ref_line.Array_linePnt_121[0].dX  : "<<map_lane.Array_lanes_5[2].ref_line.Array_linePnt_121[0].dX 
    <<" ,map_lane.Array_lanes_5[1].ref_line.Array_linePnt_121[0].dX : " <<map_lane.Array_lanes_5[1].ref_line.Array_linePnt_121[0].dX
    <<" ,map_lane.Array_lanes_5[3].ref_line.Array_linePnt_121[0].dX: " <<map_lane.Array_lanes_5[3].ref_line.Array_linePnt_121[0].dX<<std::endl;
#endif
    return true;
}

bool  WrapperOutput::MakeLaneSize(const SDLaneElementGroupSet& EHP_ele_group_set,const SEhpOutputLoc& loc,const LaneIdx& EgoLaneIdx,
                                           uint8_t& driveable_lane_size, uint8_t& fixed_lane_num) {
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "MakeLaneSize: "<<std::endl;
#endif  
    //lane_num
    fixed_lane_num = 0;
    uint8_t ego_lane_num = 0;
    if(loc.lane_id_ == 0){
        return false;
    }else{
        for(int i=0;i<loc.lane_ids_.size(); i++){
            if(loc.lane_ids_[i] == loc.lane_id_){
                ego_lane_num = i+1;
            }
        }
    }
    if(ego_lane_num == 0){
        return false;
    }
    fixed_lane_num = ego_lane_num;
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_lane_num / fixed_lane_num: " << (int)ego_lane_num << std::endl;
#endif
    uint8_t right_not_driveable_lane_size = 0;                                
    if (EHP_ele_group_set.size() <= 0) {
        return false;
    }

    for (auto lane_element_group : EHP_ele_group_set) {
        if (lane_element_group.second.size() <= 0) {
            return false;
        } else {
            for (auto lane_element : lane_element_group.second) {
                if (lane_element.lane_types.size() <= 0) {
                    return false;
                }
            }
        }
    }
    int ego_lane_size = 0;
     if (EgoLaneIdx.sd_lane_idx_.first >= 0 &&
        EgoLaneIdx.sd_lane_idx_.first < EHP_ele_group_set.size()) {
        const SDLaneElementGroup& ele_group = EHP_ele_group_set[EgoLaneIdx.sd_lane_idx_.first].second;
        if (EgoLaneIdx.sd_lane_idx_.second >= 0 &&
            EgoLaneIdx.sd_lane_idx_.second < ele_group.size()) {
            const SDLaneElement& lane_ele = ele_group[EgoLaneIdx.sd_lane_idx_.second];
            ego_lane_size = static_cast<int>(lane_ele.lane_types.size());
        }
    }  
    if (ego_lane_size == 0) {
        return false;
    }

    driveable_lane_size = 0;
    for (const auto& ele_group : EHP_ele_group_set) {
        if(ele_group.second.size() > 0){
            bool find_not_drive_f =false;
            for (const auto& lane_ele : ele_group.second) {
#ifdef MAKE_LPP_INFO
    if(lane_ele.lane_types.size()> 0 && lane_ele.lane_nums.size()>0){
        std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ele.lane_types.front(): " << (int)lane_ele.lane_types.front()
            <<" ,lane_ele.lane_nums.front():"<< (int)lane_ele.lane_nums.front()<<" ,lane_ele.lane_types.size()"<<lane_ele.lane_types.size()<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << "," << "LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_: "<< (int)LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_
            <<" ,LaneType::EnumLaneType_EMERGENCY_: "<< (int)LaneType::EnumLaneType_EMERGENCY_<<std::endl;
    }

#endif
                if (lane_ele.lane_types.size()>0 
                    && (lane_ele.lane_types.front() == LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_ || lane_ele.lane_types.front() == LaneType::EnumLaneType_EMERGENCY_)) {
                    find_not_drive_f = true;
                     break;
                }
            }
#ifdef MAKE_LPP_INFO
        std::cout << __FILE__ << "," << __LINE__ << "," << "find_not_drive_f: "<< (int)find_not_drive_f<<std::endl;
#endif
            if(find_not_drive_f == true){
                fixed_lane_num--;
                fixed_lane_num = std::max(static_cast<uint8_t>(1), fixed_lane_num);
            }else{
                driveable_lane_size++;
            }

        }
    }

#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw driveable_lane_size: " << (int)driveable_lane_size << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw fixed_lane_id: " << (int)fixed_lane_num << std::endl;
#endif
    for (int lane_num = ego_lane_num - 1; lane_num > 0; lane_num--) {
        for (auto lane_element_group : EHP_ele_group_set) {
            if (lane_element_group.second.front().lane_nums.front() == lane_num 
                && (lane_element_group.second.front().lane_types.front()!= LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_ && lane_element_group.second.front().lane_types.front()!=LaneType::EnumLaneType_EMERGENCY_)) {
                // get lane_element group
                bool is_merge_f = false;
                bool is_lane_end = true;
                for (auto lane_element : lane_element_group.second) {
                    if (lane_element.is_group_dest == true) {
                        if (lane_element.lane_nums.size() < ego_lane_size && lane_element.remain_dist<500) {
                            // lane end
                            right_not_driveable_lane_size++;
                            break;
                        } else {
                            //如果右边这条group最长,那就不判断merge了
                            // if(lane_element.lane_num_vec.size()>ego_lane_size){
                            //     break;
                            // }                            
                            // merge
                            double merge_dist = 0;
                            bool fisrt_link_length_zero = false;
                            bool link_is_in_toll_f = false; 
                            if (lane_element.first_merge_index >= 0 && lane_element.first_merge_index<lane_element.lane_s_offsets.size()) {
                                merge_dist = lane_element.lane_s_offsets[lane_element.first_merge_index];
                            }
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "link_length_vec_[0]: " << link_length_vec_[0] << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "lane_element.close_position: " << lane_element.close_position << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "merge_dist: " << merge_dist << std::endl;
                            if ((merge_dist > 0 || (fisrt_link_length_zero == true && merge_dist < 0.0001)) &&
                                merge_dist < 1000 && ego_lane_size >= lane_element.first_merge_index) {
                                right_not_driveable_lane_size++;
                                break;
                            }
                            

                        }
                    }
                }
            }
        }
    }

    if (driveable_lane_size > right_not_driveable_lane_size) {
        driveable_lane_size = driveable_lane_size - right_not_driveable_lane_size;
    }
    if (fixed_lane_num > right_not_driveable_lane_size) {
        fixed_lane_num = fixed_lane_num - right_not_driveable_lane_size;
    }

#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_not_driveable_lane_size: " << (int)right_not_driveable_lane_size << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "driveable_lane_size: " << (int)driveable_lane_size << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "fixed_lane_num: " << (int)fixed_lane_num << std::endl;
#endif

    return true;
}

bool WrapperOutput::MakeRampInfo(const SEhpOutputPath * ego_path,  const SEhpOutputLoc& loc, datatype_efm::s_MapLppInfo_t& map_lpp_info){
    if(ego_path == nullptr){
        return false;
    }
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<<"path_id:"<< ego_path->path_id_<<",parent_path_id_:"<<ego_path->parent_path_id_<<" ,ego_path->special_datas_.size():"<< ego_path->special_datas_.size()<<std::endl;
    for(int i =0; i< ego_path->special_datas_.size();i++){
        std::cout<<" ,< type:"<< (int)ego_path->special_datas_[i].type_<<",s:"<<ego_path->special_datas_[i].s_offset_<<", e:"<<ego_path->special_datas_[i].e_offset_<<">";
    }
    std::cout<<std::endl;
#endif
    if(ego_path->special_datas_.size()>0){
        for(const auto& special_data: ego_path->special_datas_){
            if(special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_TO_RAMP) {
                map_lpp_info.mRampInfo.startPoint.dX = static_cast<double>(special_data.s_offset_)/100.0 -static_cast<double>(loc.offset_)/100.0 -35.0;
                map_lpp_info.mRampInfo.endPoint.dX = static_cast<double>(special_data.e_offset_)/100.0 - static_cast<double>(loc.offset_)/100.0;
                break;
            }
        }
        for(const auto& special_data: ego_path->special_datas_){
            if(special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_TO_MAIN) {
                map_lpp_info.mRampInfo.OnRampStartPoint.dX = static_cast<double>(special_data.s_offset_)/100.0- static_cast<double>(loc.offset_)/100.0;
                map_lpp_info.mRampInfo.OnRampEndPoint.dX = static_cast<double>(special_data.e_offset_)/100.0- static_cast<double>(loc.offset_)/100.0;
                break;
            }
        }
    }

    return true;
}

bool WrapperOutput::MakeTunnelCount(const SEhpOutputPath * ego_path,  datatype_efm::s_MapLppInfo_t& map_lpp_info){
    map_lpp_info.nTunnelCount = 0;
    if(ego_path == nullptr){
        return false;
    }
    
    for(const auto& special_data: ego_path->special_datas_){
        if(special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_TUNNEL) {
            map_lpp_info.nTunnelCount++;
        }
    }
    return true;
}

bool WrapperOutput::MakeSpeedLimit(const SEhpOutputPath * ego_path, const SEhpOutputLinkList& link_list, const SEhpOutputLoc& loc, uint8_t loc_lane_num, double exit_start, double exit_end, datatype_efm::s_MapLppInfo_t& map_lpp_info){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "MakeSpeedLimit"<< " ,link_list.ehp_output_link_list.size()"<<link_list.ehp_output_link_list.size()<<std::endl;
#endif
    if(ego_path != nullptr){
        bool find_cur_spd_f = false;
        uint32_t cur_spd_offset = 0;
        uint64_t cur_spd_link_id = 0;
        uint8_t cur_spd_val = 0;
        uint32_t cur_spd_link_start_offset = 0;
        bool find_next_spd_f = false;
        uint32_t next_spd_offset = 0;
        uint64_t next_spd_link_id = 0;
        uint8_t next_spd_val = 0;
        uint32_t next_spd_link_start_offset = 0;
        bool find_next_next_spd_f = false;
        uint32_t next_next_spd_offset = 0;
        uint64_t next_next_spd_link_id = 0;
        uint8_t next_next_spd_val = 0;
        uint32_t next_next_spd_link_start_offset = 0;
        for(const auto& link:link_list.ehp_output_link_list){
            if(link.id_ == loc.link_id_ && find_cur_spd_f == false){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "link.id: "<< link.id_<<" ,loc.offset_:"<<loc.offset_<< std::endl;
#endif
                for(int i =link.speeds_.size()-1;i>=0;i-- ){
                    auto spd = link.speeds_[i];
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "spd.offset_: "<< spd.offset_ <<" , spd.value_"<< spd.value_<< std::endl;
#endif                    
                    if( loc.offset_ >= spd.offset_){
                        map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane = static_cast<uint8_t>(spd.value_);
                        find_cur_spd_f = true;
                        cur_spd_offset = loc.offset_;
                        cur_spd_link_id = loc.link_id_;
                        cur_spd_val = static_cast<uint8_t>(spd.value_);
                        cur_spd_link_start_offset = link.s_offset_;
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "cur_spd_offset: "<< cur_spd_offset<<" ,value: "<<(int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane<< std::endl;
#endif
                        break;
                    }                   
                }
                
            }
            if(find_cur_spd_f == true && find_next_spd_f == false){
                for(auto spd: link.speeds_){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "next spd.offset_: "<< spd.offset_ <<" , next spd.value_"<< spd.value_<< std::endl;
#endif 
                    if(spd.offset_> cur_spd_offset && spd.value_ != map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane){
                        map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane = static_cast<uint8_t>(spd.value_);
                        find_next_spd_f = true;
                        // next_spd_offset = spd.offset_-cur_spd_offset;
                        next_spd_offset = spd.offset_;
                        next_spd_val = static_cast<uint8_t>(spd.value_);
                        next_spd_link_id = link.id_;
                        next_spd_link_start_offset = link.s_offset_;
                        map_lpp_info.mRefLineSpeeds.nPntIdx = std::min((int)(next_spd_offset-cur_spd_offset)/250, 255);
                    }                   
                }
            }
            if(find_cur_spd_f == true && find_next_spd_f == true && find_next_next_spd_f == false){
                for(auto spd: link.speeds_){
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "next next spd.offset_: "<< spd.offset_ <<" , next next spd.value_"<< spd.value_<< std::endl;
#endif 
                    if(spd.offset_> next_spd_offset && spd.value_ != next_spd_val){
                        find_next_next_spd_f = true;
                        next_next_spd_offset = spd.offset_;
                        next_next_spd_val = static_cast<uint8_t>(spd.value_);
                        next_next_spd_link_id = link.id_;
                        next_spd_link_start_offset = link.s_offset_;
                    }                   
                }
            }
            if(find_cur_spd_f == true && find_next_spd_f == true && find_next_next_spd_f == true){
                break;
            }

        }
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "cur_spd_offset: "<< cur_spd_offset<<" ,next_spd_offset:"<< next_spd_offset<<" ,next_next_spd_offset"<<next_next_spd_offset<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< "cur_spd_val: "<< (int)cur_spd_val<<" ,next_spd_val:"<< (int)next_spd_val<<" ,next_next_spd_val"<<(int)next_next_spd_val<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< "cur_spd_link_id: "<< cur_spd_link_id<<" ,next_spd_link_id:"<< next_spd_link_id<<" ,next_next_spd_link_id"<<next_next_spd_link_id<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< "find_cur_spd_f: "<< (int)find_cur_spd_f<<" ,find_next_spd_f:"<< (int)find_next_spd_f<<" ,find_next_next_spd_f:"<<(int)find_next_next_spd_f<<std::endl;
#endif
    // //输出
    if(find_cur_spd_f == true && find_next_spd_f == true && find_next_next_spd_f == true){
        //去除中间短距离的变化段
        if(cur_spd_val == next_next_spd_val && (next_next_spd_offset - next_spd_offset)<= 10000 ){
            // std::cout << __FILE__ << "," << __LINE__ << ","<< "开始保存 static "<< std::endl;
            bool find_start_f = false;
            bool find_end_f = false;
            uint64_t link_id = 0;
            uint32_t e_offset = 0;
            uint32_t s_offset = 0;
            uint8_t val = 0;
            for(const auto& link:link_list.ehp_output_link_list){
                // std::cout << __FILE__ << "," << __LINE__ << ","<< "link.id in path: "<<link.id_ <<" link_id: "<<link_id<<std::endl;
                for(const auto& spd:link.speeds_){
                    // std::cout << __FILE__ << "," << __LINE__ << ","<< "spd.offset: "<<spd.offset_<< ", find_start_f: "<<(int)find_start_f<<" ,"<<std::endl;
                    if(find_start_f == true){
                        if(link.id_ != link_id){
                            e_offset = link.s_offset_;
                            SpeedInfoS speed_info(cur_spd_val,link_id, s_offset, e_offset);
                            bool push_f = true;
                            for(auto& iter:changed_speed_links){
                                if(iter == speed_info){
                                    push_f = false;
                                }
                            }
                            if(push_f == true ){
                                if(changed_speed_links.size()>=30){
                                    changed_speed_links.pop_front();
                                }
                                changed_speed_links.push_back(speed_info);
                                // std::cout << __FILE__ << "," << __LINE__ << ","<< "push_back: id: "<<speed_info.link_id <<" ,val: "<<(int)speed_info.value<<",s_offset:"<<speed_info.start_offset<<" ,e_offset:"<<speed_info.end_offset<<std::endl;

                            }
                            
                            find_start_f = false;
                        }
                    }
                    if(find_start_f == false){
                    // std::cout << __FILE__ << "," << __LINE__ << ","<< "spd.offset: "<<spd.offset_<< ", next_spd_offset: "<<next_spd_offset<<" ,"<<std::endl;
                        if(spd.offset_>=next_spd_offset && spd.offset_<= next_next_spd_offset && spd.value_!= cur_spd_val){
                            find_start_f = true;
                            link_id = link.id_;
                            s_offset = spd.offset_;                            
                        }
                    }
                }  
            }

        }
    }
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "before fix: cur_spd: "<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane<<" ,next_spd:"<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane
    <<",index: "<<(int)map_lpp_info.mRefLineSpeeds.nPntIdx<<std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","<< "changed_speed_links.size(): "<< changed_speed_links.size()<<std::endl;
    for(int i=0; i<changed_speed_links.size();i++){
        std::cout<<"changed_speed_links[ "<<i<<"]:"<<"val:"<<(int)changed_speed_links[i].value<<" ,link_id:"<<changed_speed_links[i].link_id<<" ,s_offset: "<<changed_speed_links[i].start_offset<<" ,e_offset:"<<changed_speed_links[i].end_offset<<std::endl;
    } 
#endif    

    for(auto & info: changed_speed_links){
        if(cur_spd_link_id == info.link_id && cur_spd_offset>= info.start_offset && cur_spd_offset<=info.end_offset){
            map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane = info.value;
        }
        if(next_spd_link_id == info.link_id && next_spd_offset>= info.start_offset && next_spd_offset<=info.end_offset){
            map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane = info.value;
        }        
    }
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "after fix: cur_spd: "<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane<<" ,next_spd:"<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane
    <<",index: "<<(int)map_lpp_info.mRefLineSpeeds.nPntIdx<<std::endl;
#endif 
    //new 20251012, 下匝道的情况，把匝道内的限速提前到下匝道起点
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "exit_start: "<< exit_start<<" ,exit_end:"<< exit_end<<std::endl;
#endif
        if(fabs(exit_end)>0.0001 && fabs(exit_start)>0.00001){
            if(exit_end>0 && fabs(exit_end - next_spd_offset/100.0)<20){
                if(exit_end<200){
                    map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane = map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane;
                }
            }            
        }

    }
//打点
    for(auto& link_id : sd_links_for_speed_limit_loc_lane_num2and1){
        if(link_id == loc.link_id_ && loc_lane_num <=1){
#ifdef MAKE_LPP_INFO
                            std::cout << __FILE__ << "," << __LINE__ << "," << "loc_lane_num : "<<(int)loc_lane_num<<" ,loc.link_id_:"<<loc.link_id_<<std::endl;
#endif
            map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane = 60;
            break;
        }
    }
    // inline static std::vector<uint64_t> sd_links_for_speed_limit_loc_lane_num2and1={11016949150637, 11005923573302, 11005923573303, 11005923573304 } ;
#ifdef MAKE_LPP_INFO
    std::cout << __FILE__ << "," << __LINE__ << ","<< "final: cur_spd: "<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane<<" ,next_spd:"<< (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane
    <<",index: "<<(int)map_lpp_info.mRefLineSpeeds.nPntIdx<<std::endl;
#endif 
    return true;
}

bool WrapperOutput::MakeRoadType(const SEhpOutputLinkList& link_list, const SEhpOutputLoc& loc, datatype_efm::s_MapLppInfo_t& map_lpp_info){
    for(const auto& link:link_list.ehp_output_link_list){
        if(link.id_ == loc.link_id_){
            switch (link.frc_) {
                case LINK_FRC::LINK_FRC_MOTORWAY:
                    map_lpp_info.RoadType = static_cast<datatype_efm::e_EfmTypRoad_t_ref>(5);
                    break;
                case LINK_FRC::LINK_FRC_URBAN_MOTORWAY:
                    map_lpp_info.RoadType = static_cast<datatype_efm::e_EfmTypRoad_t_ref>(1);
                    break;
                default:
                    map_lpp_info.RoadType = static_cast<datatype_efm::e_EfmTypRoad_t_ref>(0);
                    break;
            }
            break;
        }
    }    
    return true;
}

bool WrapperOutput::MakeNode(const NodeInfo& node, datatype_efm::s_MapLppInfo_t& map_lpp_info){ 
    map_lpp_info.NodeInfo.StartPointOffset = node.StartPointOffset;
    map_lpp_info.NodeInfo.EndPointOffset = node.EndPointOffset;
    map_lpp_info.NodeInfo.DirectionType = static_cast<datatype_efm::e_DirectionType_t_ref>(node.dir);
    map_lpp_info.NodeInfo.LaneChgType = node.LaneChgType;
    map_lpp_info.NodeInfo.LaneChgTimes = node.LaneChgTimes;
    return true;
}

bool WrapperOutput::MakeDistanceByPassMerge(const SEhpOutputPath * ego_path, datatype_efm::s_MapLppInfo_t& map_lpp_info){
    if(ego_path != nullptr){
        for(const auto& special_data: ego_path->special_datas_){
            if(special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_MAIN_ROADMERGE) {
                map_lpp_info.dDistanceByPassMerge = static_cast<double>(special_data.s_offset_)/100.0;
                break;
            }
        }

    }
    return true;
}

//L2
bool WrapperOutput::Execute(const std::vector<BevLineInnerS>& bev_lines, datatype_efm::s_MapLane_t& map_lane){
#ifdef WOUT
    std::cout << __FILE__ << "," << __LINE__ << "Execute L2 " <<std::endl;
#endif
    //用原始的bev分配
    BevLineInnerS left_line, right_line, adj_left_line, adj_right_line, bev_left_curb, bev_right_curb;
    int index_tmp = 0;
    for(auto& bev_line: bev_lines){
        if(bev_line.lane_location_type == 1){
            left_line = bev_line;
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "left_line.id: " << left_line.id<<" ,index:"<<index_tmp<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "left_line.munis: " <<(int)left_line.minus_valid<<" ," <<left_line.minus_line.size()<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "left_line.first: " <<(int)left_line.first_valid<<" ," <<left_line.first_line.size()<<std::endl;
#endif
        } else if (bev_line.lane_location_type == 2) {
            right_line = bev_line;
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "right_line.id: " << right_line.id<<" ,index:"<<index_tmp<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "right_line.munis: " <<(int)right_line.minus_valid<<" ," <<right_line.minus_line.size()<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "right_line.first: " <<(int)right_line.first_valid<<" ," <<right_line.first_line.size()<<std::endl;
#endif
        } else if (bev_line.lane_location_type == 3) {
            adj_left_line = bev_line;
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "adj_left_line.id: " << adj_left_line.id<<" ,index:"<<index_tmp<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "adj_left_line.munis: " <<(int)adj_left_line.minus_valid<<" ," <<adj_left_line.minus_line.size()<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "adj_left_line.first: " <<(int)adj_left_line.first_valid<<" ," <<adj_left_line.first_line.size()<<std::endl;
#endif
        } else if (bev_line.lane_location_type == 4) {
            adj_right_line = bev_line;
#ifdef WOUT
            std::cout << __FILE__ << "," << __LINE__ << "adj_right_line.id: " << adj_right_line.id<<" ,index:"<<index_tmp<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "adj_right_line.munis: " <<(int)adj_right_line.minus_valid<<" ," <<adj_right_line.minus_line.size()<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "adj_right_line.first: " <<(int)adj_right_line.first_valid<<" ," <<adj_right_line.first_line.size()<<std::endl;
#endif
        } else if(bev_line.lane_location_type == 21){
            bev_left_curb = bev_line;
        } else if(bev_line.lane_location_type == 22){
            bev_right_curb = bev_line;
        }
        index_tmp++;
    }
    datatype_efm::s_Lane_t &ego_lane = map_lane.Array_lanes_5[2];
    datatype_efm::s_Lane_t &left_lane = map_lane.Array_lanes_5[1];
    datatype_efm::s_Lane_t &right_lane = map_lane.Array_lanes_5[3];
    // std::cout << __FILE__ << "," << __LINE__ << "fil ego_lane left_line."<<std::endl;
    FillLaneSideLine(left_line, ego_lane.left_line);
    // std::cout << __FILE__ << "," << __LINE__ << "fil ego_lane right_line."<<std::endl;
    FillLaneSideLine(right_line, ego_lane.right_line);
    // std::cout << __FILE__ << "," << __LINE__ << "ego_lane.left_line.bIsAvailable."<<(int)ego_lane.left_line.bIsAvailable<<
    //  " ,ego_lane.right_line.bIsAvailable."<<(int)ego_lane.right_line.bIsAvailable<<std::endl;
    if(ego_lane.left_line.bIsAvailable == true || ego_lane.right_line.bIsAvailable == true){
        ego_lane.enable_flag = true;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "fil left_lane left_line."<<std::endl;

    FillLaneSideLine(left_line, left_lane.right_line);
    // std::cout << __FILE__ << "," << __LINE__ << "fil left_lane right_line."<<std::endl;

    FillLaneSideLine(adj_left_line, left_lane.left_line);
    // std::cout << __FILE__ << "," << __LINE__ << "left_lane.left_line.bIsAvailable."<<(int)left_lane.left_line.bIsAvailable<<
    //  "left_lane.right_line.bIsAvailable."<<(int)left_lane.right_line.bIsAvailable<<std::endl;
    if(left_lane.left_line.bIsAvailable == true || left_lane.right_line.bIsAvailable == true){
        left_lane.enable_flag = true;
    }

    FillLaneSideLine(right_line, right_lane.left_line);
    FillLaneSideLine(adj_right_line, right_lane.right_line);
    if(right_lane.left_line.bIsAvailable == true || right_lane.right_line.bIsAvailable == true){
        right_lane.enable_flag = true;
    }

    //路沿
    datatype_efm::s_CurbInfo_t &left_curb = map_lane.Array_CurbInfo_2[0];
    datatype_efm::s_CurbInfo_t &right_curb = map_lane.Array_CurbInfo_2[1];
    FillCurb(bev_left_curb,left_curb);
    FillCurb(bev_right_curb,right_curb);

    // PlotOneLane("egolane", map_lane.Array_lanes_5[2]);
    return true;
}

uint8_t WrapperOutput::CamLineTypeToEfm(uint8_t type_in){
    // using e_CamLineType_t_ref = enum class e_CamLineType_t_ref:uint8_t {
    //                                CamLineType_None = 0, 
    //                                CamLineType_Solid = 1, 
    //                                CamLineType_Dashed = 2, 
    //                                CamLineType_DoubleSolid = 3, 
    //                                CamLineType_DoubleDashed = 4, 
    //                                CamLineType_SolidDashed = 5, 
    //                                CamLineType_DashedSolid = 6, 
    //                                CamLineType_Fishbone = 7, 
    //                                CamLineType_TEMPORARY = 8, 
    //                                CamLineType_CURB = 9, 
    //                                CamLineType_CONE = 10
    //                              };

    // datatype_efm "" using e_LineType_t_ref = enum class e_LineType_t_ref:uint8_t {
    //                             LineType_Unknow_ = 0, 
    //                             LineType_None_ = 1, 
    //                             LineType_SolidLine_ = 2, 
    //                             LineType_DashedLine_ = 3, 
    //                             LineType_DoubleSoildLine_ = 4, 
    //                             LineType_DoubleDashedLine_ = 5, 
    //                             LineType_LeftSolidRightDashed_ = 6, 
    //                             LineType_RightSoildLeftDashed_ = 7, 
    //                             LineType_Virtual_ = 8
    //                           }; 

    uint8_t type_out = 0;
    switch(type_in){
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            type_out = (type_in + 1);
            break;
        default:
            type_out = 0;
            break;
    }   
    return type_out;
}

bool WrapperOutput::FillCurb(const BevLineInnerS& bev_line, datatype_efm::s_CurbInfo_t &out_curb){
    out_curb.Vec_linePoints.clear();
    if(bev_line.id !=0){
        for(auto p:bev_line.total_line){
            datatype_efm::s_LinePoint_t p_tmp;
            p_tmp.x = p.x;
            p_tmp.y = p.y;
            out_curb.Vec_linePoints.push_back(p_tmp);
        }
    }
    if(out_curb.Vec_linePoints.size()>1){
        out_curb.bIsAvailable = true;
        out_curb.StartIndex = 0;
        out_curb.EndIndex = out_curb.Vec_linePoints.size()-1;
    }
            
    return true;
}

bool WrapperOutput::FillLaneSideLine(const BevLineInnerS& bev_line, datatype_efm::s_LineMkr_t& lane_line){
    int back_num = 41;
    lane_line.bIsAvailable = false;
    if(bev_line.id != 0){
        int out_pnt_index = 0;
        lane_line.MdlQly = bev_line.md_qly;
        // std::cout << __FILE__ << "," << __LINE__ << "bev_line.minus_valid: "<<(int)bev_line.minus_valid<<std::endl;
        if(bev_line.minus_valid == true){
            int i = std::max(0, static_cast<int>(bev_line.minus_line.size()-1 -40));
            // std::cout << __FILE__ << "," << __LINE__ << "bev_line.minus_line.size(): "<<bev_line.minus_line.size()<<std::endl;
            for(; i>=0 && i< bev_line.minus_line.size() && out_pnt_index<back_num;i++){
                lane_line.Array_linePoints_121[out_pnt_index].dX = bev_line.minus_line[i].x;
                lane_line.Array_linePoints_121[out_pnt_index].dY = bev_line.minus_line[i].y;
                out_pnt_index ++;
            }
        }
        // std::cout << __FILE__ << "," << __LINE__ << "minus done out_pnt_index: "<<out_pnt_index<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "bev_line.first_valid: "<<(int)bev_line.first_valid<<std::endl;
        if(bev_line.first_valid == true){
            // std::cout << __FILE__ << "," << __LINE__ << "bev_line.first_line.size(): "<<bev_line.first_line.size()<<std::endl;
            for(int i =0;i<bev_line.first_line.size() && out_pnt_index<lane_line.Array_linePoints_121.size();i++){
                lane_line.Array_linePoints_121[out_pnt_index].dX = bev_line.first_line[i].x;
                lane_line.Array_linePoints_121[out_pnt_index].dY = bev_line.first_line[i].y;
                out_pnt_index ++;                
            }
        }
        // std::cout << __FILE__ << "," << __LINE__ << "first done out_pnt_index: "<<out_pnt_index<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "bev_line.sec_valid: "<<(int)bev_line.sec_valid<<std::endl;
        if(bev_line.sec_valid == true){
            // std::cout << __FILE__ << "," << __LINE__ << "bev_line.sec_line.size(): "<<bev_line.sec_line.size()<<std::endl;
            for(int i =0;i<bev_line.sec_line.size() && out_pnt_index<lane_line.Array_linePoints_121.size();i++){
                lane_line.Array_linePoints_121[out_pnt_index].dX = bev_line.sec_line[i].x;
                lane_line.Array_linePoints_121[out_pnt_index].dY = bev_line.sec_line[i].y;
                out_pnt_index ++;                
            }
        }
        // std::cout << __FILE__ << "," << __LINE__ << "sec done out_pnt_index: "<<out_pnt_index<<std::endl;
        if(out_pnt_index>1){
            lane_line.bIsAvailable = true;
            lane_line.pntSize = out_pnt_index;
        }
        // std::cout << __FILE__ << "," << __LINE__ << "lane_line.pntSize "<<(int)lane_line.pntSize<<std::endl;

        //边线type
        if (bev_line.typ_chg_point > 10000) {  // 只有一段
        //std::cout << __FILE__ << "," << __LINE__ << "," << "type_first 只有一段: "<<std::endl;
            lane_line.Array_etype_10[0].valid = true;
            lane_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.line_type));
            lane_line.Array_etype_10[0].s = 0;
        } else {  // 两段
        //std::cout << __FILE__ << "," << __LINE__ << "," << "type_first 两段 case1: "<<std::endl;
            lane_line.Array_etype_10[0].valid = true;
            lane_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.line_type));;
            lane_line.Array_etype_10[0].s = bev_line.typ_chg_point;

            lane_line.Array_etype_10[1].valid = true;
            lane_line.Array_etype_10[1].type= static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.typ_aft_chg_point));
            lane_line.Array_etype_10[1].s = 200;
        }
        // if(bev_line.first_valid == true ){
        //     lane_line.Array_etype_10[0].valid = true;
        //     lane_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.line_type));
        //     lane_line.Array_etype_10[0].s = bev_line.FirstStartPoint;
        //     if(bev_line.typ_chg_point>0){
        //         lane_line.Array_etype_10[1].valid = true;
        //         lane_line.Array_etype_10[1].type = static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.typ_aft_chg_point));
        //         lane_line.Array_etype_10[1].s = bev_line.typ_chg_point;
        //     }
        // }else if(bev_line.minus_valid == true){
        //     lane_line.Array_etype_10[0].valid = true;
        //     lane_line.Array_etype_10[0].type = static_cast<datatype_efm::e_LineType_t_ref>(CamLineTypeToEfm(bev_line.line_type));
        //     lane_line.Array_etype_10[0].s = bev_line.MinusStartPoint;
        // }

    }        
    return true;
}

bool WrapperOutput::PlotOneLane(std::string lane_name, datatype_efm::s_Lane_t& lane){
    std::cout << __FILE__ << "," << __LINE__ << ","<<lane_name
              << ".left_line.bIsAvailable: " << (int)lane.left_line.bIsAvailable<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<lane_name
              << ".left_line.pntSize: " << (int)lane.left_line.pntSize << std::endl;
    std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
    ss << lane_name<<".left_line.etype: ";
    for (int i = 0; i < lane.left_line.Array_etype_10.size(); i++) {
        ss << " ,[v: " << lane.left_line.Array_etype_10[i].valid
           << " ,s: " << lane.left_line.Array_etype_10[i].s
           << " ,type: " << (int)lane.left_line.Array_etype_10[i].type << " ]";
    }
    std::cout<<ss.str()<<std::endl;
        {
        std::stringstream ssx, ssy;
        ssx<<"lelanex = [";
        ssy<<"lelaney = [";
        for(int i=0;i< lane.left_line.pntSize;i++){
            ssx<< lane.left_line.Array_linePoints_121[i].dX<<" ";
            ssy<< lane.left_line.Array_linePoints_121[i].dY<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }

        //right
    std::cout << __FILE__ << "," << __LINE__ << ","<<lane_name
              << ".right_line.bIsAvailable: " << (int)lane.right_line.bIsAvailable
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<lane_name
              << ".right_line.pntSize: " << (int)lane.right_line.pntSize << std::endl;
    ss3 << ".right_line.etype: ";
    for (int i = 0; i < lane.right_line.Array_etype_10.size(); i++) {
        ss3 << " ,[v: " << lane.right_line.Array_etype_10[i].valid
            << " ,s: " << lane.right_line.Array_etype_10[i].s
            << " ,type: " << (int)lane.right_line.Array_etype_10[i].type << " ]";
    }
        {
        std::stringstream ssx, ssy;
        ssx<<"rilanex = [";
        ssy<<"rilaney = [";
        for(int i=0;i< lane.right_line.pntSize;i++){
            ssx<< lane.right_line.Array_linePoints_121[i].dX<<" ";
            ssy<< lane.right_line.Array_linePoints_121[i].dY<<" ";
        }
        ssx<<"]"<<std::endl;
        ssy<<"]"<<std::endl; 
        std::cout<<ssx.str();   
        std::cout<<ssy.str();    
        }
    std::cout << ss3.str() << std::endl; 
    return true;
}

bool WrapperOutput::InitMapLane(datatype_efm::s_MapLane_t& map_lane){
    //对车道线和中心线初始化
    for(int i=0;i<map_lane.Array_lanes_5.size();i++){
        map_lane.Array_lanes_5[i].enable_flag = false;
        InitOneLineMkr(map_lane.Array_lanes_5[i].left_line);
        InitOneLineMkr(map_lane.Array_lanes_5[i].right_line);
    }
    return true;
}

bool WrapperOutput::InitOneLineMkr(datatype_efm::s_LineMkr_t& lane_line){
    lane_line.MdlQly = 0;
    lane_line.bIsAvailable = false;
    lane_line.pntSize = 0;
    lane_line.Array_linePoints_121 ={0};
    for(auto & iter: lane_line.Array_etype_10){
        iter.valid = false;
        iter.type = static_cast<datatype_efm::e_LineType_t_ref>(0);
        iter.s = 0;
    }
    return true;
}

void WrapperOutput::MakeLaneId(const BevLaneElement& bev_lane, const SEhpOutputLoc& loc, datatype_efm::s_Lane_t & lane_out){
        // std::cout<<"bev_lane ids,"<<bev_lane.left_back_connect_id<<" ,"<<bev_lane.left_front_connect_id<<" ,"<<bev_lane.left_line_base_id
        // <<" ,"<<bev_lane.right_back_connect_id<<" ,"<<bev_lane.right_front_connect_id<<" ,"<<bev_lane.right_line_base_id<<std::endl;
    uint8_t& lane_id =lane_out.ref_line.pntSize;
    lane_id = 0;
    bool is_found_lane = false;
    BevLaneIds lane_ids(bev_lane.left_line_base_id, bev_lane.left_back_connect_id, bev_lane.left_front_connect_id,
                        bev_lane.right_line_base_id, bev_lane.right_back_connect_id, bev_lane.right_front_connect_id);
    for(auto & iter:lane_ids_map){
        bool is_left_same = false;
        bool is_right_same = false;
        if(bev_lane.left_back_connect_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.left_back_connect_id == iter_sub.left_back_connect_id || bev_lane.left_back_connect_id == iter_sub.left_line_base_id || bev_lane.left_back_connect_id == iter_sub.left_front_connect_id)
                    &&(iter_sub.left_back_connect_id != bev_lane.right_line_base_id)){
    // std::cout << __FILE__ << "," << __LINE__ << "," << "1111111111111111111111"<<std::endl;
    //   std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
    //     <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;    
                    is_left_same = true;
                    break;
                } 
            }           
        }
        if(bev_lane.left_line_base_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.left_line_base_id == iter_sub.left_back_connect_id || bev_lane.left_line_base_id == iter_sub.left_line_base_id || bev_lane.left_line_base_id == iter_sub.left_front_connect_id)
                    &&(iter_sub.left_line_base_id != bev_lane.right_line_base_id)){
                        
//  std::cout << __FILE__ << "," << __LINE__ << "," << "222222222222222"<<std::endl; 
//   std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
//         <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;
                    is_left_same = true;
                    break;
                }   
            }         
        }
        if(bev_lane.left_front_connect_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.left_front_connect_id == iter_sub.left_back_connect_id || bev_lane.left_front_connect_id == iter_sub.left_line_base_id || bev_lane.left_front_connect_id == iter_sub.left_front_connect_id)
                    &&(iter_sub.left_front_connect_id != bev_lane.right_line_base_id)){
//  std::cout << __FILE__ << "," << __LINE__ << "," << "33333333333333333"<<std::endl; 
//    std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
//         <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;
                    is_left_same = true;
                    break;
                }  
            }
        }
        //****************** */
        if(bev_lane.right_back_connect_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.right_back_connect_id == iter_sub.right_back_connect_id || bev_lane.right_back_connect_id == iter_sub.right_line_base_id || bev_lane.right_back_connect_id == iter_sub.right_front_connect_id)
                    &&(iter_sub.right_back_connect_id != bev_lane.left_line_base_id)){
        //                  std::cout << __FILE__ << "," << __LINE__ << "," << "4444444444444444444"<<std::endl; 
        //                    std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
        // <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;
                    is_right_same = true;
                    break;
                }    
            }        
        }
        if(bev_lane.right_line_base_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.right_line_base_id == iter_sub.right_back_connect_id || bev_lane.right_line_base_id == iter_sub.right_line_base_id || bev_lane.right_line_base_id == iter_sub.right_front_connect_id)
                    &&(iter_sub.right_line_base_id != bev_lane.left_line_base_id)){
        //                  std::cout << __FILE__ << "," << __LINE__ << "," << "5555555555555555"<<std::endl; 
        //                    std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
        // <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;
                    is_right_same = true;
                    break;
                }       
            }
        }
        if(bev_lane.right_front_connect_id != 0){
            for(auto& iter_sub: iter.second){
                if((bev_lane.right_front_connect_id == iter_sub.right_back_connect_id || bev_lane.right_front_connect_id == iter_sub.right_line_base_id || bev_lane.right_front_connect_id == iter_sub.right_front_connect_id)
                    &&(iter_sub.right_front_connect_id != bev_lane.left_line_base_id)){
        //                  std::cout << __FILE__ << "," << __LINE__ << "," << "6666666666666666"<<std::endl; 
        //                    std::cout << __FILE__ << "," << __LINE__ << "," << iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
        // <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;
                    is_right_same = true;
                    break;
                }     
            }       
        }

        if(is_right_same && is_left_same){  
            is_found_lane = true;
            lane_id = iter.first;

            bool find_iter = false;
            for(auto& iter_sub: iter.second){
                if(iter_sub == lane_ids){
                    find_iter = true;
                    break;
                }
            }
            if(find_iter == false){//如果没有一模一样的，但是又找到了，推到现有的里
                if(iter.second.size()>50){
                    iter.second.pop_front();
                }
                iter.second.push_back(lane_ids);
            }
            break;
        }
    }

    if(is_found_lane == false){
        //如果没找到，看左的base 是否一致， 如果左base一致，右侧的不一样，也认为是同一条
        for(auto & iter:lane_ids_map){
            for(auto & iter_sub: iter.second){
                if(lane_ids.left_line_base_id == iter_sub.left_line_base_id && iter_sub.left_back_connect_id == 0 && iter_sub.left_front_connect_id == 0 && lane_ids.left_back_connect_id == 0 && lane_ids.left_front_connect_id == 0){
    // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ids.left_line_base_id: " << lane_ids.left_line_base_id<<",iter_sub.left_line_base_id: "<<iter_sub.left_line_base_id<<std::endl;    
                    if(iter.second.size()>50){
                        iter.second.pop_front();
                    }
                    iter.second.push_back(lane_ids);
                    lane_id = iter.first;
                    break;
                }
                if(lane_ids.right_line_base_id == iter_sub.right_line_base_id && iter_sub.right_back_connect_id == 0 && iter_sub.right_front_connect_id == 0 && lane_ids.right_back_connect_id == 0 && lane_ids.right_front_connect_id == 0){
    // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_ids.right_line_base_id: " << lane_ids.right_line_base_id<<",iter_sub.right_line_base_id: "<<iter_sub.right_line_base_id<<std::endl;    
                    if(iter.second.size()>50){
                        iter.second.pop_front();
                    }                    
                    iter.second.push_back(lane_ids);
                    lane_id = iter.first;
                    break;
                }
            }
            if(lane_id !=0){
                break;
            }
        }

        if(lane_id ==0){
            if(lane_ids_map.size()== 254){
                int id_tmp = lane_ids_map.begin()->first;
                lane_ids_map.erase(lane_ids_map.begin());
                std::deque<BevLaneIds> vec_tmp={lane_ids};
                lane_ids_map.emplace(std::make_pair(id_tmp, vec_tmp));
                lane_id = id_tmp;
            }else{
                int id_tmp = lane_ids_map.size() +1;
                std::deque<BevLaneIds> vec_tmp={lane_ids};
                lane_ids_map.emplace(std::make_pair(id_tmp, vec_tmp));
                lane_id = id_tmp;
            }            
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << "," << "final lane_ids_map.size(): " << lane_ids_map.size()<<std::endl;    
    // for(auto & iter: lane_ids_map){
    //     std::cout<<"final lane_iter ,";
    //     for(auto & iter_sub: iter.second){
    //         std::cout<<",||,"<<iter_sub.left_back_connect_id<<" ,"<<iter_sub.left_front_connect_id<<" ,"<<iter_sub.left_line_base_id
    //         <<" ,"<<iter_sub.right_back_connect_id<<" ,"<<iter_sub.right_front_connect_id<<" ,"<<iter_sub.right_line_base_id<<std::endl;            
    //     }

    // }
}
void WrapperOutput::PostProcess(datatype_efm::s_MapLane_t& map_lane, SEhpOutputLoc& loc){
    //保存上个周期的车道中心线和定位信息
    std::deque<std::pair<int, EFMRefLinePoints>> current_cycle_center_lines;
    for(auto & lane:map_lane.Array_lanes_5){
        if(lane.enable_flag == true){
            EFMRefLinePoints points{};
            for(int i = 0; i<lane.center_line.pntSize; i++){
                EFMPoint p(lane.center_line.Array_linePnt_121[i].dX, lane.center_line.Array_linePnt_121[i].dY);
                points.push_back(p);
            }
            current_cycle_center_lines.push_back(std::make_pair(lane.ref_line.pntSize, points));
        }
    }

    local_map_ptr_->SmoothCenterLineForLastCycle(last_cycle_loc_, last_cycle_center_lines_,loc, current_cycle_center_lines);

    for(auto & lane:map_lane.Array_lanes_5){
        for(auto& line:current_cycle_center_lines){
            if(line.first == lane.ref_line.pntSize){
                int line_pnt_num = 0;
                for (int i = 0; i < line.second.size() && line_pnt_num < lane.center_line.Array_linePnt_121.size();i++) {
                    lane.center_line.Array_linePnt_121[line_pnt_num].dX = static_cast<float>(line.second[i].x);
                    lane.center_line.Array_linePnt_121[line_pnt_num].dY = static_cast<float>(line.second[i].y);
                    line_pnt_num++;
                } 
                lane.center_line.pntSize = line_pnt_num;
                if (lane.center_line.pntSize > 0) {
                    lane.enable_flag = true;
                } 
            }
        }

    }

    last_cycle_loc_ = loc;
    last_cycle_center_lines_ = current_cycle_center_lines;

}

}
