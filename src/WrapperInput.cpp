#include "WrapperInput.h"

namespace NoMapEFM{

bool WrapperInput::Execute(const SEhpOutputLinkList& links, const SEhpOutputPathList& paths, const SEhpOutputLoc& lane_loc, const MapRawDataMap& sd_data_index,
                    const datatype_fusion::s_FusionLanes_t& lanes_msg, std::vector<BevLineInnerS>& bev_lines, std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, std::vector<std::pair<int,double>>& road_split_dir_vec) {
    bev_lines.clear();
    path_point_body_with_offset.clear();
    //离散成点，并赋值原始值
    BevLinePolyToPoints(lanes_msg, bev_lines);

    CombineLines(bev_lines);    

    SEhpOutputPath path_res;
    if(GetPath(lane_loc.path_id_, paths, path_res) == true){
        CombinePathPoint(lane_loc,links.ehp_output_link_list, path_res, sd_data_index, path_point_body_with_offset);
        JudgeRoadSplitDir(lane_loc, links.ehp_output_link_list,  paths, path_res, sd_data_index, road_split_dir_vec);
    }

#ifdef WINPUT
std::cout << __FILE__ << "," << __LINE__ << "," << "paths.size: " <<paths.ehp_output_path_list.size()<<std::endl; 
    for(auto path:paths.ehp_output_path_list){
        std::cout<<" ,<path_id:"<<path.path_id_<<" ,parent_path_id_:"<<path.parent_path_id_<<" ,InParentOffset_:"<<path.InParentOffset_<<" >"
        <<",ego_offest:"<<static_cast<double>(path.InParentOffset_)-static_cast<double>(lane_loc.offset_)<<std::endl;
    }

    {
    std::stringstream ss, ss1,ss2;
    ss<<"route_x = [";
    ss1<<"route_y = [";
    ss2<<" offset: ";
    for(int j=0; j<path_point_body_with_offset.size();j++){
        if(j == path_point_body_with_offset.size()-1){
            ss<< path_point_body_with_offset[j].first.x;
            ss2<< path_point_body_with_offset[j].second;
        }else{
            ss<< path_point_body_with_offset[j].first.x<<" ,";
            ss2<< path_point_body_with_offset[j].second<<" ,";
        }            
    }
    for(int j=0; j<path_point_body_with_offset.size();j++){
        if(j == path_point_body_with_offset.size()-1){
            ss1<< path_point_body_with_offset[j].first.y;
        }else{
            ss1<< path_point_body_with_offset[j].first.y<<" ,";
        }            
    }
    ss<<"]"<<std::endl;
    ss1<<"]"<<std::endl;
    ss2<< std::endl; 
    std::cout<<ss.str();   
    std::cout<<ss1.str();    
    std::cout<<ss2.str();
    }

    //
    std::cout << __FILE__ << "," << __LINE__ << "," << "road_split_dir_vec.size: " <<road_split_dir_vec.size()<<std::endl; 
    for(auto& iter:road_split_dir_vec){
        std::cout<<" ,<dir:"<<iter.first<<" ,dist:"<<iter.second<<">";
    }
    std::cout<<std::endl;
#endif
    return true;
}

void WrapperInput::CombineLines(std::vector<BevLineInnerS>& bev_lines) {
    
    for(auto& bev_line: bev_lines){
        EFMRefLinePoints line_points{};
        if (bev_line.minus_valid == true) {
            for (auto p : bev_line.minus_line) {
                line_points.push_back(p);
            }
        }
        if (bev_line.first_valid == true) {
            for (auto p : bev_line.first_line) {
                line_points.push_back(p);
            }
        }
        if (bev_line.sec_valid == true) {
            for (auto p : bev_line.sec_line) {
                line_points.push_back(p);
            }
        }
        bev_line.total_line = line_points;
    }

}

void WrapperInput::BevLinePolyToPoints(const datatype_fusion::s_FusionLanes_t& lanes_msg,
                                    std::vector<BevLineInnerS>& bev_lines) {
    bev_lines.clear();
    for (int i = 0; i < lanes_msg.FusionLanes.Array_Lanes_50.size(); i++) {
        BevLineInnerS bev_line_inners;
#ifdef WINPUT
        std::cout << __FILE__ << "," << __LINE__ << "," << "Array_Lanes_50[ " <<i<<"]:valid: "
               <<(int)lanes_msg.FusionLanes.Array_Lanes_50[i].valid<< std::endl;
#endif
        if (lanes_msg.FusionLanes.Array_Lanes_50[i].valid) {
#ifdef WINPUT
            std::cout << " ,id:"<< lanes_msg.FusionLanes.Array_Lanes_50[i].CamObjId;
            std::cout << " ,MinusStartPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].MinusStartPoint;
            std::cout << " ,MinusEndPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].MinusEndPoint<<std::endl;
            std::cout << " ,IsFirstValid:"<<(int)lanes_msg.FusionLanes.Array_Lanes_50[i].IsFirstValid;
            std::cout << " ,C0First:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C0First;
            std::cout << " ,C1First:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C1First;
            std::cout << " ,C2First:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C2First;
            std::cout << " ,C3First:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C3First;
            std::cout << " ,FirstStartPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].FirstStartPoint;
            std::cout << " ,FirstEndPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].FirstEndPoint<<std::endl;
            std::cout << " ,IsSecValid:"<<(int)lanes_msg.FusionLanes.Array_Lanes_50[i].IsSecValid;
            std::cout << " ,C0Sec:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C0Sec;
            std::cout << " ,C1Sec:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C1Sec;
            std::cout << " ,C2Sec:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C2Sec;
            std::cout << " ,C3Sec:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].C3Sec;
            std::cout << " ,SecStartPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].SecStartPoint;
            std::cout << " ,FirstEndPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].FirstEndPoint<<std::endl;
            std::cout << " ,TypeLane:"<<(int)lanes_msg.FusionLanes.Array_Lanes_50[i].TypeLane;
            std::cout << " ,TypChgPoint:"<<lanes_msg.FusionLanes.Array_Lanes_50[i].TypChgPoint;
            std::cout << " ,TypAftChgPoint:"<<(int)lanes_msg.FusionLanes.Array_Lanes_50[i].TypAftChgPoint<<std::endl;            
#endif
            double back_offset = 0;
            bev_line_inners.id = lanes_msg.FusionLanes.Array_Lanes_50[i].CamObjId;
            bev_line_inners.md_qly = lanes_msg.FusionLanes.Array_Lanes_50[i].MdlQlyLane;
            bev_line_inners.lane_location_type = lanes_msg.FusionLanes.Array_Lanes_50[i].lane_locationType;
            bev_line_inners.line_type = static_cast<int>(lanes_msg.FusionLanes.Array_Lanes_50[i].TypeLane);
            bev_line_inners.typ_chg_point = lanes_msg.FusionLanes.Array_Lanes_50[i].TypChgPoint;
            bev_line_inners.typ_aft_chg_point = static_cast<int>(lanes_msg.FusionLanes.Array_Lanes_50[i].TypAftChgPoint);
            float bev_x = int(lanes_msg.FusionLanes.Array_Lanes_50[i].MinusStartPoint / 2.5f) * 2.5f;
            if (lanes_msg.FusionLanes.Array_Lanes_50[i].MinusStartPoint < 0.0f) {
                bev_line_inners.C0Minus = lanes_msg.FusionLanes.Array_Lanes_50[i].C0Minus;
                bev_line_inners.C1Minus = lanes_msg.FusionLanes.Array_Lanes_50[i].C1Minus;
                bev_line_inners.C2Minus = lanes_msg.FusionLanes.Array_Lanes_50[i].C2Minus;
                bev_line_inners.C3Minus = lanes_msg.FusionLanes.Array_Lanes_50[i].C3Minus;
                bev_line_inners.MinusStartPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].MinusStartPoint;
                bev_line_inners.MinusEndPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].MinusEndPoint;
                bev_line_inners.minus_valid = true;
                if(lanes_msg.FusionLanes.Array_Lanes_50[i].MinusEndPoint<-5){
                    back_offset = -5;
                }
                
                for (; bev_x < lanes_msg.FusionLanes.Array_Lanes_50[i].MinusEndPoint+back_offset; bev_x += 2.5f) {
                    bev_line_inners.minus_line.push_back(
                        EFMPoint(bev_x, lanes_msg.FusionLanes.Array_Lanes_50[i].C3Minus * bev_x * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C2Minus * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C1Minus * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C0Minus));
                }
            }
#ifdef WINPUT
            if(bev_line_inners.minus_line.size()>0){
                std::cout << " ,minus_line.back().x:"<< bev_line_inners.minus_line.back().x<<" ,bev_x:"<<bev_x<<" ,back_offset:"<<back_offset<<std::endl;
            }
#endif            

            // if (abs(lanes_msg.FusionLanes.Array_Lanes_50[i].C0First) > 0.0f) {
            if(lanes_msg.FusionLanes.Array_Lanes_50[i].IsFirstValid 
                &&lanes_msg.FusionLanes.Array_Lanes_50[i].FirstEndPoint>lanes_msg.FusionLanes.Array_Lanes_50[i].FirstStartPoint
                &&(lanes_msg.FusionLanes.Array_Lanes_50[i].C0First !=0 ||lanes_msg.FusionLanes.Array_Lanes_50[i].C1First !=0 ||lanes_msg.FusionLanes.Array_Lanes_50[i].C2First !=0||lanes_msg.FusionLanes.Array_Lanes_50[i].C3First !=0)){
                bev_line_inners.C0First = lanes_msg.FusionLanes.Array_Lanes_50[i].C0First;
                bev_line_inners.C1First = lanes_msg.FusionLanes.Array_Lanes_50[i].C1First;
                bev_line_inners.C2First = lanes_msg.FusionLanes.Array_Lanes_50[i].C2First;
                bev_line_inners.C3First = lanes_msg.FusionLanes.Array_Lanes_50[i].C3First;
                bev_line_inners.FirstStartPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].FirstStartPoint;
                bev_line_inners.FirstEndPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].FirstEndPoint;
                bev_line_inners.first_valid = true;
                for (; bev_x <= lanes_msg.FusionLanes.Array_Lanes_50[i].FirstEndPoint; bev_x += 2.5f) {
                    if (bev_x < lanes_msg.FusionLanes.Array_Lanes_50[i].FirstStartPoint+back_offset) continue;
                    bev_line_inners.first_line.push_back(
                        EFMPoint(bev_x, lanes_msg.FusionLanes.Array_Lanes_50[i].C3First * bev_x * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C2First * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C1First * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C0First));
                }
            }
#ifdef WINPUT
            if(bev_line_inners.first_line.size()>0){
                std::cout << " ,first_line.front().x:"<< bev_line_inners.first_line.front().x<<std::endl;
            }
#endif 
            // if (abs(lanes_msg.FusionLanes.Array_Lanes_50[i].C0Sec) > 0.0f) {
            if(lanes_msg.FusionLanes.Array_Lanes_50[i].IsSecValid
                &&lanes_msg.FusionLanes.Array_Lanes_50[i].SecEndPoint>lanes_msg.FusionLanes.Array_Lanes_50[i].SecStartPoint
                &&(lanes_msg.FusionLanes.Array_Lanes_50[i].C0Sec !=0 ||lanes_msg.FusionLanes.Array_Lanes_50[i].C1Sec !=0 ||lanes_msg.FusionLanes.Array_Lanes_50[i].C2Sec !=0||lanes_msg.FusionLanes.Array_Lanes_50[i].C3Sec !=0)){
                bev_line_inners.C0Sec = lanes_msg.FusionLanes.Array_Lanes_50[i].C0Sec;
                bev_line_inners.C1Sec = lanes_msg.FusionLanes.Array_Lanes_50[i].C1Sec;
                bev_line_inners.C2Sec = lanes_msg.FusionLanes.Array_Lanes_50[i].C2Sec;
                bev_line_inners.C3Sec = lanes_msg.FusionLanes.Array_Lanes_50[i].C3Sec;
                bev_line_inners.SecStartPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].SecStartPoint;
                bev_line_inners.SecEndPoint = lanes_msg.FusionLanes.Array_Lanes_50[i].SecEndPoint;
                bev_line_inners.sec_valid = true;
                for (; bev_x <= lanes_msg.FusionLanes.Array_Lanes_50[i].SecEndPoint; bev_x += 2.5f) {
                    if (bev_x < lanes_msg.FusionLanes.Array_Lanes_50[i].SecStartPoint) continue;
                    bev_line_inners.sec_line.push_back(
                        EFMPoint(bev_x, lanes_msg.FusionLanes.Array_Lanes_50[i].C3Sec * bev_x * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C2Sec * bev_x * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C1Sec * bev_x +
                                              lanes_msg.FusionLanes.Array_Lanes_50[i].C0Sec));
                }
            }
            bev_lines.push_back(bev_line_inners);
        }
    }
}

bool WrapperInput::GetPath(uint32_t path_id, const SEhpOutputPathList& raw_paths, SEhpOutputPath& path_res){
    for(auto& path: raw_paths.ehp_output_path_list){
        if(path.path_id_ == path_id){
            path_res = path;
            return true;
        }
    }
    return false;
}

bool WrapperInput::CombinePathPoint(const SEhpOutputLoc& lane_loc, const std::vector<SEhpOutputLink>& link_list, const SEhpOutputPath& path, const MapRawDataMap& data_index, 
                                    std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset){
    // path_point_body.clear();
    // std::cout<<__FILE__ << "," << __LINE__ <<","<<"loc.link_id: "<<lane_loc.link_id_<<" ,loc.offset:"<<lane_loc.offset_<<std::endl;
    std::stringstream ss;
    // ss<< __FILE__ << "," << __LINE__ << "," <<"route list: ";
    path_point_body_with_offset.clear();
    DoublePosePoint ego_pose(lane_loc.lon_, lane_loc.lat_, 0, lane_loc.heading_, 0, 0);
    for(auto& link: path.link_offsets_){
        if(data_index.link_path_offset_id_index_map.find(link.path_offset_id_)!= data_index.link_path_offset_id_index_map.end()){
            int index_tmp = data_index.link_path_offset_id_index_map.at(link.path_offset_id_);
            if(index_tmp < link_list.size() && index_tmp>= 0 && link_list[index_tmp].e_offset_>lane_loc.offset_){
                // ss<< "< "<<link_list[index_tmp].id_<<","<<link_list[index_tmp].e_offset_<<" ,in_links: "<< link_list[index_tmp].all_in_links_.size()<<">,";
                if(link_list[index_tmp].frc_ != LINK_FRC::LINK_FRC_MOTORWAY && link_list[index_tmp].frc_ != LINK_FRC::LINK_FRC_URBAN_MOTORWAY){
                    break;
                }
                std::vector<EFMPoint> geoms{};
                std::vector<EFMPoint> body_res{};
                geoms.insert(geoms.begin(), link_list[index_tmp].geoms_.begin(), link_list[index_tmp].geoms_.end());
                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(geoms,ego_pose,body_res);
                double link_s_offset = static_cast<double>(link.s_offset_)/100.0 -  static_cast<double>(lane_loc.offset_)/100.0;
                double offset_tmp = link_s_offset;
                for(int i=0;i<body_res.size();i++){
                    if(i==0){

                    }else{                               
                        offset_tmp += sqrt(pow(body_res[i].x - body_res[i-1].x, 2) + pow(body_res[i].y - body_res[i-1].y, 2));
                    }
                    path_point_body_with_offset.push_back(std::make_pair(body_res[i],offset_tmp));                   
                }
                // path_point_body.insert(path_point_body.end(), body_res.begin(), body_res.end());
            }            
        }else{
            break;
        }
    }
// std::cout<<ss.str()<<std::endl;
//         {
//     std::stringstream ss, ss1,ss2;
//     ss<<"raw_route_x = [";
//     ss1<<"raw_route_y = [";
//     ss2<<" offset: ";
//     for(int j=0; j<path_point_body_with_offset.size();j++){
//         if(j == path_point_body_with_offset.size()-1){
//             ss<< path_point_body_with_offset[j].first.x;
//             ss2<< path_point_body_with_offset[j].second;
//         }else{
//             ss<< path_point_body_with_offset[j].first.x<<" ,";
//             ss2<< path_point_body_with_offset[j].second<<" ,";
//         }            
//     }
//     for(int j=0; j<path_point_body_with_offset.size();j++){
//         if(j == path_point_body_with_offset.size()-1){
//             ss1<< path_point_body_with_offset[j].first.y;
//         }else{
//             ss1<< path_point_body_with_offset[j].first.y<<" ,";
//         }            
//     }
//     ss<<"]"<<std::endl;
//     ss1<<"]"<<std::endl;
//     ss2<< std::endl; 
//     std::cout<<ss.str();   
//     std::cout<<ss1.str();    
//     std::cout<<ss2.str();
//     }

    //再对拼号的路径中，删掉merge 和split前后的50m
    //找到两段 split merge需要删除的段
    double offset_threhold_start=0, offset_threhold_end = 0;
    double offset_threhold_start2=0, offset_threhold_end2 = 0;
    bool find_first_section = false, find_sec_section = false;
    for(int i =0; i<path.link_offsets_.size(); i++){
        auto& link = path.link_offsets_[i];
        if(data_index.link_path_offset_id_index_map.find(link.path_offset_id_)!= data_index.link_path_offset_id_index_map.end()){
            int index_tmp = data_index.link_path_offset_id_index_map.at(link.path_offset_id_);
            if(index_tmp < link_list.size() && index_tmp>= 0 && link_list[index_tmp].e_offset_>lane_loc.offset_){  

                if(link_list[index_tmp].all_in_links_.size()>1 || link_list[index_tmp].all_out_links_.size()>1){
                    if(find_first_section == false){
                        offset_threhold_start = static_cast<double>(link_list[index_tmp].e_offset_)/100.0 - static_cast<double>(lane_loc.offset_)/100.0- 50;
                        offset_threhold_end = static_cast<double>(link_list[index_tmp].e_offset_)/100.0 -static_cast<double>(lane_loc.offset_)/100.0+ 50;
                        find_first_section = true;                        
                    }else{
                        offset_threhold_start2 = static_cast<double>(link_list[index_tmp].e_offset_)/100.0- static_cast<double>(lane_loc.offset_)/100.0- 50;
                        offset_threhold_end2 = static_cast<double>(link_list[index_tmp].e_offset_)/100.0-static_cast<double>(lane_loc.offset_)/100.0 + 50;
                        find_sec_section = true;  
                        break;                       
                    }
                }
            }
        }
    }
    // std::cout<<__FILE__ << "," << __LINE__ <<","<<"offset_threhold_start: "<<offset_threhold_start<<" ,offset_threhold_end:"<<offset_threhold_end<<" ,find_first_section:"<<(int)find_first_section<<std::endl;
    // std::cout<<__FILE__ << "," << __LINE__ <<","<<"offset_threhold_start2: "<<offset_threhold_start2<<" ,offset_threhold_end2:"<<offset_threhold_end2<<" ,find_first_section:"<<(int)find_sec_section<<std::endl;

    std::vector<std::pair<EFMPoint,double>> res{};
    for(auto& iter:path_point_body_with_offset){
        if(iter.second >= offset_threhold_start && iter.second <= offset_threhold_end &&  find_first_section == true){
            if(res.size()>0){
                break;
            }
        }else if(iter.second >= offset_threhold_start2 && iter.second <= offset_threhold_end2 &&  find_sec_section == true){
            if(res.size()>0){
                break;
            }            
        }else{
            res.push_back(iter);
        }
       
    }
    path_point_body_with_offset = res;
    // std::cout<<ss.str()<<std::endl;
    return true;
}

bool WrapperInput::JudgeRoadSplitDir(const SEhpOutputLoc& lane_loc, const std::vector<SEhpOutputLink>& link_list,  const SEhpOutputPathList& raw_paths, const SEhpOutputPath& ego_path, const MapRawDataMap& data_index,std::vector<std::pair<int,double>>& road_split_dir_vec){
    road_split_dir_vec.clear();

    for(int link_index=1;link_index< ego_path.link_offsets_.size();link_index++){
        auto link = ego_path.link_offsets_[link_index-1];
        uint64_t ego_next_link_path_offset_id = ego_path.link_offsets_[link_index].path_offset_id_;
        if(data_index.link_path_offset_id_index_map.find(link.path_offset_id_)!= data_index.link_path_offset_id_index_map.end()){
            int index_tmp = data_index.link_path_offset_id_index_map.at(link.path_offset_id_);
            if(index_tmp < link_list.size() && index_tmp>= 0 && link_list[index_tmp].all_out_links_.size()>1){
                std::vector<LinkLaneInfo> lanes_info = link_list[index_tmp].link_lane_info_list_;

                // bubble sort, lane_num min->max
                for (int i = 0; i < lanes_info.size(); i++) {
                    for (int j = 0; j < lanes_info.size() - 1 - i; j++) {
                        if (lanes_info[j].lane_num > lanes_info[j + 1].lane_num) {
                            LinkLaneInfo tmp;
                            tmp = lanes_info[j];
                            lanes_info[j] = lanes_info[j + 1];
                            lanes_info[j + 1] = tmp;
                        }
                    }
                }
                
                //如果道路分歧处的lane,连接的下一段link不是ego_path里的，那么如果another_path_lane_num小于 ego_path上的lane_num，那么raod split的方向向左
                uint8_t to_another_path_lane_num = 0;
                uint8_t to_ego_path_lane_num = 0;
                bool finish_f =false;
                for(auto& lane: lanes_info){
                    for(auto& next_lane_id: lane.next_lane_ids){
                        if(data_index.lane_id_index_map.find(next_lane_id) != data_index.lane_id_index_map.end()){
                            //找到next_lane的link id
                            int next_link_index = data_index.lane_id_index_map.at(next_lane_id).at(0);                   
                            uint64_t next_link_path_offset_id = link_list[next_link_index].path_offset_id_;
                            if(next_link_path_offset_id != ego_next_link_path_offset_id){//next_lane不在 ego_path上，记录lane_num
                                to_another_path_lane_num = lane.lane_num;
                            }else{
                                to_ego_path_lane_num = lane.lane_num;
                            }

                            if(to_another_path_lane_num !=0 && to_ego_path_lane_num!= 0){
                                double split_dist = static_cast<double>(link.e_offset_)/100.0 - static_cast<double>(lane_loc.offset_)/100.0;
                                if(to_another_path_lane_num < to_ego_path_lane_num){
                                    road_split_dir_vec.push_back(std::make_pair(1, split_dist));
                                }else{
                                    road_split_dir_vec.push_back(std::make_pair(2, split_dist));
                                }
                                finish_f = true;
                                break;
                            }

                        }
                    }
                    if(finish_f == true){
                        break;
                    }
                }
            }
        }
    }

    return true;
}
    
}
