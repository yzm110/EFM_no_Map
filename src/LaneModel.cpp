#include "LaneModel.h"

namespace NoMapEFM{

bool LaneModel::Execute(std::vector<BevLineInnerS>& bev_lines, BevLaneElementGroupSet& bev_lane_group_set, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info,
                        const StoredInfo& last_cycle_info, const MapRawDataMap& sd_data_index, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, const std::vector<std::pair<int,double>>& road_split_dir_vec){
    // std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines.size: " <<bev_lines.size() <<std::endl;
    sd_data_index_ = std::make_shared<const MapRawDataMap>(sd_data_index);

    CalBevLineAttribute(bev_lines);
#ifdef ALL_LINE
    std::cout << __FILE__ << "," << __LINE__ << "," << "after CalBevLineAttribute bev_lines.size: " <<bev_lines.size() <<std::endl;
    // for (int i = 0; i < bev_lines.size(); i++) {
    //     std::cout << " bev_lines[ " << i << " ]::  id:"<<bev_lines[i].id<<" ,line.line_is_available: "<<(int)bev_lines[i].line_is_available
    //     <<" ,line.is_inside: "<<(int)bev_lines[i].ego_is_inside<< " ,line.ego_side: "<<bev_lines[i].ego_side
    //     <<" ,line.length: "<<bev_lines[i].line_length<< " ,line.total_point.size: "<<bev_lines[i].total_line.size()<<std::endl;
    // }
    for (int i = 0; i < bev_lines.size(); i++) {
        std::cout << " bev_lines[ " << i << " ]::  id:"<<bev_lines[i].id<<" ,line.line_is_available: "<<(int)bev_lines[i].line_is_available
        <<" ,line.lane_location_type: "<<bev_lines[i].lane_location_type<< " ,line.line_type: "<<bev_lines[i].line_type
        <<" ,line.typ_aft_chg_point: "<<bev_lines[i].typ_aft_chg_point<< " ,line.typ_chg_point: "<<bev_lines[i].typ_chg_point<<" ,attach_curb_side:"<<bev_lines[i].attached_curb_side<<std::endl;
    }    
#endif
    SortBevLine(bev_lines,bev_lines_in_side_ego_,bev_lines_not_in_side_ego_);
    ConnectBevLines(bev_lines_in_side_ego_,bev_lines, connected_lines_, multi_line_index_);
#ifdef LM_CONNECT_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw connected_lines_.size: " <<connected_lines_.size() <<std::endl;
    for(int i = 0; i<connected_lines_.size();i++){
        std::cout << "," << "raw connected_lines_[" <<i<<"]:: base_id:"<<connected_lines_[i].base_line_ptr->id;
        std::cout<<" ,front_connrct_id: ";
        for(auto iter:connected_lines_[i].front_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }
        std::cout<<" ,back_connrct_id: ";
        for(auto iter:connected_lines_[i].back_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }
        std::cout<<std::endl;
        std::cout<<" ,split_merge_type: "<<connected_lines_[i].split_merge_type<< " ,split_side: "<<connected_lines_[i].split_side <<" , split_point:"<<connected_lines_[i].split_point.x <<","<< connected_lines_[i].split_point.y << " ,merge_side: "<<connected_lines_[i].merge_side <<" , merge_point:"<<connected_lines_[i].merge_point.x <<","<< connected_lines_[i].merge_point.y;
        std::cout<<std::endl;
    }
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw multi_line_index_.size: " <<multi_line_index_.size() <<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw multi_line_index_: " ;
    for(auto index:multi_line_index_){
        std::cout<<",<"<<index.first<< ", "<<index.second<<"> ";
    }
    std::cout<<std::endl;
#endif
    ConnectLinePostProc(connected_lines_, multi_line_index_);
    //PlotConnectLines(connected_lines_);
    // FixConnectLines(connected_lines_);//2.5一个点
#ifdef LM_CONNECT_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << " connected_lines_.size: " <<connected_lines_.size() <<std::endl;
    for(int i = 0; i<connected_lines_.size();i++){
        std::cout << "," << " connected_lines_[" <<i<<"]:: base_id:"<<connected_lines_[i].base_line_ptr->id;
        std::cout<<" ,front_connrct_id: ";
        for(auto iter:connected_lines_[i].front_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }
        std::cout<<" ,back_connrct_id: ";
        for(auto iter:connected_lines_[i].back_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }
        std::cout<<std::endl;
        std::cout<<" ,split_merge_type: "<<connected_lines_[i].split_merge_type<< " ,split_side: "<<connected_lines_[i].split_side <<" , split_point:"<<connected_lines_[i].split_point.x <<","<< connected_lines_[i].split_point.y << " ,merge_side: "<<connected_lines_[i].merge_side <<" , merge_point:"<<connected_lines_[i].merge_point.x <<","<< connected_lines_[i].merge_point.y;
        std::cout<<std::endl;
    }
    std::cout << __FILE__ << "," << __LINE__ << "," << " multi_line_index_.size: " <<multi_line_index_.size() <<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " multi_line_index_: " ;
    for(auto index:multi_line_index_){
        std::cout<<",<"<<index.first<< ", "<<index.second<<"> ";
    }
    std::cout<<std::endl;
#endif
    GenerateLaneGroupSet(bev_lines, connected_lines_, multi_line_index_,path_point_body_with_offset, road_split_dir_vec, bev_lane_group_set, ego_lane_left_index_, ego_lane_right_index_);

    // GenerateSideCurb(connected_lines_, bev_lane_group_set);
    
    BevLineInnerS* ego_left_line = nullptr;
    BevLineInnerS* ego_right_line = nullptr;
    if(ego_lane_left_index_>=0 && ego_lane_left_index_<connected_lines_.size()){
        ego_left_line = connected_lines_[ego_lane_left_index_].base_line_ptr;
    }
    if(ego_lane_left_index_>=0 && ego_lane_left_index_<connected_lines_.size()){
        ego_right_line = connected_lines_[ego_lane_right_index_].base_line_ptr;
    } 

    const BevLaneElementGroup *ego_bev_lane_group = nullptr;
    for(auto& lane_group : bev_lane_group_set){
        if(lane_group.second.size() > 0){
            if(lane_group.first == 0){
                ego_bev_lane_group = &lane_group.second;
            }
        }
    }  
    GetEgoLaneIdx(bev_lines_in_side_ego_, ego_left_line, ego_right_line,lane_loc, ele_group,loc_res_info,ego_bev_lane_group,last_cycle_info);
    return true;
}

void LaneModel::PlotEgoLaneGroup(const BevLaneElementGroup& lane_group){
    std::array<std::vector<double>,3> left_line_x{};
    std::array<std::vector<double>,3> left_line_y{};
    std::array<std::vector<double>,3> right_line_x{};
    std::array<std::vector<double>,3> right_line_y{};
    // ZTEXT("EGOLANE", "lane_group: ", 16, 21, "lane_group: {}", lane_group.size());
    for(int i = 0; i<lane_group.size() && i<3;i++){
        for(int p_i=0 ; p_i<lane_group[i].left_line_points.size();p_i++){
            auto & p = lane_group[i].left_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].left_line_points.size()-1 || p_i%3==0){
                left_line_x[i].push_back(p.x);
                left_line_y[i].push_back(p.y);
            }

        }
        for(int p_i=0 ; p_i<lane_group[i].right_line_points.size();p_i++){
            auto & p = lane_group[i].right_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].right_line_points.size()-1 || p_i%3==0){
                right_line_x[i].push_back(p.x);
                right_line_y[i].push_back(p.y);
            }

        }       
    }
    for(int i = 0; i<3;i++){
        switch(i){
            case 0:
                // ZPLOTXYF("EGOLANE", "-purple3", left_line_x[0], left_line_y[0]);
                // ZPLOTXYF("EGOLANE", "-blue3", right_line_x[0], right_line_y[0]);
                break;
            case 1:
                // ZPLOTXYF("EGOLANE", "-green3", left_line_x[1], left_line_y[1]);
                // ZPLOTXYF("EGOLANE", "-brown3", right_line_x[1], right_line_y[1]);
                break;
            case 2:
                // ZPLOTXYF("EGOLANE", "-orange3", left_line_x[2], left_line_y[2]);
                // ZPLOTXYF("EGOLANE", "-pink3", right_line_x[2], right_line_y[2]);
                break;
        }
    }
    
}

void LaneModel::PlotRightLaneGroup(const BevLaneElementGroup& lane_group){
    std::array<std::vector<double>,3> left_line_x{};
    std::array<std::vector<double>,3> left_line_y{};
    std::array<std::vector<double>,3> right_line_x{};
    std::array<std::vector<double>,3> right_line_y{};
    // ZTEXT("RiLANE", "lane_group: ", 16, 21, "lane_group: {}", lane_group.size());
    for(int i = 0; i<lane_group.size() && i<3;i++){
        for(int p_i=0 ; p_i<lane_group[i].left_line_points.size();p_i++){
            auto & p = lane_group[i].left_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].left_line_points.size()-1 || p_i%3==0){
                left_line_x[i].push_back(p.x);
                left_line_y[i].push_back(p.y);
            }

        }
        for(int p_i=0 ; p_i<lane_group[i].right_line_points.size();p_i++){
            auto & p = lane_group[i].right_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].right_line_points.size()-1 || p_i%3==0){
                right_line_x[i].push_back(p.x);
                right_line_y[i].push_back(p.y);
            }

        }      
    }
    for(int i = 0; i<3;i++){
        switch(i){
            case 0:
                // ZPLOTXYF("RiLANE", "-purple3", left_line_x[0], left_line_y[0]);
                // ZPLOTXYF("RiLANE", "-blue3", right_line_x[0], right_line_y[0]);
                break;
            case 1:
                // ZPLOTXYF("RiLANE", "-green3", left_line_x[1], left_line_y[1]);
                // ZPLOTXYF("RiLANE", "-brown3", right_line_x[1], right_line_y[1]);
                break;
            case 2:
                // ZPLOTXYF("RiLANE", "-orange3", left_line_x[2], left_line_y[2]);
                // ZPLOTXYF("RiLANE", "-pink3", right_line_x[2], right_line_y[2]);
                break;
        }
    }
    
}

void LaneModel::PlotLeftLaneGroup(const BevLaneElementGroup& lane_group){
    std::array<std::vector<double>,3> left_line_x{};
    std::array<std::vector<double>,3> left_line_y{};
    std::array<std::vector<double>,3> right_line_x{};
    std::array<std::vector<double>,3> right_line_y{};
    // ZTEXT("LeLANE", "lane_group: ", 16, 21, "lane_group: {}", lane_group.size());
    for(int i = 0; i<lane_group.size() && i<3;i++){
        for(int p_i=0 ; p_i<lane_group[i].left_line_points.size();p_i++){
            auto & p = lane_group[i].left_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].left_line_points.size()-1 || p_i%3==0){
                left_line_x[i].push_back(p.x);
                left_line_y[i].push_back(p.y);
            }

        }
        for(int p_i=0 ; p_i<lane_group[i].right_line_points.size();p_i++){
            auto & p = lane_group[i].right_line_points[p_i];
            if(p_i==0 || p_i==lane_group[i].right_line_points.size()-1 || p_i%3==0){
                right_line_x[i].push_back(p.x);
                right_line_y[i].push_back(p.y);
            }

        }        
    }
    for(int i = 0; i<3;i++){
        switch(i){
            case 0:
                // ZPLOTXYF("LeLANE", "-purple3", left_line_x[0], left_line_y[0]);
                // ZPLOTXYF("LeLANE", "-blue3", right_line_x[0], right_line_y[0]);
                break;
            case 1:
                // ZPLOTXYF("LeLANE", "-green3", left_line_x[1], left_line_y[1]);
                // ZPLOTXYF("LeLANE", "-brown3", right_line_x[1], right_line_y[1]);
                break;
            case 2:
                // ZPLOTXYF("LeLANE", "-orange3", left_line_x[2], left_line_y[2]);
                // ZPLOTXYF("LeLANE", "-pink3", right_line_x[2], right_line_y[2]);
                break;
        }
    }
    
}

void LaneModel::PlotConnectLines(const std::vector<BevLineInnerConnect>& connected_lines){
    std::array<std::vector<double>,20> left_line_x{};
    std::array<std::vector<double>,20> left_line_y{};
    for(int i = 0; i<connected_lines.size() && i<20;i++){
        for(int p_i=0 ; p_i<connected_lines[i].line_point.size();p_i++){
            auto & p = connected_lines[i].line_point[p_i];
            if(p_i==0 || p_i==connected_lines[i].line_point.size()-1 || p_i%3==0){
            left_line_x[i].push_back(p.x);
            left_line_y[i].push_back(p.y);
            }
        }
        
    }
    // for(int i = 0; i<20;i++){
    //     switch(i){
    //         case 0:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-purple3", left_line_x[0], left_line_y[0]);
    //             break;
    //         case 1:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-blue3", left_line_x[1], left_line_y[1]);
    //             break;
    //         case 2:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-green3", left_line_x[2], left_line_y[2]);
    //             break;
    //         case 3:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-orange3", left_line_x[3], left_line_y[3]);
    //             break;
    //         case 4:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-brown3", left_line_x[4], left_line_y[4]);
    //             break;
    //         case 5:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-pink3", left_line_x[5], left_line_y[5]);
    //             break;
    //         case 6:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-black3", left_line_x[6], left_line_y[6]);
    //             break;
    //         case 7:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-red3", left_line_x[7], left_line_y[7]);
    //             break;
    //         case 8:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-cyan3", left_line_x[8], left_line_y[8]);
    //             break;
    //         case 9:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-purple3", left_line_x[9], left_line_y[9]);
    //             break;
    //         case 10:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-blue3", left_line_x[10], left_line_y[10]);
    //             break;
    //         case 11:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-green3", left_line_x[11], left_line_y[11]);
    //             break;
    //         case 12:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-orange3", left_line_x[12], left_line_y[12]);
    //             break;
    //         case 13:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-brown3", left_line_x[13], left_line_y[13]);
    //             break;
    //         case 14:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-pink3", left_line_x[14], left_line_y[14]);
    //             break;
    //         case 15:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-black3", left_line_x[15], left_line_y[15]);
    //             break;
    //         case 16:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-red3", left_line_x[16], left_line_y[16]);
    //             break;
    //         case 17:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-cyan3", left_line_x[17], left_line_y[17]);
    //             break;
    //         case 18:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-purple3", left_line_x[18], left_line_y[18]);
    //             break;
    //         default:
    //             ZPLOTXYF("CONNECT_BEV_LINE", "-blue3", left_line_x[19], left_line_y[19]);
    //             break;
    //     }      
    // }

    // std::vector<double> adj_left_line_x{};   // for plot
    // std::vector<double> adj_left_line_y{};   // for plot
    // std::vector<double> right_line_x{};      // for plot
    // std::vector<double> right_line_y{};      // for plot
    // std::vector<double> adj_right_line_x{};  // for plot
    // std::vector<double> adj_right_line_y{};  // for plot
    // for (auto p : left_line) {
    //     left_line_x.push_back(p.x);
    //     left_line_y.push_back(p.y);
    // }
    // for (auto p : adj_left_line) {
    //     adj_left_line_x.push_back(p.x);
    //     adj_left_line_y.push_back(p.y);
    // }
    // for (auto p : right_line) {
    //     right_line_x.push_back(p.x);
    //     right_line_y.push_back(p.y);
    // }
    // for (auto p : adj_right_line) {
    //     adj_right_line_x.push_back(p.x);
    //     adj_right_line_y.push_back(p.y);
    // }
    // ZPLOTXYF("BEV_LINE", "-purple3", left_line_x, left_line_y);
    // ZPLOTXYF("BEV_LINE", "-green3", right_line_x, right_line_y);
    // ZPLOTXYF("BEV_LINE", "-orange3", adj_left_line_x, adj_left_line_y);
    // ZPLOTXYF("BEV_LINE", "-brown3", adj_right_line_x, adj_right_line_y);
    return;

}

void LaneModel::FixBevAbnormalAtt(BevLineInnerS& bev_line){
    if(bev_line.typ_aft_chg_point == bev_line.line_type && bev_line.typ_chg_point<10000){
        bev_line.typ_chg_point = static_cast<float>(std::numeric_limits<double>::infinity());
    }
}

void LaneModel::CalBevLineAttribute(std::vector<BevLineInnerS>& bev_lines){
    EFMPoint ego_position(0.0, 0.0);
    EFMPoint ego_position_front(5, 0.0);
    EFMPoint ego_position_front2(8, 0.0);
    for(auto& line : bev_lines){      
        if(line.total_line.size()>1){
            line.line_is_available = true;
        }else{
            line.line_is_available = false;
            continue;
        }
        // 
        FixBevAbnormalAtt(line);
        PointSLd sl_front, sl_front2;
        bool front_is_inside = false, front2_is_inside = false;
        int ego_nearest_index = -1;
        line.line_length =CommonTool::DiscretePointsMath::GetInstance()->LineLength(line.total_line);
        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(line.total_line, ego_position, line.sl_to_ego, line.ego_is_inside, ego_nearest_index);
        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(line.total_line, ego_position_front, sl_front, front_is_inside);
        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(line.total_line, ego_position_front2, sl_front2, front2_is_inside);
        line.sl_to_ego.l = (line.sl_to_ego.l+sl_front.l + sl_front2.l)*0.333333;
        // 扩充下inside的范围，前后s 5m
        if(line.ego_is_inside == false){
            if(line.sl_to_ego.s<0 && line.sl_to_ego.s>-5){
                //线起点在自车前方5m范围内，认为inside
                line.ego_is_inside = true;
            }
        //     if(line.sl_to_ego.s>line.line_length && (line.sl_to_ego.s - line.line_length)<4 ){
        //         //线终点在自车后方5m范围内，认为inside
        //         line.ego_is_inside = true;
        //     }
        }

        line.ego_side = 0;
        if(line.ego_is_inside == true){
            if(line.sl_to_ego.l<0){
                line.ego_side = 2;
            }else{
                line.ego_side = 1;
            }
        }
        for(auto& line_sub : bev_lines){
            if(line_sub.id != line.id && line.lane_location_type != 21 && line.lane_location_type != 22 && line.lane_location_type != 24
                && (line_sub.lane_location_type == 21 || line_sub.lane_location_type == 22 || line_sub.lane_location_type == 24) && line.total_line.size()>0 && line_sub.total_line.size()>0){
                //判断路沿的方向,
                // std::cout << __FILE__ << "," << __LINE__ << "," << " line id: " <<line.id<< " ,curb_id:"<<line_sub.id <<std::endl;
                double l = 10000;
                int curb_overlap_start_index = -1;
                int curb_overlap_end_index = -1;//线在路沿的点的index
                //计算线在路沿的overlap 区间
                if(CalTwoLineDist(line_sub.total_line ,line.total_line, l, 2, curb_overlap_start_index, curb_overlap_end_index)==true){
                    if(fabs(l)<1.5){
                        // std::cout << __FILE__ << "," << __LINE__ << "," << " curb_overlap_start_index: " <<curb_overlap_start_index<< " ,curb_overlap_end_index:"<<curb_overlap_end_index <<std::endl;
                        if(curb_overlap_start_index>=20 && (curb_overlap_end_index - curb_overlap_start_index)<10 || (ego_nearest_index> curb_overlap_end_index)){
                            continue;//路沿在线的f范围小25m， 且路沿在线的前方50m
                        }
                        if(l<0){
                            line.attached_curb_side =1; //路沿在zuo侧
                        }else{
                            line.attached_curb_side =2; //路沿在you侧
                        }                            
                        

                    }                    
                }

            }

        }

        if(line.id == 1176)
        {
        std::stringstream ss, ss1;
        ss<<"ego_left_line_x_1176 = [";
        ss1<<"ego_left_line_y_1176 = [";
        for(int j=0; j<line.total_line.size();j++){
            if(j == line.total_line.size()-1){
                ss<< line.total_line[j].x;
            }else{
                ss<< line.total_line[j].x<<" ,";
            }            
        }
        for(int j=0; j<line.total_line.size();j++){
            if(j == line.total_line.size()-1){
                ss1<< line.total_line[j].y;
            }else{
                ss1<< line.total_line[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        if(line.id == 1282)
        {
        std::stringstream ss, ss1;
        ss<<"ego_left_line_x_1282 = [";
        ss1<<"ego_left_line_y_1282 = [";
        for(int j=0; j<line.total_line.size();j++){
            if(j == line.total_line.size()-1){
                ss<< line.total_line[j].x;
            }else{
                ss<< line.total_line[j].x<<" ,";
            }            
        }
        for(int j=0; j<line.total_line.size();j++){
            if(j == line.total_line.size()-1){
                ss1<< line.total_line[j].y;
            }else{
                ss1<< line.total_line[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        // if(line.id == 2304)
        // {
        // std::stringstream ss, ss1;
        // ss<<"ego_left_line_x_2304= [";
        // ss1<<"ego_left_line_y_2304 = [";
        // for(int j=0; j<line.total_line.size();j++){
        //     if(j == line.total_line.size()-1){
        //         ss<< line.total_line[j].x;
        //     }else{
        //         ss<< line.total_line[j].x<<" ,";
        //     }            
        // }
        // for(int j=0; j<line.total_line.size();j++){
        //     if(j == line.total_line.size()-1){
        //         ss1<< line.total_line[j].y;
        //     }else{
        //         ss1<< line.total_line[j].y<<" ,";
        //     }            
        // }
        // ss<<"]"<<std::endl;
        // ss1<<"]"<<std::endl; 
        // std::cout<<ss.str();   
        // std::cout<<ss1.str();    
        // }
        // if(line.id == 2579)
        // {
        // std::stringstream ss, ss1;
        // ss<<"ego_left_line_x_2579= [";
        // ss1<<"ego_left_line_y_2579 = [";
        // for(int j=0; j<line.total_line.size();j++){
        //     if(j == line.total_line.size()-1){
        //         ss<< line.total_line[j].x;
        //     }else{
        //         ss<< line.total_line[j].x<<" ,";
        //     }            
        // }
        // for(int j=0; j<line.total_line.size();j++){
        //     if(j == line.total_line.size()-1){
        //         ss1<< line.total_line[j].y;
        //     }else{
        //         ss1<< line.total_line[j].y<<" ,";
        //     }            
        // }
        // ss<<"]"<<std::endl;
        // ss1<<"]"<<std::endl; 
        // std::cout<<ss.str();   
        // std::cout<<ss1.str();    
        // }
    }
    return;
}  

void LaneModel::SortBevLine(std::vector<BevLineInnerS>& bev_lines,std::vector<BevLineInnerS*>& bev_lines_in_side_ego,
                            std::vector<BevLineInnerS*>& bev_lines_not_in_side_ego){
    //step1, 把线inside ego的，按照l的大小排序，l最小的即最右的放[0]
    bev_lines_in_side_ego.clear();
    bev_lines_not_in_side_ego.clear();
    EFMPoint ego_position(0.0, 0.0);
    for(auto& line : bev_lines){      
        if(line.line_is_available == true && line.id <10000){
            if(line.ego_is_inside == true){
                bev_lines_in_side_ego.push_back(&line);
            }else{
                bev_lines_not_in_side_ego.push_back(&line);
            }            
        }
    }
// 
#ifdef LM_SORT   
    std::cout << __FILE__ << "," << __LINE__ << "," << "befor sort bev_lines_in_side_ego.size: " <<bev_lines_in_side_ego.size() <<std::endl;
    for (int i = 0; i < bev_lines_in_side_ego.size(); i++) {
        std::cout << " bev_lines_in_side_ego[ " << i << " ]::  id:"<<bev_lines_in_side_ego[i]->id<<" ,sl.l: "<<
         bev_lines_in_side_ego[i]->sl_to_ego.l<<std::endl;
    }
#endif
//    
    // bubble sort, max->min, 线从自车的最右边到最左边，自车右侧的线，自车到线的l>0
    for (int i = 0; i < bev_lines_in_side_ego.size(); i++) {
        for (int j = 0; j < bev_lines_in_side_ego.size() - 1 - i; j++) {
            if (bev_lines_in_side_ego[j]->sl_to_ego.l < bev_lines_in_side_ego[j + 1]->sl_to_ego.l) {
                BevLineInnerS* tmp;
                tmp = bev_lines_in_side_ego[j];
                bev_lines_in_side_ego[j] = bev_lines_in_side_ego[j + 1];
                bev_lines_in_side_ego[j + 1] = tmp;
            }
        }
    }   
#ifdef LOC
    std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines_in_side_ego.size: " <<bev_lines_in_side_ego.size() <<std::endl;
    for (int i = 0; i < bev_lines_in_side_ego.size(); i++) {
        std::cout << " bev_lines_in_side_ego[ " << i << " ]::  id:"<<bev_lines_in_side_ego[i]->id<<" ,sl.l: "<<
         bev_lines_in_side_ego[i]->sl_to_ego.l<<std::endl;
    }
#endif

    return;
}  

void LaneModel::ConnectBevLines(std::vector<BevLineInnerS*>& base_lines,std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines_res, std::unordered_map<int,int>& multi_line_index){
    // std::cout << __FILE__ << "," << __LINE__ << "," << " ConnectBevLines start: " <<std::endl;
    connected_lines_res.clear();
    multi_line_index.clear();//哪两条线组成merge或者split
    for(auto& b_line: base_lines){
        int is_multi_line = 0;//0-straight line; 1- merge; 2- split; 3- both
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " b_line->id: " << b_line->id<<std::endl;
#endif
        std::vector<BevLineInnerConnect> connect_lines{};
        ConnectLine(bev_lines,*b_line,connect_lines,is_multi_line);
        //当前的multi line只做了两条，最左和最右的
        for(auto line:connect_lines){
            connected_lines_res.push_back(line);
        }
        if(is_multi_line > 0){
            multi_line_index.emplace(std::make_pair(connected_lines_res.size()-2, connected_lines_res.size()-1));
            multi_line_index.emplace(std::make_pair(connected_lines_res.size()-1, connected_lines_res.size()-2));
        }
    }

    MultiLineIndexPostProcess(connected_lines_res, multi_line_index);

    // DeleteRepeatLine(connected_lines_res, multi_line_index);

    return;
}

bool LaneModel::MultiLineIndexPostProcess(std::vector<BevLineInnerConnect>& connect_lines ,std::unordered_map<int,int>& multi_line_index){
    //一连多的情况，做成merge
    for(int i= 0; i<connect_lines.size(); i++){
        if(connect_lines[i].back_connect_line_set_ptr.size()>0){
            for(int j=i+1; j<connect_lines.size(); j++){
                if(connect_lines[j].back_connect_line_set_ptr.size()>0){
                    if(connect_lines[i].back_connect_line_set_ptr.begin()->first->id == 
                        connect_lines[j].back_connect_line_set_ptr.begin()->first->id
                        && connect_lines[i].back_connect_line_set_ptr.begin()->first->total_line.size()>0
                        && connect_lines[j].back_connect_line_set_ptr.begin()->first->total_line.size()>0
                        && multi_line_index.find(i) == multi_line_index.end()){
                            multi_line_index.emplace(std::make_pair(i, j));
                            multi_line_index.emplace(std::make_pair(j, i));  
                            connect_lines[i].split_merge_type = 2;
                            connect_lines[j].split_merge_type = 2;
                            connect_lines[i].merge_side = 2;
                            connect_lines[j].merge_side = 1;
                            double line1_connect_line_x = connect_lines[i].back_connect_line_set_ptr.begin()->first->total_line.front().x;
                            double line2_connect_line_x = connect_lines[j].back_connect_line_set_ptr.begin()->first->total_line.front().x;
                            for(int p_index = 0; p_index<connect_lines[i].line_point.size(); p_index++){
                                if(connect_lines[i].line_point.at(p_index).x > line1_connect_line_x+0.001 && p_index>0){
                                    connect_lines[i].merge_point_index = p_index;
                                }
                            }

                            for(int p_index = 0; p_index<connect_lines[j].line_point.size(); p_index++){
                                if(connect_lines[j].line_point.at(p_index).x > line2_connect_line_x+0.001 && p_index>0){
                                    connect_lines[j].merge_point_index = p_index;
                                }
                            }
                        }
                }
            }
        }
    }
    return true;
}

// bool LaneModel::DeleteRepeatLine(std::vector<BevLineInnerConnect>& connect_lines ,std::unordered_map<int,int>& multi_line_index){
//     //优先删除 front连接的线， 比如front_connrct_id==4 && base id ==5, 和另一条线 base id ==4 && back_connect_id == 5, 删除掉base id是5的线
//     for (auto it = connect_lines.begin(); it != connect_lines.end(); ) { // 注意这里没有 it++
//         for(auto& line: connect_lines){
//             if ((*it).front_connect_line_set_ptr.size()>0 && (*it).base_line_ptr != nullptr && line.base_line_ptr != nullptr && line.back_connect_line_set_ptr.size()>0) {
//                 int it_front_connect_id = (*it).front_connect_line_set_ptr.begin()->first->id;
//                 int it_base_id = (*it).base_line_ptr->id;
//                 int line_base_id = line.base_line_ptr->id;
//                 int line_back_connect_id = line.back_connect_line_set_ptr.begin()->first->id;
//                 it = connect_lines.erase(it); // erase 返回指向下一个有效元素的迭代器
//                 break;
//             } else {
//                 it++; // 只有没删除时，才将迭代器指向下一个元素
//             }

//         }

//     }
//     return true;
// }

bool LaneModel::CutLineInSplitMerge(EFMRefLinePoints& line, bool is_split){
    if (line.size() <= 2){
        return true;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " is_split: " << (int)is_split<< std::endl;
    
    double count_length = 0.0;
    if (is_split){
        //分歧点，删除当前line最后面LINE_CUT_DISTANCE
        for (int i = line.size() - 1; i > 0; i--){
            count_length += std::hypot(line[i].x - line[i-1].x, line[i].y - line[i-1].y);
            // std::cout << __FILE__ << "," << __LINE__ << "," << " count_length: " << count_length << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " i: " << i<< std::endl;
            if (count_length >= LINE_CUT_DISTANCE){
                // std::cout << __FILE__ << "," << __LINE__ << "," << " i: " << i<< std::endl;
                line.erase(line.begin() + i, line.end());
                // std::cout << __FILE__ << "," << __LINE__ << "," << " line.size(): " << line.size()<< std::endl;
                break;
            }
        }
        
    }else{
        //合流点，删除当前line最前面LINE_CUT_DISTANCE
        for (int i = 0; i < line.size() - 1; i++){
            count_length += std::hypot(line[i].x - line[i+1].x, line[i].y - line[i+1].y);
            // std::cout << __FILE__ << "," << __LINE__ << "," << " count_length: " << count_length << std::endl;
            if (count_length >= LINE_CUT_DISTANCE){
                // std::cout << __FILE__ << "," << __LINE__ << "," << " i: " << i<< std::endl;
                line.erase(line.begin(), line.begin() + i + 1);
                // std::cout << __FILE__ << "," << __LINE__ << "," << " i: " << i<< std::endl;
                break;
            }
        }

    }
    return true;
 }

void LaneModel::ConnectLine(std::vector<BevLineInner>& bev_lines, BevLineInner& base_line,std::vector<BevLineInnerConnect>& connect_lines_res, int& is_multi_lines) {
    // 在所有的线里找到和当前需要拼的线近的，头、尾都找
    connect_lines_res.clear();
    is_multi_lines =0;
    // if(&base_line == nullptr){
    //     return;
    // }
    BevLineInnerConnect connect_line1(&base_line);//左线
    BevLineInnerConnect connect_line2(&base_line);//右线
    int connect_time =0;
#ifdef LM_CONNECT
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.id: " << base_line.id<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.total_points.size: " << base_line.total_line.size()<< std::endl;
#endif
    std::vector<std::pair<std::pair<EFMRefLinePoints,int>,BevLineInner*>> line_res_res01;//结果1<<0,1<<1的所有线, connect_line的头部连接线集合, int放的是res
    std::vector<std::pair<std::pair<EFMRefLinePoints,int>,BevLineInner*>> line_res_res23;//结果1<<2,1<<3的所有线，connect_line的尾部连接线集合
    while(connect_time<1){
        for(auto& bev_ln: bev_lines){
#ifdef LM_CONNECT
            std::cout << __FILE__ << "," << __LINE__ << "," << " LineRelativeLoc: base_line.id: " << base_line.id<< std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << " LineRelativeLoc: bev_ln.id: " << bev_ln.id<< std::endl;
#endif
            std::pair<EFMRefLinePoints,int> line_res01{};
            std::pair<EFMRefLinePoints,int> line_res23{};
            if(base_line.id != bev_ln.id && bev_ln.id<10000){// && bev_ln.is_connected_f == false
                bool is_curv_f = bev_ln.lane_location_type==21 || bev_ln.lane_location_type == 22 || bev_ln.lane_location_type==24 || base_line.lane_location_type==21 || base_line.lane_location_type == 22 || base_line.lane_location_type==24;
                uint8_t res =LineRelativeLoc(base_line.total_line, bev_ln.total_line, line_res01, line_res23, is_curv_f);
                if(((res & 1<<0) ||(res & 1<<1)) && line_res01.first.size()>0){
                    line_res_res01.push_back(std::make_pair(line_res01,&bev_ln));
                    // bev_ln.is_connected_f = true;
                    // connect_line.line_point = line_res;
                    // connect_line.connect_line_id.insert(bev_ln.id);
                }
                if(((res & 1<<2) ||(res & 1<<3))&& line_res23.first.size()>0){
                    line_res_res23.push_back(std::make_pair(line_res23,&bev_ln));
                }
            } 
        }
        connect_time++;
    }
#ifdef LM_CONNECT
    //plot
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.id: " << base_line.id<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res01.size: " << line_res_res01.size()<<std::endl;
    for(int i = 0; i<line_res_res01.size(); i++){
        std::stringstream ss2, ss3;
        double y = line_res_res01[i].first.first.front().y + line_res_res01[i].first.first.back().y;
        std::cout<<i<<" ,bev_id: "<<line_res_res01[i].second->id<<" ,points_size:"<<line_res_res01[i].first.first.size()<<" ,y_avg:"<<y<<std::endl;
        ss2 << "line_res_x =[";
        ss3 << "line_res_y =[";
        for (int j = 0; j < line_res_res01[i].first.first.size(); j++) {
            ss2<<" "<<line_res_res01[i].first.first.at(j).x;
            ss3<<" "<<line_res_res01[i].first.first.at(j).y;
        }
        ss2 << "]"<<std::endl;
        ss3 << "]"<<std::endl;    
        std::cout << ss2.str();
        std::cout << ss3.str();
    }
    //plot
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.id: " << base_line.id<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res23.size: " <<line_res_res23.size()<<std::endl;
    for(int i = 0; i<line_res_res23.size(); i++){
        double y = line_res_res23[i].first.first.front().y + line_res_res23[i].first.first.back().y;
        std::cout<<i<<" ,bev_id: "<<line_res_res23[i].second->id<<" ,points_size:"<<line_res_res23[i].first.first.size()<<" ,y_avg:"<<y<<std::endl;
        std::stringstream ss2, ss3;
        ss2 << "line_res_x =[";
        ss3 << "line_res_y =[";
        for (int j = 0; j < line_res_res23[i].first.first.size(); j++) {
            ss2<<" "<<line_res_res23[i].first.first.at(j).x;
            ss3<<" "<<line_res_res23[i].first.first.at(j).y;
        }
        ss2 << "]"<<std::endl;
        ss3 << "]"<<std::endl;    
        std::cout << ss2.str();
        std::cout << ss3.str();
    }
#endif
    //多条线判断左右，优先连左线,左正右负
    MultiLinesLeftRight(line_res_res01);
    MultiLinesLeftRight(line_res_res23);
    int left_index = 0;
    int right_index = 1;
#ifdef LM_CONNECT
    //plot
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.id: " << base_line.id<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "after leftRight line_res_res01.size: " << line_res_res01.size()<<std::endl;
    for(int i = 0; i<line_res_res01.size(); i++){
        std::stringstream ss2, ss3;
        double y = line_res_res01[i].first.first.front().y + line_res_res01[i].first.first.back().y;
        std::cout<<i<<" ,bev_id: "<<line_res_res01[i].second->id<<" ,points_size:"<<line_res_res01[i].first.first.size()<<" ,y_avg:"<<y<<std::endl;
    }
    //plot
    std::cout << __FILE__ << "," << __LINE__ << "," << " base_line.id: " << base_line.id<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " after leftRight line_res_res23.size: " <<line_res_res23.size()<<std::endl;
    for(int i = 0; i<line_res_res23.size(); i++){
        double y = line_res_res23[i].first.first.front().y + line_res_res23[i].first.first.back().y;
        std::cout<<i<<" ,bev_id: "<<line_res_res23[i].second->id<<" ,points_size:"<<line_res_res23[i].first.first.size()<<" ,y_avg:"<<y<<std::endl;
    }
#endif
    if(line_res_res01.size()>1){
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res01.size()>1"<< std::endl;
#endif
        EFMRefLinePoints base_line_point = base_line.total_line;
        CutLineInSplitMerge(base_line_point, false);

        connect_line1.line_point = base_line_point;//两条线的base line相同
        connect_line2.line_point = base_line_point;
        //左侧线
        connect_line1.merge_side = 1;
        connect_line1.merge_point = connect_line1.line_point.front();
        int size_tmp = connect_line1.line_point.size();
        connect_line1.line_point.insert(connect_line1.line_point.begin(),line_res_res01[left_index].first.first.begin(), line_res_res01[left_index].first.first.end());
        connect_line1.merge_point_index = connect_line1.line_point.size() - size_tmp;
        connect_line1.front_connect_line_set_ptr.emplace(line_res_res01[left_index].second,line_res_res01[left_index].first.second);
        line_res_res01[left_index].second->is_connected_f = true;
        //右侧线
        connect_line2.merge_side = 2;
        connect_line2.merge_point = connect_line2.line_point.front();
        size_tmp = connect_line2.line_point.size();
        connect_line2.line_point.insert(connect_line2.line_point.begin(),line_res_res01[right_index].first.first.begin(), line_res_res01[right_index].first.first.end());
        connect_line2.merge_point_index = connect_line2.line_point.size() - size_tmp;
        connect_line2.front_connect_line_set_ptr.emplace(line_res_res01[right_index].second,line_res_res01[right_index].first.second);
        line_res_res01[right_index].second->is_connected_f = true;   

        is_multi_lines = (1<<1|is_multi_lines); //merge
        connect_line1.split_merge_type =  is_multi_lines;
        connect_line2.split_merge_type =  is_multi_lines;   

    }else if(line_res_res01.size() ==1){
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res01.size()==1"<< std::endl;
#endif
        EFMRefLinePoints base_line_point = base_line.total_line;
        CutLineInSplitMerge(base_line_point, false);

        connect_line1.line_point.insert(connect_line1.line_point.begin(),line_res_res01[0].first.first.begin(), line_res_res01[0].first.first.end());
        // line_res_res01[0].second->is_connected_f = true;
        connect_line1.front_connect_line_set_ptr.emplace(line_res_res01[0].second,line_res_res01[0].first.second);
        line_res_res01[0].second->is_connected_f = true;   
    }

    if(line_res_res23.size()>1){
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res23.size()>1"<< std::endl;
#endif
        EFMRefLinePoints base_line_point = base_line.total_line;
        CutLineInSplitMerge(connect_line1.line_point, true);
        CutLineInSplitMerge(connect_line2.line_point, true);
        //左线
        connect_line1.split_side = 1;
        connect_line1.split_point = connect_line1.line_point.back();
        connect_line1.split_point_index = connect_line1.line_point.size()-1;
        connect_line1.line_point.insert(connect_line1.line_point.end(),line_res_res23[left_index].first.first.begin(), line_res_res23[left_index].first.first.end());
        connect_line1.back_connect_line_set_ptr.emplace(line_res_res23[left_index].second,line_res_res23[left_index].first.second);
        line_res_res23[left_index].second->is_connected_f = true;

        connect_line2.split_side = 2;
        connect_line2.split_point = connect_line2.line_point.back();
        connect_line2.split_point_index = connect_line2.line_point.size()-1;
        connect_line2.line_point.insert(connect_line2.line_point.end(),line_res_res23[right_index].first.first.begin(), line_res_res23[right_index].first.first.end());
        connect_line2.back_connect_line_set_ptr.emplace(line_res_res23[right_index].second,line_res_res23[right_index].first.second);  
        line_res_res23[right_index].second->is_connected_f = true;

        is_multi_lines = (1<<0|is_multi_lines);//split
        connect_line1.split_merge_type =  is_multi_lines;
        connect_line2.split_merge_type =  is_multi_lines;           
        
    }else if(line_res_res23.size()==1){
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " line_res_res23.size()==1"<< std::endl;
#endif
        if(is_multi_lines > 0){
            //两条线
            CutLineInSplitMerge(connect_line1.line_point, true);
            CutLineInSplitMerge(connect_line2.line_point, true);

            connect_line1.line_point.insert(connect_line1.line_point.end(),line_res_res23[0].first.first.begin(), line_res_res23[0].first.first.end());
            connect_line1.back_connect_line_set_ptr.emplace(line_res_res23[0].second,line_res_res23[0].first.second);

            connect_line2.line_point.insert(connect_line2.line_point.end(),line_res_res23[0].first.first.begin(), line_res_res23[0].first.first.end());
            connect_line2.back_connect_line_set_ptr.emplace(line_res_res23[0].second,line_res_res23[0].first.second); 
            line_res_res23[0].second ->is_connected_f = true;           
        }else{
            CutLineInSplitMerge(connect_line1.line_point, true);
            connect_line1.line_point.insert(connect_line1.line_point.end(),line_res_res23[0].first.first.begin(), line_res_res23[0].first.first.end());
            connect_line1.back_connect_line_set_ptr.emplace(line_res_res23[0].second,line_res_res23[0].first.second);
            line_res_res23[0].second ->is_connected_f = true;
        }  
    }

#ifdef LM_CONNECT
    std::cout << __FILE__ << "," << __LINE__ << "," << " connect_line1.line_point.size(): " << connect_line1.line_point.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " connect_line1.front_connect_line_set_ptr.size(): " << connect_line1.front_connect_line_set_ptr.size() << std::endl;
    for(auto iter: connect_line1.front_connect_line_set_ptr){
        std::cout <<" ,"<<iter.first->id;
    }
    std::cout << std::endl;
#endif

    if(is_multi_lines > 0){
        connect_lines_res.push_back(connect_line2);
        connect_lines_res.push_back(connect_line1);
    }else{
        connect_lines_res.push_back(connect_line1);
    }
    return;
}

void LaneModel::MultiLinesLeftRight(std::vector<std::pair<std::pair<EFMRefLinePoints,int>,BevLineInner*>>& connect_set){
    //选出最左和最右的两个, [0]左， [1]右
    if(connect_set.size()<2){
        return;
    }
    //添加计算公共部分来判断左右
    // double start_x = -255;
    // double end_x = 255;
    // for(auto& iter: connect_set){
    //     if(start_x < iter.first.first.front().x){
    //         start_x = iter.first.first.front().x;
    //     }
    //     if(end_x > iter.first.first.back().x){
    //         end_x = iter.first.first.back().x;
    //     }
    // }
    std::vector<std::pair<std::pair<EFMRefLinePoints,int>,BevLineInner*>>res{};
    double max_y = -10000;
    double min_y = 10000;
    int max_y_index = 0;
    int min_y_index = 0;

    // for(int i = 0; i<connect_set.size();i++) {
    //     if(connect_set[i].first.first.size()<1){
    //         continue;
    //     }
    //     double y =0;
    //     for(auto& pt: connect_set[i].first.first){
    //         y+= pt.y;
    //     }
    //      y = y/ connect_set[i].first.first.size();
    //     if(y>max_y){
    //         max_y_index = i;
    //         max_y = y;
    //     }
    //     if(y<min_y){
    //         min_y_index = i;
    //         min_y = y;
    //     }
    //     // std::cout<< __FILE__ << "," << __LINE__ <<i<<" ,bev_id: "<<line_res_res01[i].second->id<<" ,points_size:"<<line_res_res01[i].first.size()<<" ,y_avg:"<<y<<std::endl;
    //     // std::cout<< __FILE__ << "," << __LINE__ <<i<<" ,max_y_index: "<<max_y_index<<" ,max_y:"<<max_y<<std::endl;
    // }

    //方法2，取一条线当做基准,其他线计算到这条线的距离
    EFMRefLinePoints base_line_point = connect_set[0].first.first;
    double l=0;
    double min_l = 0, max_l = 0;
    for(int i = 1; i<connect_set.size();i++) {
        if(connect_set[i].first.first.size()<1){
            continue;
        }
        CalTwoLineDist(base_line_point, connect_set[i].first.first, l, 1);
        if(l<0){
            if(l<min_l){
                min_l = l;
                min_y_index = i;
            }
        }else{
            if(l>max_l){
                max_l = l;
                max_y_index = i;
            }
        }

    }
    if(min_l<max_l && max_y_index>=0 && min_y_index>=0){
        res.push_back(connect_set[max_y_index]);
        res.push_back(connect_set[min_y_index]);
        connect_set.clear();
        connect_set = res;
    }

    return;
}
int LaneModel::IsNeedConnect(EFMPoint tar_point, EFMPoint tar_point_dir,EFMRefLinePoints line, bool is_front){
    // tar_point端点， tar_point_dir端点的方向， line线
    //case1:线的端点，在line的线内，并且l<1, 需要把line切两段
    //case2:端点在line区间内，但是l>1,计算端点的延长线是否和line相交，求出交点并计算斜率（夹角），斜率大于一定的，也要连，line切两段
    //case3:端点不在线内，如果端点的距离线的距离s^2+l^2<25, 那么也认为需要连，这种直接连

    //res -2 切断， 1-直连
    
    double p_l_thred_level1 = 1;
    double p_l_thred_level2 = 2;
    int res = 0;
    if(line.size()<1 || tar_point.x<-50 || tar_point.x>100){
        return res;
    }
    PointSLd p_sl;
    bool is_inside = false;  
    double line_length = CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line);
#ifdef LM_CONNECT
    std::cout << __FILE__ << "," << __LINE__ << "," << " line_length: " <<line_length<<std::endl; 
    std::cout << __FILE__ << "," << __LINE__ << "," << " tar_point.x: " << tar_point.x<< ",tar_point.y: "<< tar_point.y<<std::endl;  
#endif
    // std::cout << __FILE__ << "," << __LINE__ << "," << " sl_first_p.l: " << sl_first_p.l<< ",sl_first_p.s: "<< sl_first_p.s<<std::endl;    
    EFMPoint p_front, p_back, p_proj;
    if(CommonTool::DiscretePointsMath::GetInstance()-> CalPointSLBodyCoordinate(line, tar_point, p_sl, is_inside,p_front,p_back,p_proj) == true){
#ifdef LM_CONNECT
        std::cout << __FILE__ << "," << __LINE__ << "," << " p_sl.l: " << p_sl.l<< ",p_sl.s: "<< p_sl.s<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << "," << " is_inside: " <<(int)is_inside<<std::endl; 
        std::cout << __FILE__ << "," << __LINE__ << "," << " tar_point.x: " <<tar_point.x<< ",tar_point.y: "<<tar_point.y<<std::endl; 
#endif
        // bool connect_f =false;
        // if((is_front == true && line.front().x+15< tar_point.x)
        //    ||(is_front == false && line.back().x-15 >tar_point.x)){
        //     connect_f = true;
        // }
// #ifdef LM_CONNECT
//         std::cout << __FILE__ << "," << __LINE__ << "," << "is_front: " << (int)is_front<<std::endl;
//         std::cout << __FILE__ << "," << __LINE__ << "," << " line.back().x: " <<line.back().x<<" line.front().x: " <<line.front().x<< " tar_point.x: " << tar_point.x<<std::endl; 
// #endif
        // if(connect_f){
        if(is_inside == true){
            if(fabs(p_sl.l)< p_l_thred_level1){
#ifdef LM_CONNECT               
                std::cout << __FILE__ << "," << __LINE__ << "," << " 111111111111"<< std::endl;
#endif
                res =2 ;
            }else{
                //延长tar_point 构成线，计算交点
                EFMPoint extern_p=CommonTool::DiscretePointsMath::GetInstance()-> ExtendPoint(tar_point, tar_point_dir, 1);
                EFMPoint intersection_p;
                if(CommonTool::DiscretePointsMath::GetInstance()->computeIntersection(tar_point, extern_p, p_front, p_back, intersection_p) == true){
#ifdef LM_CONNECT 
                    std::cout << __FILE__ << "," << __LINE__ << "," << ",intersection_p.x: "<< intersection_p.x<< ",intersection_p.y: "<< intersection_p.y<<std::endl;
#endif
                    //判断交点的方向是否在tar_point_dir的方向上
                    double c = sqrt(pow(tar_point.x-intersection_p.x,2)+pow(tar_point.y-intersection_p.y,2));
                    double k = c / fabs(p_sl.l);//斜边比对边,越大夹角越小
#ifdef LM_CONNECT 
                    std::cout << __FILE__ << "," << __LINE__ << "," << " c: " << sqrt(pow(tar_point.x-intersection_p.x,2)+pow(tar_point.y-intersection_p.y,2))<<" ,a:"<< fabs(p_sl.l)<<std::endl;
                    std::cout << __FILE__ << "," << __LINE__ << "," << " k: " << k<<std::endl;
#endif
                    if(tar_point.x <extern_p.x){
                        if(tar_point.x<intersection_p.x){
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << " 2222222222"<< std::endl;
#endif
                            if(k<6 && fabs(p_sl.l)< p_l_thred_level2){
                                res = 2;
                            }
                        }else{
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << "###### 2222222222"<< std::endl;
#endif
                            //如果交点反向，那么判断交点和端点的距离，以及交点和投影点的距离
                            double b = sqrt(pow(intersection_p.x-p_proj.x,2)+pow(intersection_p.y-p_proj.y,2));
                            double tri_k = (b-p_sl.s)/b;
                            double trans_a = tri_k* p_sl.l;
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << "b: "<<b<<" ,tri_k: "<<tri_k <<" ,trans_a: "<<trans_a<<std::endl;
#endif
                            if(c<12.5 && fabs(trans_a)<p_l_thred_level2){
                                res = 2;
                            }
                        }
                    }else{
                        if(tar_point.x>intersection_p.x){
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << " 3333333333"<< std::endl;
#endif
                            if(k<6 && fabs(p_sl.l)< p_l_thred_level2){
                                res = 2;
                            }                            
                        }else{
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << "###### 3333333333"<< std::endl;
#endif
                            //如果交点反向，那么判断交点和端点的距离，如果距离小于10m,那么也认为是可连接
                            double b = sqrt(pow(intersection_p.x-p_proj.x,2)+pow(intersection_p.y-p_proj.y,2));
                            double tri_k = (b-p_sl.s)/b;
                            double trans_a = tri_k* p_sl.l;
#ifdef LM_CONNECT 
                            std::cout << __FILE__ << "," << __LINE__ << "," << "b: "<<b<<" ,tri_k: "<<tri_k <<" ,trans_a: "<<trans_a<<std::endl;
#endif
                            if(c<12.5 && fabs(trans_a)<p_l_thred_level2){
                                res = 2;
                            }
                        }
                    }
                }
            }
        }else{
            //不在区间内,
#ifdef LM_CONNECT 
            std::cout << __FILE__ << "," << __LINE__ << "," << " 不在区间内"<< std::endl;
#endif
            double s = 10000;
            if(is_front == true){
                s= p_sl.s-line_length;
            }else{
                s = p_sl.s;
            }
            double k =1000;
            if(fabs(s)<0.2){
                // k =p_sl.l;
            }else{
                k= p_sl.l/s;
            }
#ifdef LM_CONNECT 
            std::cout << __FILE__ << "," << __LINE__ << "," << " k:"<<k<<" ,s:"<<s<< std::endl;
#endif
            if(sqrt(pow(p_sl.l,2)+pow(s,2))<4 && (fabs(k)<0.6 ||(k>999 && fabs(p_sl.l)<0.9))){
                if(is_front == true && p_sl.s>line_length){
                    //如果端点是front的，那么需要端点在line的 s> line_length
                    res = 1;
                }else if(is_front == false){
                    //如果端点是back的，那么需要端点在line的 s<0
                    res =1;
                }
            }
        } 
        // }  
    }
    return res;
}

int LaneModel::LineRelativeLoc(EFMRefLinePoints line1, EFMRefLinePoints line2, std::pair<EFMRefLinePoints,int>& line_res01, std::pair<EFMRefLinePoints,int>& line_res23, bool is_line2_side_curv){
    //line1的头和line2的l接近，且line2在line1头的头部方向，则直接拼接1<<0
    //line1的头和line2的l接近，且落line2的中间，则截断拼接，1<<1
    //line1的尾和line2的l接近，且line2在line1尾的尾部方向，则直接拼接1<<2
    //line1的尾和line2的l接近，且落line2的中间，则截断拼接，1<<3
    //不和物理隔离的线连
    uint8_t res =0;
    line_res01.first.clear();
    line_res01.second = 0;
    line_res23.first.clear();
    line_res23.second = 0;
    if(line1.size()<2 || line2.size()<2){
        return 0;
    }
    if(is_line2_side_curv == true){
        return 0;
    }
    double line1_length = CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line1);
    EFMPoint line1_first_p = line1[0];
    double dir_length = sqrt(pow(line1_first_p.x - line1[1].x,2)+pow(line1_first_p.y - line1[1].y,2));
    EFMPoint line1_first_p_extern_dir((line1_first_p.x - line1[1].x)/dir_length,  (line1_first_p.y - line1[1].y)/dir_length);
    EFMPoint line1_last_p = line1.back();
    dir_length = sqrt(pow(line1_last_p.x - line1.at(line1.size()-2).x,2)+pow(line1_last_p.y - line1.at(line1.size()-2).y,2));
    EFMPoint line1_last_p_extern_dir((line1_last_p.x - line1.at(line1.size()-2).x)/dir_length,  (line1_last_p.y - line1.at(line1.size()-2).y)/dir_length);

    double line2_length = CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line2);
    EFMPoint line2_first_p = line2[0];
    dir_length = sqrt(pow(line2_first_p.x - line2[1].x,2)+pow(line2_first_p.y - line2[1].y,2));
    EFMPoint line2_first_p_extern_dir((line2_first_p.x - line2[1].x)/dir_length,  (line2_first_p.y - line2[1].y)/dir_length);
    EFMPoint line2_last_p = line2.back();
    dir_length = sqrt(pow(line2_last_p.x - line2.at(line2.size()-2).x,2)+pow(line2_last_p.y - line2.at(line2.size()-2).y,2));
    EFMPoint line2_last_p_extern_dir((line2_last_p.x - line2.at(line2.size()-2).x)/dir_length,  (line2_last_p.y - line2.at(line2.size()-2).y)/dir_length);
// #ifdef CONNECT_LINE
//     //plot
//     std::stringstream ss, ss1, ss2, ss3, ss4, ss5;
//     ss << __FILE__ << "," << __LINE__ << "line1_first_p =["<<line1_first_p.x<<" "<<line1_first_p.y<<"]"<<std::endl;
//     std::cout << ss.str();
//     ss1 <<__FILE__ << "," << __LINE__ << "line1_last_p =["<<line1_last_p.x<<" "<<line1_last_p.y<<"]"<<std::endl;
//     std::cout << ss1.str();
//     // ss2 << "line2_x =[";
//     // ss3 << "line2_y =[";
//     // for (int i = 0; i < line2.size(); i++) {
//     //     ss2<<" "<<line2[i].x;
//     //     ss3<<" "<<line2[i].y;
//     // }
//     // ss2 << "]"<<std::endl;
//     // ss3 << "]"<<std::endl;    
//     // std::cout << ss2.str();
//     // std::cout << ss3.str();
// #endif
    //line1 头判断
    bool is_front = true;
#ifdef LM_CONNECT
    std::cout << __FILE__ << "," << __LINE__ << "," << " 头部判断"<<std::endl;
#endif
    int cut_or_connect = IsNeedConnect(line1_first_p, line1_first_p_extern_dir,line2,is_front);
    if(cut_or_connect ==1 && line2_length>10){
        //直连
        //case 1<<0
        line_res01.first = line2;
        res = res | 1<<0;
        line_res01.second = res;        
    }else if(cut_or_connect == 2){
        //line2切断
        //case 1<<2
        EFMRefLinePointsSection line2_section;
        if(CommonTool::DiscretePointsMath::GetInstance()-> SaperateLineIntoTwoPart(line2, line1_first_p, line2_section) == true){
            double length_tmp=CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line2_section[0]);
            if(line2_section[0].size()>1 && length_tmp>10){
                line_res01.first.assign(line2_section[0].begin(),line2_section[0].end()-1);
                res = res | 1<<1;
                line_res01.second = res;
            }                    
        }         
    }

    //line1 尾判断
#ifdef LM_CONNECT
    std::cout << __FILE__ << "," << __LINE__ << "," << " wei部判断"<<std::endl;
#endif
    int cut_or_connect2 = IsNeedConnect(line1_last_p, line1_last_p_extern_dir,line2, false);
    if(cut_or_connect2 ==1 && line2_length>10){
        //直连
        line_res23.first = line2;
        res = res | 1<<2;
        line_res23.second = res;        
    }else if(cut_or_connect2 == 2){
        //line2切断
        //case 1<<3
        EFMRefLinePointsSection line2_section;
        if(CommonTool::DiscretePointsMath::GetInstance()-> SaperateLineIntoTwoPart(line2, line1_last_p, line2_section) == true){
            double length_tmp=CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line2_section[1]);
            if(line2_section[1].size()>1 && length_tmp>10){
                line_res23.first.assign(line2_section[1].begin()+1,line2_section[1].end());
                res = res | 1<<3;
                line_res23.second = res;
            }                     
        }        
    }
    

// #ifdef CONNECT_LINE
//     std::cout << __FILE__ << "," << __LINE__ << "," << " line_res01.points.size(): " <<line_res01.first.size()<<std::endl; 
//     std::cout << __FILE__ << "," << __LINE__ << "," << " line_res23.points.size(): " <<line_res23.first.size()<<std::endl; 
//     std::cout << __FILE__ << "," << __LINE__ << "," << " res: " <<(int)res<<std::endl;
// #endif
    return res;
}

bool LaneModel::FixConnectLines(std::vector<BevLineInnerConnect>& connected_lines){
    //去掉过短的线
    std::vector<BevLineInnerConnect> backup_connected_lines = connected_lines;
    connected_lines.clear();
    for(auto& line: backup_connected_lines){
        double line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(line.line_point);
        if(line_length<30){
            continue;
        }
        EFMRefLinePoints fix_line_points{};
        if(CommonTool::DiscretePointsMath::GetInstance()->FixPathDensity(line.line_point, 2.5,fix_line_points) == true){
            line.line_point = fix_line_points;
        }
        connected_lines.push_back(line);        
    }

    return true;
}

bool LaneModel::GenerateLaneGroupSet(std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines, const std::unordered_map<int,int>& multi_line_index, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset,const std::vector<std::pair<int,double>>& road_split_dir_vec,
                                    BevLaneElementGroupSet& lane_group_set, int& ego_lane_left_index, int& ego_lane_right_index){
    //输出 车道合集，split的放一个
    //当前，只做左中右三个group
    //step1: 自车车道，先找到自车道的左右线, 左右车道不会是split的车道
    lane_group_set.clear();
    ego_lane_left_index = -1;
    ego_lane_right_index = -1;
    int& ego_nearest_right = ego_lane_right_index;
    int& ego_nearest_left = ego_lane_left_index;

    for(int i=1;i<connected_lines.size();i++){
        if(connected_lines[i].base_line_ptr->sl_to_ego.l<=0 && connected_lines[i-1].base_line_ptr->sl_to_ego.l>0){
            ego_nearest_right = i-1;
            ego_nearest_left = i;
        }
    }
    if(ego_nearest_right>= 0 && ego_nearest_left>=0){
        double dist_l = 0;
        int count = 0;
        while(count<3 && fabs(dist_l)<1.5 && ego_nearest_right>=0){
            count ++;
            if(connected_lines[ego_nearest_left].base_line_ptr->id == connected_lines[ego_nearest_right].base_line_ptr->id){//split的base相同
                ego_nearest_right --;
            }else{
                CalTwoLineDist(connected_lines[ego_nearest_right].line_point, connected_lines[ego_nearest_left].line_point, dist_l, 3);                
                if(fabs(dist_l)>=1.5){
                    break;
                }else{
                    ego_nearest_right--;
                }                
            }
        }       
    }

#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << " ego_nearest_right: " <<ego_nearest_right<<" ego_nearest_left: " <<ego_nearest_left<<std::endl; 
#endif
    BevLaneElementGroup ego_lane_group{}, right_lane_group{}, left_lane_group{};
    if(ego_nearest_right!=ego_nearest_left && ego_nearest_right>=0 && ego_nearest_left<connected_lines.size()){
        GenerateLaneGroup(bev_lines, connected_lines, multi_line_index, ego_nearest_right, ego_nearest_left,0, ego_lane_group, ego_nearest_right, ego_nearest_left);
        //延长
        // ExtendLane(ego_lane_group, path_point_body_with_offset, road_split_dir_vec);
    }   
    PlotEgoLaneGroup(ego_lane_group);
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "end ego_nearest_right: " <<ego_nearest_right<<" ,end ego_nearest_left: " <<ego_nearest_left<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " ego_lane_group.size(): " <<ego_lane_group.size()<<std::endl; 
    for(int i = 0; i< ego_lane_group.size(); i++){
        std::cout<< "ego_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<ego_lane_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< ego_lane_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<ego_lane_group[i].left_front_connect_id
                                                <<" right_line_base_id: "<<ego_lane_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< ego_lane_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<ego_lane_group[i].right_front_connect_id
                                                 <<" ,left_line_size"<<ego_lane_group[i].left_line_points.size()<<" ,right_line_size"<<ego_lane_group[i].right_line_points.size()<<std::endl;
        std::cout<<"            left_line_split_index: "<<ego_lane_group[i].left_line_split_index << " ,left_line_merge_index: "<<ego_lane_group[i].left_line_merge_index<<
        " ,right_line_split_index: "<<ego_lane_group[i].right_line_split_index << " ,right_line_merge_index: "<<ego_lane_group[i].right_line_merge_index<<std::endl;
        std::cout<<"            left_line_split_x: "<<ego_lane_group[i].left_line_split_x << " ,left_line_merge_x: "<<ego_lane_group[i].left_line_merge_x<<
        " ,right_line_split_x: "<<ego_lane_group[i].right_line_split_x << " ,right_line_merge_x: "<<ego_lane_group[i].right_line_merge_x<<std::endl;
        std::cout<<" left_line_type:";
        for(auto& type: ego_lane_group[i].left_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        std::cout<<" right_line_type:";
        for(auto& type: ego_lane_group[i].right_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        // if(ego_lane_group[i].right_line_split_index>=0 ){
        //     std::cout<<" ,right_split_point=["<<ego_lane_group[i].right_line_points[ego_lane_group[i].right_line_split_index].x<<","<<ego_lane_group[i].right_line_points[ego_lane_group[i].right_line_split_index].y<<"]";
        // }
        // if(ego_lane_group[i].right_line_merge_index>=0 ){
        //     std::cout<<" ,right_line_merge_point=["<<ego_lane_group[i].right_line_points[ego_lane_group[i].right_line_merge_index].x<<","<<ego_lane_group[i].right_line_points[ego_lane_group[i].right_line_merge_index].y<<"]";
        // }
        // if(ego_lane_group[i].left_line_split_index>=0 ){
        //     std::cout<<" ,left_split_point=["<<ego_lane_group[i].left_line_points[ego_lane_group[i].left_line_split_index].x<<","<<ego_lane_group[i].left_line_points[ego_lane_group[i].left_line_split_index].y<<"]";
        // }
        // if(ego_lane_group[i].left_line_merge_index>=0 ){
        //     std::cout<<" ,left_line_merge_point=["<<ego_lane_group[i].left_line_points[ego_lane_group[i].left_line_merge_index].x<<","<<ego_lane_group[i].left_line_points[ego_lane_group[i].left_line_merge_index].y<<"]";
        // }
        // std::cout<<std::endl;
        {
        std::stringstream ss, ss1;
        ss<<"ego_left_line_x = [";
        ss1<<"ego_left_line_y = [";
        for(int j=0; j<ego_lane_group[i].left_line_points.size();j++){
            if(j == ego_lane_group[i].left_line_points.size()-1){
                ss<< ego_lane_group[i].left_line_points[j].x;
            }else{
                ss<< ego_lane_group[i].left_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<ego_lane_group[i].left_line_points.size();j++){
            if(j == ego_lane_group[i].left_line_points.size()-1){
                ss1<< ego_lane_group[i].left_line_points[j].y;
            }else{
                ss1<< ego_lane_group[i].left_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        {
        std::stringstream ss, ss1;
        ss<<"ego_right_line_x = [";
        ss1<<"ego_right_line_y = [";
        for(int j=0; j<ego_lane_group[i].right_line_points.size();j++){
            if(j == ego_lane_group[i].right_line_points.size()-1){
                ss<< ego_lane_group[i].right_line_points[j].x;
            }else{
                ss<< ego_lane_group[i].right_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<ego_lane_group[i].right_line_points.size();j++){
            if(j == ego_lane_group[i].right_line_points.size()-1){
                ss1<< ego_lane_group[i].right_line_points[j].y;
            }else{
                ss1<< ego_lane_group[i].right_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }         
    }
#endif

    int right_nearest_left = ego_nearest_right;
    int right_nearest_right = right_nearest_left-1;
    if(right_nearest_left>0){
        double dist_l = 0;
        int count = 0;
        while(count<3 && fabs(dist_l)<1.5 && right_nearest_right>0){
            count ++;
            if(connected_lines[right_nearest_right].base_line_ptr->id == connected_lines[right_nearest_left].base_line_ptr->id){//split的base相同
                right_nearest_right --;
            }else{
                int overlap_start_index=-1, overlap_end_index = -1;
                CalTwoLineDist(connected_lines[right_nearest_left].line_point, connected_lines[right_nearest_right].line_point, dist_l, 1,overlap_start_index, overlap_end_index);   
                std::cout << __FILE__ << "," << __LINE__ << "," << "dist_l: " <<dist_l<<",right_nearest_right:"<<right_nearest_right<<",right_nearest_left:"<<right_nearest_left<<std::endl;             
                if(fabs(dist_l)>=1.5){
                    break;
                }else{
                    //再判断下是不是split
                    if(overlap_start_index>=0 && overlap_end_index>=0){
                        PointSLd start_sl;
                        PointSLd end_sl;
                        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(connected_lines[right_nearest_left].line_point, connected_lines[right_nearest_right].line_point[overlap_start_index], start_sl);
                        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(connected_lines[right_nearest_left].line_point, connected_lines[right_nearest_right].line_point[overlap_end_index], end_sl);
                        if(fabs(start_sl.l)+1< fabs(end_sl.l)){
                            //split, 不跳过
                        }else{
                            right_nearest_right--;
                        }
                    }
                    
                }                
            }
        }        
    }
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "right_nearest_right: " <<right_nearest_right<<" , right_nearest_left: " <<right_nearest_left<<std::endl;
#endif
    if(right_nearest_right!=right_nearest_left && right_nearest_right>=0 && right_nearest_left<connected_lines.size()){
        GenerateLaneGroup(bev_lines, connected_lines, multi_line_index, right_nearest_right, right_nearest_left,2, right_lane_group, right_nearest_right, right_nearest_left);
    } 
    PlotRightLaneGroup(right_lane_group);
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "end right_nearest_right: " <<right_nearest_right<<" ,end right_nearest_left: " <<right_nearest_left<<std::endl;    

    std::cout << __FILE__ << "," << __LINE__ << "," << " right_lane_group.size(): " <<right_lane_group.size()<<std::endl; 
    for(int i = 0; i< right_lane_group.size(); i++){
        std::cout<< "right_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<right_lane_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< right_lane_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<right_lane_group[i].left_front_connect_id
                                                <<" right_line_base_id: "<<right_lane_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< right_lane_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<right_lane_group[i].right_front_connect_id
                                                 <<" ,left_line_size"<<right_lane_group[i].left_line_points.size()<<" ,right_line_size"<<right_lane_group[i].right_line_points.size()<<std::endl;
        std::cout<<"            left_line_split_index: "<<right_lane_group[i].left_line_split_index << " ,left_line_merge_index: "<<right_lane_group[i].left_line_merge_index<<
        " ,right_line_split_index: "<<right_lane_group[i].right_line_split_index << " ,right_line_merge_index: "<<right_lane_group[i].right_line_merge_index<<std::endl;
        std::cout<<"            left_line_split_x: "<<right_lane_group[i].left_line_split_x << " ,left_line_merge_x: "<<right_lane_group[i].left_line_merge_x<<
        " ,right_line_split_x: "<<right_lane_group[i].right_line_split_x << " ,right_line_merge_x: "<<right_lane_group[i].right_line_merge_x<<std::endl;
        std::cout<<" left_line_type:";
        for(auto& type: right_lane_group[i].left_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        std::cout<<" right_line_type:";
        for(auto& type: right_lane_group[i].right_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        // if(right_lane_group[i].right_line_split_index>=0  && right_lane_group[i].right_line_split_index<right_lane_group[i].right_line_points.size()){
        //     std::cout<<" ,right_split_point=["<<right_lane_group[i].right_line_points[right_lane_group[i].right_line_split_index].x<<","<<right_lane_group[i].right_line_points[right_lane_group[i].right_line_split_index].y<<"]";
        // }
        // if(right_lane_group[i].right_line_merge_index>=0  && right_lane_group[i].right_line_merge_index<right_lane_group[i].right_line_points.size()){
        //     std::cout<<" ,right_line_merge_point=["<<right_lane_group[i].right_line_points[right_lane_group[i].right_line_merge_index].x<<","<<right_lane_group[i].right_line_points[right_lane_group[i].right_line_merge_index].y<<"]";
        // }
        // if(right_lane_group[i].left_line_split_index>=0 ){
        //     std::cout<<" ,left_split_point=["<<right_lane_group[i].left_line_points[right_lane_group[i].left_line_split_index].x<<","<<right_lane_group[i].left_line_points[right_lane_group[i].left_line_split_index].y<<"]";
        // }
        // if(right_lane_group[i].left_line_merge_index>=0 ){
        //     std::cout<<" ,left_line_merge_point=["<<right_lane_group[i].left_line_points[right_lane_group[i].left_line_merge_index].x<<","<<right_lane_group[i].left_line_points[right_lane_group[i].left_line_merge_index].y<<"]";
        // }
        // std::cout<<std::endl;        
        {
        std::stringstream ss, ss1;
        ss<<"ri_left_line_x = [";
        ss1<<"ri_left_line_y = [";
        for(int j=0; j<right_lane_group[i].left_line_points.size();j++){
            if(j == right_lane_group[i].left_line_points.size()-1){
                ss<< right_lane_group[i].left_line_points[j].x;
            }else{
                ss<< right_lane_group[i].left_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<right_lane_group[i].left_line_points.size();j++){
            if(j == right_lane_group[i].left_line_points.size()-1){
                ss1<< right_lane_group[i].left_line_points[j].y;
            }else{
                ss1<< right_lane_group[i].left_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        {
        std::stringstream ss, ss1;
        ss<<"ri_right_line_x = [";
        ss1<<"ri_right_line_y = [";
        for(int j=0; j<right_lane_group[i].right_line_points.size();j++){
            if(j == right_lane_group[i].right_line_points.size()-1){
                ss<< right_lane_group[i].right_line_points[j].x;
            }else{
                ss<< right_lane_group[i].right_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<right_lane_group[i].right_line_points.size();j++){
            if(j == right_lane_group[i].right_line_points.size()-1){
                ss1<< right_lane_group[i].right_line_points[j].y;
            }else{
                ss1<< right_lane_group[i].right_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
    }
#endif

    int left_nearest_right = ego_nearest_left;
    int left_nearest_left = left_nearest_right+1;
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_nearest_right: " <<left_nearest_right<<" , left_nearest_left: " <<left_nearest_left<<std::endl;
#endif    
    if(left_nearest_right>0 && left_nearest_left< connected_lines.size()){
        double dist_l = 0;
        int count = 0;
        while(count<3 && fabs(dist_l)<1.5 && left_nearest_left<connected_lines.size()-1 && left_nearest_left>0){
            count ++;
            dist_l = 0;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "while start count: " <<count<<" , left_nearest_left: " <<left_nearest_left<<" ,left_nearest_right:"<<left_nearest_right<<std::endl;
            if(connected_lines[left_nearest_right].base_line_ptr->id == connected_lines[left_nearest_left].base_line_ptr->id){//split的base相同
                left_nearest_left ++;
            }else{

                BevLineInnerConnect& right_line = connected_lines[left_nearest_right];
                BevLineInnerConnect& left_line = connected_lines[left_nearest_left];
                
                if(right_line.base_line_ptr!= nullptr &&  left_line.base_line_ptr!= nullptr){
                    //如果左右线是同一条线的split或merge，那么不做成group
                    bool is_split = false;
                    bool is_merge = false;
                    IsTowLineSplit(left_line, right_line, is_split, is_merge);
#ifdef LM_LANEGROUP
                        std::cout << __FILE__ << "," << __LINE__ << "," << " is_split: " <<(int)is_split<<std::endl; 
#endif
                    if(is_merge == true){
                        double lane_broadth = 0;
                        if(CalTwoLineDist(left_line.line_point, right_line.line_point, lane_broadth, 3) == true){
#ifdef LM_LANEGROUP
                        std::cout << __FILE__ << "," << __LINE__ << "," << " lane_broadth: " <<lane_broadth<<std::endl; 
#endif
                            if(fabs(lane_broadth)<4.5){

                                bool is_drivable =IsDrivableSplitMerge(left_line.line_point, right_line.line_point, left_line.base_line_ptr->line_type, right_line.base_line_ptr->line_type, left_line.base_line_ptr->id, right_line.base_line_ptr->id ,
                                                    bev_lines, 1, left_line.base_line_ptr->attached_curb_side, right_line.base_line_ptr->attached_curb_side);
                                if (is_drivable == false){
#ifdef LM_LANEGROUP
                        std::cout << __FILE__ << "," << __LINE__ << "," << " group not drivable: " <<std::endl; 
#endif                            
                                    left_nearest_right++;
                                    left_nearest_left++;
                                }
                                
                            }
                        }
                    }else{
                        CalTwoLineDist(connected_lines[left_nearest_right].line_point, connected_lines[left_nearest_left].line_point, dist_l, 3);                
                        if(fabs(dist_l)>=1.5){
                            break;
                        }else{
                            left_nearest_left++;
                        }                         
                    }
                    
                }             
            }
        }        
    }
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_nearest_right: " <<left_nearest_right<<" , left_nearest_left: " <<left_nearest_left<<std::endl;
#endif
    if(left_nearest_left!=left_nearest_right && left_nearest_left<connected_lines.size() && left_nearest_right>=0){
        GenerateLaneGroup(bev_lines, connected_lines, multi_line_index, left_nearest_right, left_nearest_left, 1,left_lane_group, left_nearest_right, left_nearest_left);
    }
    PlotLeftLaneGroup(left_lane_group);
#ifdef LM_LANEGROUP_RES
    std::cout << __FILE__ << "," << __LINE__ << "," << "end left_nearest_right: " <<left_nearest_right<<" ,end left_nearest_left: " <<left_nearest_left<<std::endl;    

    std::cout << __FILE__ << "," << __LINE__ << "," << " left_lane_group.size(): " <<left_lane_group.size()<<std::endl; 
    for(int i = 0; i< left_lane_group.size(); i++){
        std::cout<< "left_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<left_lane_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< left_lane_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<left_lane_group[i].left_front_connect_id
                                                <<" right_line_base_id: "<<left_lane_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< left_lane_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<left_lane_group[i].right_front_connect_id
                                                 <<" ,left_line_size"<<left_lane_group[i].left_line_points.size()<<" ,right_line_size"<<left_lane_group[i].right_line_points.size()<<std::endl;
        std::cout<<"            left_line_split_index: "<<left_lane_group[i].left_line_split_index << " ,left_line_merge_index: "<<left_lane_group[i].left_line_merge_index<<
        " ,right_line_split_index: "<<left_lane_group[i].right_line_split_index << " ,right_line_merge_index: "<<left_lane_group[i].right_line_merge_index<<std::endl;
        std::cout<<"            left_line_split_x: "<<left_lane_group[i].left_line_split_x << " ,left_line_merge_x: "<<left_lane_group[i].left_line_merge_x<<
        " ,right_line_split_x: "<<left_lane_group[i].right_line_split_x << " ,right_line_merge_x: "<<left_lane_group[i].right_line_merge_x<<std::endl;
        std::cout<<" left_line_type:";
        for(auto& type: left_lane_group[i].left_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        std::cout<<" right_line_type:";
        for(auto& type: left_lane_group[i].right_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        // if(left_lane_group[i].right_line_split_index>=0 ){
        //     std::cout<<" ,right_split_point=["<<left_lane_group[i].right_line_points[left_lane_group[i].right_line_split_index].x<<","<<left_lane_group[i].right_line_points[left_lane_group[i].right_line_split_index].y<<"]";
        // }
        // if(left_lane_group[i].right_line_merge_index>=0 ){
        //     std::cout<<" ,right_line_merge_point=["<<left_lane_group[i].right_line_points[left_lane_group[i].right_line_merge_index].x<<","<<left_lane_group[i].right_line_points[left_lane_group[i].right_line_merge_index].y<<"]";
        // }
        // if(left_lane_group[i].left_line_split_index>=0 ){
        //     std::cout<<" ,left_split_point=["<<left_lane_group[i].left_line_points[left_lane_group[i].left_line_split_index].x<<","<<left_lane_group[i].left_line_points[left_lane_group[i].left_line_split_index].y<<"]";
        // }
        // if(left_lane_group[i].left_line_merge_index>=0 ){
        //     std::cout<<" ,left_line_merge_point=["<<left_lane_group[i].left_line_points[left_lane_group[i].left_line_merge_index].x<<","<<left_lane_group[i].left_line_points[left_lane_group[i].left_line_merge_index].y<<"]";
        // }
        // std::cout<<std::endl;          
        {
        std::stringstream ss, ss1;
        ss<<"le_left_line_x = [";
        ss1<<"le_left_line_y = [";
        for(int j=0; j<left_lane_group[i].left_line_points.size();j++){
            if(j == left_lane_group[i].left_line_points.size()-1){
                ss<< left_lane_group[i].left_line_points[j].x;
            }else{
                ss<< left_lane_group[i].left_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<left_lane_group[i].left_line_points.size();j++){
            if(j == left_lane_group[i].left_line_points.size()-1){
                ss1<< left_lane_group[i].left_line_points[j].y;
            }else{
                ss1<< left_lane_group[i].left_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        {
        std::stringstream ss, ss1;
        ss<<"le_right_line_x = [";
        ss1<<"le_right_line_y = [";
        for(int j=0; j<left_lane_group[i].right_line_points.size();j++){
            if(j == left_lane_group[i].right_line_points.size()-1){
                ss<< left_lane_group[i].right_line_points[j].x;
            }else{
                ss<< left_lane_group[i].right_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<left_lane_group[i].right_line_points.size();j++){
            if(j == left_lane_group[i].right_line_points.size()-1){
                ss1<< left_lane_group[i].right_line_points[j].y;
            }else{
                ss1<< left_lane_group[i].right_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
    }
#endif
    if(right_lane_group.size()>0){
        lane_group_set.push_back(std::make_pair(2,right_lane_group));
    }
    if(ego_lane_group.size()>0){
        lane_group_set.push_back(std::make_pair(0,ego_lane_group));
    }
    if(left_lane_group.size()>0){
        lane_group_set.push_back(std::make_pair(1,left_lane_group));
    }
    return true;
}

bool LaneModel::GenerateLaneGroup(std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines, const std::unordered_map<int,int>& multi_line_index, int nearest_right, int nearest_left, int group_dir, BevLaneElementGroup& res_lane_group, int& res_right_line_index, int& res_left_line_index){
#ifdef LM_LANEGROUP
std::cout << __FILE__ << "," << __LINE__ << "," << " group_dir: " <<group_dir<<std::endl; 
std::cout << __FILE__ << "," << __LINE__ << "," << " nearest_right: " <<nearest_right<<" nearest_left,"<<nearest_left
     << "," << " right id: " <<connected_lines[nearest_right].base_line_ptr->id<<" ,left id,"<<connected_lines[nearest_left].base_line_ptr->id<<std::endl;
#endif
    //group_dir表示 group是自车左右，0自车，1左侧，2右侧
    res_lane_group.clear();
    res_left_line_index = nearest_left;
    res_right_line_index = nearest_right;
    //nearest_right， nearest_left， connect_line的索引，
    //如果左右侧的线构成split/merge, 那么判断这个区域是否可行驶，针对途径上匝道，途径下匝道的场景
    if(nearest_right>=0 && nearest_left>=0){
        BevLineInnerConnect& right_line = connected_lines[nearest_right];
        BevLineInnerConnect& left_line = connected_lines[nearest_left];
        
        if(right_line.base_line_ptr!= nullptr &&  left_line.base_line_ptr!= nullptr
           && (right_line.base_line_ptr->lane_location_type !=21 && right_line.base_line_ptr->lane_location_type !=22 && right_line.base_line_ptr->lane_location_type !=24)
            && (left_line.base_line_ptr->lane_location_type !=21 && left_line.base_line_ptr->lane_location_type !=22 && left_line.base_line_ptr->lane_location_type !=24)){
            //如果左右线是同一条线的split或merge，那么不做成group
            bool is_split = false;
            bool is_merge = false;
            IsTowLineSplit(left_line, right_line, is_split, is_merge);
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " is_split: " <<(int)is_split<<" is_merge: "<<(int)is_merge<<std::endl; 
#endif
            if(is_split == true || is_merge == true){
                double lane_broadth = 0;
                if(CalTwoLineDist(left_line.line_point, right_line.line_point, lane_broadth, 3) == true){
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " lane_broadth: " <<lane_broadth<< ",fabs(lane_broadth):"<<fabs(lane_broadth)<<std::endl; 
#endif
		//判断是否可行驶
                    if(fabs(lane_broadth)<4.5){
                        int left_line_type = 0, right_line_type = 0;
                        int left_line_attached_curb_side = 0, right_line_attached_curb_side = 0;
                        int left_line_id = 0, right_line_id = 0;
                        if(multi_line_index.find(nearest_left) != multi_line_index.end() && multi_line_index.at(nearest_left) == nearest_right 
                            && left_line.back_connect_line_set_ptr.size()>0 && right_line.back_connect_line_set_ptr.size()>0){
                            left_line_type = left_line.back_connect_line_set_ptr.begin()->first->line_type;
                            right_line_type = right_line.back_connect_line_set_ptr.begin()->first->line_type;
                            left_line_attached_curb_side = left_line.back_connect_line_set_ptr.begin()->first->attached_curb_side;
                            right_line_attached_curb_side = right_line.back_connect_line_set_ptr.begin()->first->attached_curb_side;
                            left_line_id = left_line.back_connect_line_set_ptr.begin()->first->id;
                            right_line_id = right_line.back_connect_line_set_ptr.begin()->first->id;
                        }else{
                            if(left_line.base_line_ptr->typ_chg_point < 0){
                                left_line_type = left_line.base_line_ptr->typ_aft_chg_point;
                            }else{
                                left_line_type = left_line.base_line_ptr->line_type;
                            }
                            if(right_line.base_line_ptr->typ_chg_point < 0){
                                right_line_type = right_line.base_line_ptr->typ_aft_chg_point;
                            }else{
                                right_line_type = right_line.base_line_ptr->line_type;
                            }                            
                            
                            left_line_attached_curb_side = left_line.base_line_ptr->attached_curb_side;
                            right_line_attached_curb_side = right_line.base_line_ptr->attached_curb_side;
                            left_line_id = left_line.base_line_ptr->id;
                            right_line_id = right_line.base_line_ptr->id;                            
                        }
                        bool is_drivable =IsDrivableSplitMerge(left_line.line_point, right_line.line_point, left_line_type, right_line_type, left_line_id, right_line_id ,
                                             bev_lines, group_dir, left_line_attached_curb_side, right_line_attached_curb_side);
                        if (is_drivable == false){
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " group not drivable: " <<std::endl; 
#endif                            
                           return false;
                        }
                        
                    }
                }
            }
            
        }
    }

    if(nearest_right>=0 && nearest_left>=0){
        int first_judge_line_index =nearest_right;//先判断group的左线或右线
        int sec_judge_line_index = nearest_left;//后判断的
        if(group_dir == 1){//左侧group,先右后左

        }else if(group_dir ==2){//右侧group,先左后右
            first_judge_line_index =nearest_left;
            sec_judge_line_index = nearest_right;
        }
        //车道本身的右侧的线存在.看下右线是不是分歧或合流，如果是，先判断分歧合流之间是不是可以行驶的车道
        bool is_bev_split_merge_f = false;
        if(multi_line_index.find(first_judge_line_index)!= multi_line_index.end()){
            if(group_dir == 2){
                //如果是右侧group, split、 merge的另外一条线要在 更右边
                if(multi_line_index.at(first_judge_line_index) <first_judge_line_index){
                    is_bev_split_merge_f = true;
                }
            }else if(group_dir == 1){
                //如果是zuo侧group, split、 merge的另外一条线要在 更zuo边
                if(multi_line_index.at(first_judge_line_index) >first_judge_line_index){
                    is_bev_split_merge_f = true;
                }               
            }else{
                //如果是自车的group
                is_bev_split_merge_f = true;
            }
        }
        if(is_bev_split_merge_f == true){
                //如果分歧合流的两条线都是实线，那么不可行驶
                int another_line_index = multi_line_index.at(first_judge_line_index);
                BevLineInnerS* split_merge_left_line = nullptr;//split或merge的左线
                BevLineInnerS* split_merge_right_line = nullptr;//split或merge的右线  
                bool is_drivable = false;    
                int split_left_index =-1, split_right_index = -1; //split,merge的左右线, 是connect line的下标       
                //判断split或merge的左右
                if(connected_lines[first_judge_line_index].split_merge_type == 1){
                    if(connected_lines[first_judge_line_index].split_side == 1){//split的左线
                        split_left_index = first_judge_line_index;
                        split_right_index = another_line_index;
                    }else{
                        split_right_index = first_judge_line_index;
                        split_left_index = another_line_index;
                    }
                }else if(connected_lines[first_judge_line_index].split_merge_type == 2){
                    if(connected_lines[first_judge_line_index].merge_side == 1){//merge的左线
                        split_left_index = first_judge_line_index;
                        split_right_index = another_line_index;
                    }else{
                        split_right_index = first_judge_line_index;
                        split_left_index = another_line_index;
                    }
                }
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " left_index: " <<split_left_index<<" right_index: " <<split_right_index<<std::endl; 
#endif
                if(split_left_index>=0 && split_right_index>=0 && split_left_index<connected_lines.size() && split_right_index<connected_lines.size()){
                    int relative_dir =group_dir;//判断的区域在自车的左边还是右边
                    if(relative_dir == 0){
                        if(first_judge_line_index == nearest_right){
                            relative_dir =2;//在自车的右边
                        }else if(first_judge_line_index == nearest_left){
                            relative_dir =1;//在自车的左边
                        }
                    }
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " relative_dir: " <<relative_dir<<std::endl; 
#endif
                    is_drivable = IsDrivableSplitMerge(connected_lines[split_left_index], connected_lines[split_right_index], connected_lines[first_judge_line_index].split_merge_type, bev_lines,split_merge_left_line, split_merge_right_line, relative_dir);
                }
#ifdef LM_LANEGROUP                
                std::cout << __FILE__ << "," << __LINE__ << "," << " is_drivable: " <<(int)is_drivable<<std::endl; 
                std::cout << __FILE__ << "," << __LINE__ << "," << " split_merge_left_line: " <<split_merge_left_line<<" ,split_merge_right_line: "<<split_merge_right_line<<std::endl; 
#endif
                //如果split区域不可行驶，那么(这里只考虑了split)
                //1.如果不可行驶区域在车道本身的左侧，那么取splith的右线作为接下来判断的左线
                //2. 如果不可行驶区域在车道本身的you侧，那么取splith的zuo线作为接下来判断的右线
                //判断区域在车道本身的左右
                int lane_dir = 0;//区域在车道本身的左右
                if(first_judge_line_index == nearest_right){
                    lane_dir =2;//在车道本身的右边
                }else if(first_judge_line_index == nearest_left){
                    lane_dir =1;//在车道本身的左边
                }
                if(is_drivable == false){
                    //不可行驶，将split和merge的index去掉
                    if(connected_lines[first_judge_line_index].split_merge_type==1 && split_left_index>=0 && split_right_index>=0){//split
                        connected_lines[split_left_index].split_point_index = -1;
                        connected_lines[split_right_index].split_point_index = -1;                        
                    }else if(connected_lines[first_judge_line_index].split_merge_type==2 && split_left_index>=0 && split_right_index>=0){//merge
                        connected_lines[split_left_index].merge_point_index = -1;
                        connected_lines[split_right_index].merge_point_index = -1;                           
                    }
                    int choosed_inner_line_index = -1;//对应bev_line的下标
                    BevLineInnerConnect replace_line;
                    int replace_res = 0;
                    int left_line_index = -1;
                    int right_line_index = -1;

                    if(lane_dir == 1){
                        left_line_index = split_right_index;
                        right_line_index = nearest_right;
                    }else{
                        left_line_index = nearest_left;
                        right_line_index = split_left_index;
                    }
#ifdef LM_LANEGROUP
                    std::cout << __FILE__ << "," << __LINE__ << "," << " left_line_index: " <<left_line_index<<" right_line_index: " <<right_line_index<<std::endl; 
#endif
                    if(left_line_index>=0 && right_line_index>=0 && left_line_index<connected_lines.size() && right_line_index<connected_lines.size()){
                        FindLineInner(connected_lines[left_line_index],connected_lines[right_line_index], bev_lines, choosed_inner_line_index, replace_line, replace_res);
                        GenerateInnerLaneFirstTime(connected_lines[left_line_index], connected_lines[right_line_index], bev_lines, choosed_inner_line_index, replace_line, replace_res, res_lane_group);
                        if(split_merge_left_line!=nullptr && split_merge_right_line!= nullptr){
                            if(lane_dir == 1){
                                ////对于车道本身zuo侧的split merge， 如果you线实线， zuo线虚线，那么
                                if(split_merge_left_line->line_type ==2 && split_merge_right_line->line_type != 2){
                                    res_right_line_index = split_left_index;
                                }
                            }else{
                                //对于车道本身右侧的split merge， 如果左线实线， 右线虚线，那么lanegroup的右线是split_left_index
                                if(split_merge_left_line->line_type !=2 && split_merge_right_line->line_type == 2){
                                    res_left_line_index = split_right_index;
                                }                                
                            }
                        }
                    }
                }else{
                    //如果可行驶，那么看分歧的两条线之间是否有线介于之间                    
                    if(split_merge_left_line != nullptr && split_merge_right_line != nullptr){
                        int choosed_inner_line_index = -1;//对应bev_line的下标
                        BevLineInnerConnect replace_line;
                        int replace_res = 0;
                        BevLineInnerConnect le_line_tmp(split_merge_left_line);
                        BevLineInnerConnect ri_line_tmp(split_merge_right_line);
#ifdef LM_LANEGROUP
                        std::cout << __FILE__ << "," << __LINE__ << "," << " le_line_tmp.base_id: " <<le_line_tmp.base_line_ptr->id<< " ,ri_line_tmp.base_id: " <<ri_line_tmp.base_line_ptr->id<<std::endl; 
#endif
                        FindLineInner(le_line_tmp, ri_line_tmp, bev_lines, choosed_inner_line_index,replace_line, replace_res); 
                        GenerateInnerLaneFirstTime(connected_lines[split_left_index], connected_lines[split_right_index], bev_lines, choosed_inner_line_index, replace_line, replace_res, res_lane_group);
                        if(lane_dir == 1){
                            //车道本身zuo侧的split merge
                            res_left_line_index = split_left_index; 
                        }else{
                            //车道本身you侧的split merge
                            res_right_line_index = split_right_index; 
                        }
                                                
                    }
                    //可行驶的spli或merge判断完了，继续判断split的左线和车道左线
                    int choosed_inner_line_index = -1;//对应bev_line的下标
                    BevLineInnerConnect replace_line;
                    int replace_res = 0;
                    FindLineInner(connected_lines[nearest_left], connected_lines[split_left_index], bev_lines, choosed_inner_line_index, replace_line, replace_res);
                    GenerateInnerLaneFirstTime(connected_lines[nearest_left], connected_lines[split_left_index], bev_lines, choosed_inner_line_index, replace_line, replace_res, res_lane_group);   

                }
            
        }else{
            //不是分叉，看是否有线介于两线之间
            int choosed_inner_line_index = -1;
            BevLineInnerConnect replace_line;//还可能存在不可行驶区域，存在不可行驶的，需要替换边线（原因是bev做出的交叉不稳定）
            int replace_res = 0;
#ifdef LM_LANEGROUP
            std::cout << __FILE__ << "," << __LINE__ << "," << "not split FindLineInner: " <<nearest_left<<" ,"<<nearest_right<<std::endl;
#endif
            FindLineInner(connected_lines[nearest_left], connected_lines[nearest_right], bev_lines, choosed_inner_line_index, replace_line, replace_res); 
#ifdef LM_LANEGROUP    
            std::cout << __FILE__ << "," << __LINE__ << "," << "not split choosed_inner_line_index: " <<choosed_inner_line_index<< " ,replace_res:"<<replace_res<<std::endl;
            if(replace_line.base_line_ptr != nullptr){
                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_line.base_line_ptr->id "<<replace_line.base_line_ptr->id<<std::endl;
                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_line.back_connect_line_set_ptr.size() "<<replace_line.back_connect_line_set_ptr.size()<<" ,back_connrct_id:";
                for(auto iter:replace_line.back_connect_line_set_ptr){
                    std::cout<<" ,"<<iter.first->id;
                }
                std::cout<<std::endl;
            } 
#endif
            GenerateInnerLaneFirstTime(connected_lines[nearest_left], connected_lines[nearest_right], bev_lines, choosed_inner_line_index, replace_line, replace_res, res_lane_group);       
        }


        is_bev_split_merge_f = false;
        if(multi_line_index.find(sec_judge_line_index)!= multi_line_index.end()){
            if(group_dir == 2){
                //如果是右侧group, split、 merge的另外一条线要在 更右边
                if(multi_line_index.at(sec_judge_line_index) <first_judge_line_index){
                    is_bev_split_merge_f = true;
                }
            }else if(group_dir == 1){
                //如果是zuo侧group, split、 merge的另外一条线要在 更zuo边
                if(multi_line_index.at(sec_judge_line_index) >first_judge_line_index){
                    is_bev_split_merge_f = true;
                }               
            }else{
                //如果是自车的group
                is_bev_split_merge_f = true;
            }
        }
        //车道本身的右线判断过了分歧了，现在判断车道本身左线的分歧
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " is_bev_split_merge_f: " <<(int)is_bev_split_merge_f<<std::endl; 
#endif
        if(is_bev_split_merge_f ){
                //如果分歧合流的两条线都是实线，那么不可行驶
                int another_line_index = multi_line_index.at(sec_judge_line_index);
                BevLineInnerS* split_merge_left_line = nullptr;//split或merge的左线
                BevLineInnerS* split_merge_right_line = nullptr;//split或merge的右线  
                bool is_drivable = false;    
                int split_left_index =-1, split_right_index = -1; //split,merge的左右线, 是connect line的下标       
                //判断split或merge的左右
                if(connected_lines[sec_judge_line_index].split_merge_type == 1){
                    if(connected_lines[sec_judge_line_index].split_side == 1){//split的左线
                        split_left_index = sec_judge_line_index;
                        split_right_index = another_line_index;
                    }else{
                        split_right_index = sec_judge_line_index;
                        split_left_index = another_line_index;
                    }
                }
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " left_index: " <<split_left_index<<" right_index: " <<split_right_index<<std::endl; 
#endif
                if(split_left_index>=0 && split_right_index>=0 && split_left_index<connected_lines.size() && split_right_index<connected_lines.size()){
                    int relative_dir =group_dir;//判断的区域在自车的左边还是右边
                    if(relative_dir == 0){
                        if(sec_judge_line_index == nearest_right){
                            relative_dir =2;
                        }else if(sec_judge_line_index == nearest_left){
                            relative_dir =1;
                        }
                    }
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " relative_dir: " <<relative_dir<<" ,sec_judge_line_index:"<<sec_judge_line_index<<" ,connected_lines[sec_judge_line_index].split_merge_type:"<<connected_lines[sec_judge_line_index].split_merge_type<<std::endl; 
#endif
                    is_drivable = IsDrivableSplitMerge(connected_lines[split_left_index], connected_lines[split_right_index], connected_lines[sec_judge_line_index].split_merge_type, bev_lines,split_merge_left_line, split_merge_right_line, relative_dir);
                }
#ifdef LM_LANEGROUP                
                std::cout << __FILE__ << "," << __LINE__ << "," << " is_drivable: " <<(int)is_drivable<<std::endl; 
                std::cout << __FILE__ << "," << __LINE__ << "," << " split_merge_left_line: " <<split_merge_left_line<<" ,split_merge_right_line: "<<split_merge_left_line<<std::endl;
#endif 
                //如果split区域不可行驶，那么(这里只考虑了split)
                //1.如果不可行驶区域在车道本身的左侧，那么取splith的右线作为接下来判断的左线
                //2. 如果不可行驶区域在车道本身的you侧，那么取splith的zuo线作为接下来判断的右线
                int lane_dir = 0;//区域在车道本身的左右
                if(sec_judge_line_index == nearest_right){
                    lane_dir =2;//在车道本身的右边
                }else if(sec_judge_line_index == nearest_left){
                    lane_dir =1;//在车道本身的左边
                }
                if(is_drivable == false){
                    //不可行驶，将split和merge的index去掉,对于这里车道已经做成了，所以在原有的车道里也要去掉
                    if(connected_lines[sec_judge_line_index].split_merge_type==1 && split_left_index>=0 && split_right_index>=0){//split
                        for(auto& lane: res_lane_group){
                            if(lane.left_base_line_ptr->id == connected_lines[split_right_index].base_line_ptr->id && lane.left_line_split_index == connected_lines[split_left_index].split_point_index){
                                lane.left_line_split_index = -1;
                                lane.left_line_split_x = 0;
                            }
                        }
                        connected_lines[split_left_index].split_point_index = -1;
                        connected_lines[split_right_index].split_point_index = -1;                        
                    }else if(connected_lines[sec_judge_line_index].split_merge_type==2 && split_left_index>=0 && split_right_index>=0){//merge
                        for(auto& lane: res_lane_group){
                            if(lane.left_base_line_ptr->id == connected_lines[split_right_index].base_line_ptr->id && lane.left_line_merge_index == connected_lines[split_left_index].merge_point_index){
                                lane.left_line_merge_index = -1;
                                lane.left_line_merge_x = 0;
                            }
                        }
                        connected_lines[split_left_index].merge_point_index = -1;
                        connected_lines[split_right_index].merge_point_index = -1;                           
                    }
                    //已经在右侧里做过了，只要填充group最外侧线index
                    if(split_merge_left_line!=nullptr && split_merge_right_line!= nullptr){
                        if(lane_dir == 1){
                            //对于左侧的split merge， 如果右线实线， 左线虚线，那么lanegroup的左线是 split_right_index
                            if(split_merge_left_line->line_type ==2 && split_merge_right_line->line_type != 2){
                                res_left_line_index = split_right_index;
                            }
                        }else{
                            //对于you侧的split merge， 如果zuo线实线， you线虚线，那么lanegroup的左线是 split_right_index
                            if(split_merge_left_line->line_type !=2 && split_merge_right_line->line_type == 2){
                                res_right_line_index = split_left_index;
                            }
                        }
                    }                    
                }else{
                    //如果可行驶，那么看分歧的两条线之间是否有线介于之间                    
                    if(split_merge_left_line != nullptr && split_merge_right_line != nullptr){
                        int lane_size = res_lane_group.size();
                        int choosed_inner_line_index = -1;//对应bev_line的下标
                        BevLineInnerConnect replace_line;
                        int replace_res = 0;
                        BevLineInnerConnect le_line_tmp(split_merge_left_line);
                        BevLineInnerConnect ri_line_tmp(split_merge_right_line);
#ifdef LM_LANEGROUP
                        std::cout << __FILE__ << "," << __LINE__ << "," << " le_line_tmp.base_id: " <<le_line_tmp.base_line_ptr->id<< " ,ri_line_tmp.base_id: " <<ri_line_tmp.base_line_ptr->id<<std::endl; 
#endif
                        FindLineInner(le_line_tmp, ri_line_tmp,bev_lines, choosed_inner_line_index,replace_line, replace_res); 
                        GenerateInnerLaneFirstTime(connected_lines[split_left_index], connected_lines[split_right_index], bev_lines, choosed_inner_line_index, replace_line, replace_res, res_lane_group); 
                        if(lane_dir == 1){
                            //车道本身zuo侧的split merge
                            res_left_line_index = split_left_index; 
                        }else{
                            //车道本身you侧的split merge
                            res_right_line_index = split_right_index; 
                        }
                        // res_left_line_index = split_left_index;
                        if(group_dir == 2){//因为lane_group里的lane是从 右到左 排列， 对于右侧的group,先做了左边线的lane，所以先push_back了左侧的lane,在这里需要把先做的lane（可能多条）作为一个整体，放到第一个（不改变整体里的顺序）
                            BevLaneElementGroup res_lane_group_temp = res_lane_group;
                            res_lane_group.clear();
                            for(int i= lane_size; i<res_lane_group_temp.size(); i++){
                                res_lane_group.push_back(res_lane_group_temp[i]);
                            }
                            for(int i = 0; i<lane_size; i++){
                                res_lane_group.push_back(res_lane_group_temp[i]);
                            }
                        }                        
                    }

                }
            } 
             
    }
    return true;
}

bool LaneModel::GenerateInnerLaneFirstTime(BevLineInnerConnect& connect_line_left, BevLineInnerConnect& connect_line_right, std::vector<BevLineInnerS>& bev_lines, 
                                  int choosed_inner_line_index, BevLineInnerConnect& replace_line, int replace_res, BevLaneElementGroup& lane_group){
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " GenerateInnerLaneFirstTime start"<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "GenerateInnerLaneFirstTime choosed_inner_line_index: "<<choosed_inner_line_index<<" ,GenerateInnerLaneFirstTime replace_res:"<<replace_res<<std::endl;
#endif
    int replace_line_id = 0;
    if(replace_line.back_connect_line_set_ptr.size()>0){
        replace_line_id = replace_line.back_connect_line_set_ptr.begin()->first->id;
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " 111 replace_line id:"<<replace_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
#endif
    }else if(replace_line.base_line_ptr != nullptr){
        replace_line_id = replace_line.base_line_ptr->id;
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " 222 replace_line id:"<<replace_line.base_line_ptr->id<<std::endl;
#endif
    }
    if(connect_line_left.base_line_ptr == nullptr || connect_line_right.base_line_ptr == nullptr){
        return false;
    }
    if(choosed_inner_line_index>=0){
        //有线介于线之间
        // BevLaneElement lane_ele_tmp;
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " 有线介于线之间,"<<"bev_lines[choosed_inner_line_index].id: "<<bev_lines[choosed_inner_line_index].id <<std::endl;
#endif
        if(replace_line.base_line_ptr!= nullptr){
            //repalce的线和inner线是同一根，替换
            if(replace_res == 1 ){//左线替换,再找一遍
                int choosed_inner_line_index_sub = -1;//对应bev_line的下标
                BevLineInnerConnect replace_line_sub;
                int replace_res_sub = 0;
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " 左线替换,再找一遍"<<std::endl;
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<replace_line.base_line_ptr->id<<" ,right_line_id:"<<connect_line_right.base_line_ptr->id<<std::endl;
                if(replace_line.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<replace_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
                if(connect_line_right.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<connect_line_right.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
#endif
                FindLineInner(replace_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
                GenerateInnerLane(replace_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group);   
                                            
            }else if(replace_res == 2){ // 右线替换,再找一遍
                int choosed_inner_line_index_sub = -1;//对应bev_line的下标
                BevLineInnerConnect replace_line_sub;
                int replace_res_sub = 0;
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " 右线替换,再找一遍"<<std::endl;
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<connect_line_left.base_line_ptr->id<<" ,right_line_id:"<<replace_line.base_line_ptr->id<<std::endl;
                if(connect_line_left.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<connect_line_left.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
                if(replace_line.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<replace_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
#endif
                FindLineInner(connect_line_left, replace_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
                GenerateInnerLane(connect_line_left, replace_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group);  
                
            }            
        }else{
            //youchedao
            BevLineInnerConnect le_line(&(bev_lines[choosed_inner_line_index]));
            int choosed_inner_line_index_sub = -1;//对应bev_line的下标
            BevLineInnerConnect replace_line_sub;
            int replace_res_sub = 0;
#ifdef LM_LANEGROUP
            std::cout << __FILE__ << "," << __LINE__ << "," << " 无替换，右车道再找一遍"<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<le_line.base_line_ptr->id<<" ,right_line_id:"<<connect_line_right.base_line_ptr->id<<std::endl;
            if(le_line.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<le_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
            if(connect_line_right.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<connect_line_right.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
#endif
            FindLineInner(le_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
            GenerateInnerLane(le_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group);

            {//zuochedao
                BevLineInnerConnect ri_line(&(bev_lines[choosed_inner_line_index]));
                int choosed_inner_line_index_sub = -1;//对应bev_line的下标
                BevLineInnerConnect replace_line_sub;
                int replace_res_sub = 0;
#ifdef LM_LANEGROUP
                std::cout << __FILE__ << "," << __LINE__ << "," << " left的车道,再找一遍"<<std::endl;
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<connect_line_left.base_line_ptr->id<<" ,right_line_id:"<<ri_line.base_line_ptr->id<<std::endl;
                if(connect_line_left.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<connect_line_left.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
                if(ri_line.back_connect_line_set_ptr.size()>0){
                    std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<ri_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
                }
#endif
                FindLineInner(connect_line_left, ri_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
                GenerateInnerLane(connect_line_left, ri_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group);             
            }
           
        }
                              
    }else{
        //没线介于之间, 判断是否有replace， 有则替换线
        if(replace_res == 1){//左线替换
            int choosed_inner_line_index_sub = -1;//对应bev_line的下标
            BevLineInnerConnect replace_line_sub;
            int replace_res_sub = 0;
#ifdef LM_LANEGROUP
            std::cout << __FILE__ << "," << __LINE__ << "," << "没线介于之间 左线替换,再找一遍"<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<replace_line.base_line_ptr->id<<" ,right_line_id:"<<connect_line_right.base_line_ptr->id<<std::endl;
            if(replace_line.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<replace_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
            if(connect_line_right.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<connect_line_right.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
#endif
            FindLineInner(replace_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
            GenerateInnerLane(replace_line, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group); 
                                         
        }else if(replace_res == 2){ // 右线替换
            int choosed_inner_line_index_sub = -1;//对应bev_line的下标
            BevLineInnerConnect replace_line_sub;
            int replace_res_sub = 0;
#ifdef LM_LANEGROUP
            std::cout << __FILE__ << "," << __LINE__ << "," << "没线介于之间 右线替换,再找一遍"<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<connect_line_left.base_line_ptr->id<<" ,right_line_id:"<<replace_line.base_line_ptr->id<<std::endl;
            if(connect_line_left.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<connect_line_left.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
            if(replace_line.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<replace_line.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
#endif
            FindLineInner(connect_line_left, replace_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
            GenerateInnerLane(connect_line_left, replace_line, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group); 
             
        }else{//不替换
            int choosed_inner_line_index_sub = -1;//对应bev_line的下标
            BevLineInnerConnect replace_line_sub;
            int replace_res_sub = 0;
#ifdef LM_LANEGROUP
            std::cout << __FILE__ << "," << __LINE__ << "," << "没线介于之间 不替换, 不再找"<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << "left_line_id: "<<connect_line_left.base_line_ptr->id<<" ,right_line_id:"<<connect_line_right.base_line_ptr->id<<std::endl;
            if(connect_line_left.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "left_line back_connect id: "<<connect_line_left.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
            if(connect_line_right.back_connect_line_set_ptr.size()>0){
                std::cout << __FILE__ << "," << __LINE__ << "," << "right_line back_connect id: "<<connect_line_right.back_connect_line_set_ptr.begin()->first->id<<std::endl;
            }
#endif
            // FindLineInner(connect_line_left, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub);
            GenerateInnerLane(connect_line_left, connect_line_right, bev_lines, choosed_inner_line_index_sub, replace_line_sub, replace_res_sub, lane_group); 
 
        }
    
    }
    return true;
}

bool LaneModel::GenerateInnerLane(const BevLineInnerConnect& connect_line_left, const BevLineInnerConnect& connect_line_right, std::vector<BevLineInnerS>& bev_lines, 
                                  int choosed_inner_line_index,const BevLineInnerConnect& replace_line, int replace_res, BevLaneElementGroup& lane_group){
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " GenerateInnerLane start"<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " choosed_inner_line_index: "<<choosed_inner_line_index<<" ,replace_res:"<<replace_res<<std::endl;
#endif
    if(connect_line_left.base_line_ptr == nullptr || connect_line_right.base_line_ptr == nullptr){
        return false;
    }
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " connect_line_left.base_id " <<connect_line_left.base_line_ptr->id;
    if(connect_line_left.back_connect_line_set_ptr.size()>0){
        std::cout<<"left back_connect_id: ";
        for(auto& iter: connect_line_left.back_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }       
    }
    std::cout<<" , connect_line_left.point_size: "<< connect_line_left.line_point.size()<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " connect_line_right.base_id " <<connect_line_right.base_line_ptr->id;
    if(connect_line_right.back_connect_line_set_ptr.size()>0){
        std::cout<<"right back_connect_id: ";
        for(auto& iter: connect_line_right.back_connect_line_set_ptr){
            std::cout<<" ,"<<iter.first->id;
        }       
    }
    std::cout<<" , connect_line_right.point_size: "<< connect_line_right.line_point.size()<<std::endl;
#endif
    if(choosed_inner_line_index>=0){
        //有线介于线之间
        BevLineInnerConnect inner_line(&(bev_lines[choosed_inner_line_index]));
        if(replace_line.base_line_ptr!= nullptr){
            if(replace_res == 1){//左线替换
                //且有替换， 需要判断inner线和 替换线的相对关系
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " 有线介于线之间,且替换左线 " <<std::endl;
#endif
                int rep_to_inner_dir = 0;
                if(JudgeInnerLineAndReplaceLine(replace_line, inner_line, rep_to_inner_dir) == true){
                    if(rep_to_inner_dir ==1){
                        //replace 在inner的左边, 先做最右侧的lane
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " replace线在 inner线左边" <<std::endl;
#endif
                        BevLaneElement lane_ele_tmp;
                        if(GenerateLaneElement(inner_line, connect_line_right ,lane_ele_tmp) == true){
                            lane_group.push_back(lane_ele_tmp);
                        }
                        //再做右侧第二个lane
                        BevLaneElement lane_ele_tmp2;
                        if(GenerateLaneElement(replace_line, inner_line ,lane_ele_tmp2) == true){
                            lane_group.push_back(lane_ele_tmp2);
                        }
                    }else{                        
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " //replace 在inner的右边, 不太可能" <<std::endl;
#endif
                        BevLaneElement lane_ele_tmp;
                        if(GenerateLaneElement(replace_line, connect_line_right ,lane_ele_tmp) == true){
                            lane_group.push_back(lane_ele_tmp);
                        }
                    }
                }else{
                    BevLaneElement lane_ele_tmp;
                    if(GenerateLaneElement(replace_line, connect_line_right ,lane_ele_tmp) == true){
                        lane_group.push_back(lane_ele_tmp);
                    }                    
                }

            }else if(replace_res == 2){ // 右线替换
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " 有线介于线之间,且替换 右线 " <<std::endl;
#endif
                int rep_to_inner_dir = 0;
                if(JudgeInnerLineAndReplaceLine(replace_line, inner_line, rep_to_inner_dir) == true){
                    if(rep_to_inner_dir ==2){
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " replace线在 inner线右边" <<std::endl;
#endif
                        BevLaneElement lane_ele_tmp;
                        if(GenerateLaneElement(inner_line, replace_line ,lane_ele_tmp) == true){
                            lane_group.push_back(lane_ele_tmp);
                        }
                        //再做右侧第二个lane
                        BevLaneElement lane_ele_tmp2;
                        if(GenerateLaneElement(connect_line_left, inner_line ,lane_ele_tmp2) == true){
                            lane_group.push_back(lane_ele_tmp2);
                        }
                    }else{                        
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " //不太可能" <<std::endl;
#endif
                        BevLaneElement lane_ele_tmp;
                        if(GenerateLaneElement(connect_line_left, replace_line  ,lane_ele_tmp) == true){
                            lane_group.push_back(lane_ele_tmp);
                        }
                    }
                }else{ 
                    BevLaneElement lane_ele_tmp;
                    if(GenerateLaneElement(connect_line_left, replace_line ,lane_ele_tmp) == true){
                        lane_group.push_back(lane_ele_tmp); 
                    }                                      
                }
                
            }            
        }else{
            BevLaneElement lane_ele_tmp;
            if(GenerateLaneElement(inner_line, connect_line_right ,lane_ele_tmp) == true){
                lane_group.push_back(lane_ele_tmp);
            }
            BevLaneElement lane_ele_tmp_left;
            if(GenerateLaneElement(connect_line_left, inner_line ,lane_ele_tmp_left) == true){
                lane_group.push_back(lane_ele_tmp_left); 
            }            
        }
                              
    }else{
        //没线介于之间, 判断是否有replace， 有则替换线
        if(replace_res == 1){//左线替换
            BevLaneElement lane_ele_tmp;
            if(GenerateLaneElement(replace_line, connect_line_right ,lane_ele_tmp) == true){
                lane_group.push_back(lane_ele_tmp);
            }
                                         
        }else if(replace_res == 2){ // 右线替换
            BevLaneElement lane_ele_tmp;
            if(GenerateLaneElement(connect_line_left, replace_line ,lane_ele_tmp) == true){
                lane_group.push_back(lane_ele_tmp); 
            }
             
        }else{//不替换
            BevLaneElement lane_ele_tmp;
            if(GenerateLaneElement(connect_line_left, connect_line_right ,lane_ele_tmp) == true){
                lane_group.push_back(lane_ele_tmp); 
            }
 
        }
    
    }
    return true;
}

bool LaneModel::JudgeInnerLineAndReplaceLine(const BevLineInnerConnect& replace_line, const BevLineInnerConnect& inner_line, int& rep_to_inner_dir){
    //rep_to_inner_dir, 1表示replace线在inner线的左边； 2-表示replace线在inner线的右边；
    rep_to_inner_dir = 0;
    if(replace_line.base_line_ptr == nullptr || inner_line.base_line_ptr == nullptr || replace_line.line_point.size() == 0 || inner_line.line_point.size() == 0){
        return false;
    }
    double inner_to_rep_l = 0;
    if(CalTwoLineDist(replace_line.line_point, inner_line.line_point, inner_to_rep_l, 1) == true){
        //res line 在inner线左边的话，l<0, 
        if(inner_to_rep_l<0 && fabs(inner_to_rep_l)>0.5){
            rep_to_inner_dir =1;
        }else if(inner_to_rep_l>0 && fabs(inner_to_rep_l)>0.5){
            rep_to_inner_dir = 2;
        }        
    }else{
        if(replace_line.line_point.front().y > inner_line.line_point.front().y){
            rep_to_inner_dir =1;
        }else{
            rep_to_inner_dir =2;
        }
    }
    return true;
}

bool LaneModel::GenerateLaneElement(const BevLineInnerConnect& line_left, const BevLineInnerConnect& line_right, BevLaneElement& lane){
    //路沿和线不构成道路
    //判断，如果一侧是路沿，那么不做成lane
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " GenerateLaneElement " <<std::endl;
#endif
    if(line_left.base_line_ptr == nullptr || line_right.base_line_ptr == nullptr){
        return false;
    }
    if(line_left.base_line_ptr->lane_location_type == 21 || line_left.base_line_ptr->lane_location_type == 22 || line_left.base_line_ptr->lane_location_type == 24
        ||line_right.base_line_ptr->lane_location_type == 21 || line_right.base_line_ptr->lane_location_type == 22 || line_right.base_line_ptr->lane_location_type == 24){
        double l = 0;
        CalTwoLineDist(line_left.line_point, line_right.line_point, l, 1);
        if(fabs(l)<3.5){
            return false;
        }            
    }

    if(line_left.base_line_ptr != nullptr){
        lane.left_base_line_ptr = line_left.base_line_ptr;
        // MakeSideLine(line_left, lane.left_types, lane.left_line_points,lane.left_line_split_index, lane.left_line_merge_index);
        MakeSideLine(line_left, lane.left_types, lane.left_line_points);
        //debug
        lane.left_line_base_id = line_left.base_line_ptr->id;
        if(line_left.front_connect_line_set_ptr.size()>0){
            lane.left_front_connect_id = line_left.front_connect_line_set_ptr.begin()->first->id;
        }
        if(line_left.back_connect_line_set_ptr.size()>0){
            lane.left_back_connect_id = line_left.back_connect_line_set_ptr.begin()->first->id;
        }
        lane.left_line_split_index = line_left.split_point_index;
        lane.left_line_merge_index = line_left.merge_point_index;
        if(line_left.split_point_index>=0 && line_left.split_point_index<line_left.line_point.size()){
            lane.left_line_split_x = line_left.line_point[line_left.split_point_index].x;
        }
        if(line_left.merge_point_index>=0 && line_left.merge_point_index<line_left.line_point.size()){
            lane.left_line_merge_x = line_left.line_point[line_left.merge_point_index].x;
        }
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " lane.left_line_base_id: " <<lane.left_line_base_id
        << " line_left.front_connect_line_set_ptr.size(): " <<line_left.front_connect_line_set_ptr.size()
        << " lane.left_front_connect_id: " <<lane.left_front_connect_id<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << "," << " line_left.back_connect_line_set_ptr.size(): " <<line_left.back_connect_line_set_ptr.size()
        << " lane.left_back_connect_id: " <<lane.left_back_connect_id<<std::endl;
#endif
    }
    if(line_right.base_line_ptr != nullptr){
        lane.right_base_line_ptr = line_right.base_line_ptr;
        // MakeSideLine(line_right, lane.right_types, lane.right_type_sec, lane.right_line_points,lane.right_line_split_index, lane.right_line_merge_index);
        MakeSideLine(line_right, lane.right_types, lane.right_line_points);
        //debug
        lane.right_line_base_id = line_right.base_line_ptr->id;
        if(line_right.front_connect_line_set_ptr.size()>0){
            lane.right_front_connect_id = line_right.front_connect_line_set_ptr.begin()->first->id;
        }
        if(line_right.back_connect_line_set_ptr.size()>0){
            lane.right_back_connect_id = line_right.back_connect_line_set_ptr.begin()->first->id;
        }
        lane.right_line_split_index = line_right.split_point_index;
        lane.right_line_merge_index = line_right.merge_point_index;
        if(line_right.split_point_index>=0 && line_right.split_point_index<line_right.line_point.size()){
            lane.right_line_split_x = line_right.line_point[line_right.split_point_index].x;
        }
        if(line_right.merge_point_index>=0 && line_right.merge_point_index<line_right.line_point.size()){
            lane.right_line_merge_x = line_right.line_point[line_right.merge_point_index].x;
        }
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " lane.right_line_base_id: " <<lane.right_line_base_id
        << " line_right.front_connect_line_set_ptr.size(): " <<line_right.front_connect_line_set_ptr.size()
        << " lane.right_front_connect_id: " <<lane.right_front_connect_id<<std::endl;

        std::cout << __FILE__ << "," << __LINE__ << "," <<" line_right.back_connect_line_set_ptr.size(): " <<line_right.back_connect_line_set_ptr.size()
         << " lane.right_back_connect_id: " <<lane.right_back_connect_id<<std::endl;
#endif
    }
    return true;

}

bool LaneModel::MakeSideLine(const BevLineInnerConnect& bev_line, std::vector<LineTypeInfo>& types, EFMRefLinePoints &line_points){
    line_points.clear();
    types.clear();
    if(bev_line.base_line_ptr != nullptr){
        line_points = bev_line.line_point;
        if(bev_line.base_line_ptr->first_valid == true && bev_line.base_line_ptr->FirstStartPoint>0.001){
            //base line 在自车前方。 type用front connect的
            LineTypeInfo type_info1, type_info2;
            if(bev_line.front_connect_line_set_ptr.size()>0 && bev_line.front_connect_line_set_ptr.begin()->first->first_valid){
                type_info1.is_valid = true;
                if(bev_line.front_connect_line_set_ptr.begin()->first->MinusStartPoint<-0.001){//负段存在
                    type_info1.start_point = bev_line.front_connect_line_set_ptr.begin()->first->MinusStartPoint;
                }else{
                    type_info1.start_point = bev_line.front_connect_line_set_ptr.begin()->first->FirstStartPoint;
                }               
                type_info1.line_type = bev_line.front_connect_line_set_ptr.begin()->first->line_type;
                type_info1.typ_chg_point = bev_line.front_connect_line_set_ptr.begin()->first->typ_chg_point;
                type_info1.typ_aft_chg_point = bev_line.front_connect_line_set_ptr.begin()->first->typ_aft_chg_point;
                if(bev_line.base_line_ptr->first_valid){
                    type_info2.is_valid = true;
                    type_info2.start_point = bev_line.base_line_ptr->FirstStartPoint;
                    type_info2.line_type = bev_line.base_line_ptr->line_type;
                    type_info2.typ_chg_point = bev_line.base_line_ptr->typ_chg_point;
                    type_info2.typ_aft_chg_point = bev_line.base_line_ptr->typ_aft_chg_point;                    
                }                
            }else if(bev_line.base_line_ptr->first_valid){
                type_info1.is_valid = true;
                if(bev_line.base_line_ptr->MinusStartPoint<-0.001){//负段存在
                    type_info1.start_point = bev_line.base_line_ptr->MinusStartPoint;
                }else{
                    type_info1.start_point = bev_line.base_line_ptr->FirstStartPoint;
                }                
                type_info1.line_type = bev_line.base_line_ptr->line_type;
                type_info1.typ_chg_point = bev_line.base_line_ptr->typ_chg_point;
                type_info1.typ_aft_chg_point = bev_line.base_line_ptr->typ_aft_chg_point; 
                if(bev_line.back_connect_line_set_ptr.size()>0 && bev_line.back_connect_line_set_ptr.begin()->first->first_valid){
                    type_info2.is_valid = true;
                    if(bev_line.back_connect_line_set_ptr.begin()->first->MinusStartPoint<-0.001){//负段存在
                        type_info2.start_point = bev_line.back_connect_line_set_ptr.begin()->first->MinusStartPoint;
                    }else{
                        type_info2.start_point = bev_line.back_connect_line_set_ptr.begin()->first->FirstStartPoint;
                    }                    
                    type_info2.line_type = bev_line.back_connect_line_set_ptr.begin()->first->line_type;
                    type_info2.typ_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_chg_point;
                    type_info2.typ_aft_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point;                    
                }
            }
            //填充
            if(type_info1.is_valid == true){
                types.push_back(type_info1);
                if(type_info2.is_valid == true){
                    types.push_back(type_info2);
                }
            }            
        }else if(bev_line.base_line_ptr->first_valid == true && bev_line.base_line_ptr->MinusEndPoint<-0.001){
            LineTypeInfo type_info1;
            //base line 在自车尾后。 type用back connect的
            type_info1.is_valid = true;
            type_info1.start_point = bev_line.back_connect_line_set_ptr.begin()->first->FirstStartPoint;
            type_info1.line_type = bev_line.back_connect_line_set_ptr.begin()->first->line_type;
            type_info1.typ_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_chg_point;
            type_info1.typ_aft_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point;

            if(type_info1.is_valid == true){
                types.push_back(type_info1);
            } 
        }else{//按照目前的连线逻辑，base_line都是经过自车的，所以理论上都只会走到这里！！！！！！！！！！！！！！！！！！
            LineTypeInfo type_info1, type_info2;
            if(bev_line.base_line_ptr->first_valid){
                type_info1.is_valid = true;
                if(bev_line.base_line_ptr->MinusStartPoint<-0.001){//负段存在
                    type_info1.start_point = bev_line.base_line_ptr->MinusStartPoint;
                }else{
                    type_info1.start_point = bev_line.base_line_ptr->FirstStartPoint;
                }                
                type_info1.line_type = bev_line.base_line_ptr->line_type;
                type_info1.typ_chg_point = bev_line.base_line_ptr->typ_chg_point;
                type_info1.typ_aft_chg_point = bev_line.base_line_ptr->typ_aft_chg_point; 

                if(bev_line.back_connect_line_set_ptr.size()>0 && bev_line.back_connect_line_set_ptr.begin()->first->first_valid){
                    type_info2.is_valid = true;
                    if(bev_line.back_connect_line_set_ptr.begin()->first->MinusStartPoint<-0.001){//负段存在
                        type_info2.start_point = bev_line.back_connect_line_set_ptr.begin()->first->MinusStartPoint;
                    }else{
                        type_info2.start_point = bev_line.back_connect_line_set_ptr.begin()->first->FirstStartPoint;
                    }                    
                    type_info2.line_type = bev_line.back_connect_line_set_ptr.begin()->first->line_type;
                    type_info2.typ_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_chg_point;
                    type_info2.typ_aft_chg_point = bev_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point;                    
                }                
            } 
            if(type_info1.is_valid == true){
                types.push_back(type_info1);
                if(type_info2.is_valid == true){
                    types.push_back(type_info2);
                }
            }             
        }
    }
    // split_index = bev_line.split_point_index;
    // merge_index = bev_line.merge_point_index;
    if(types.size() == 2){
        //如果两段的线型都一样，只保留一段
        if(types[1].typ_chg_point>10000 && types[1].line_type == types[0].typ_aft_chg_point){//第1段的后一段类型和第2段的前一段类型一样，且第2段只有一个类型
            types.pop_back();
        }
    }
    return true;

}

bool LaneModel::GetSplitMergeLine(BevLineInnerConnect& connect_line1, BevLineInnerConnect& connect_line2, int is_split_merge, BevLineInnerS* &split_left_line, BevLineInnerS* &split_right_line){
    //connect_line1split左线， connect_line2split右线， dir 1左 2右，表示 split在自车的左右， 如果split在自车右边，那么split的左线是虚线才可行驶， 反之split在自车左边，那么split的右线是虚线才可行驶
    split_left_line = nullptr;
    split_right_line = nullptr;
    //case1, 分歧合流区域，看两条线是不是实线, is_split_merge 1-split,2-merge
    int line1_type = 2, line2_type = 2;//虚线，这里是找分歧合流段的type
    EFMRefLinePoints line1_points{}, line2_points{};
    int line1_id = 0, line2_id = 0;
    if(is_split_merge == 1){
        if(connect_line1.back_connect_line_set_ptr.size()>0){
            line1_type = connect_line1.back_connect_line_set_ptr.begin()->first->line_type;
            line1_points = connect_line1.back_connect_line_set_ptr.begin()->first->total_line;
            line1_id = connect_line1.back_connect_line_set_ptr.begin()->first->id;
            split_left_line = connect_line1.back_connect_line_set_ptr.begin()->first;
        }
        if(connect_line2.back_connect_line_set_ptr.size()>0){
            line2_type = connect_line2.back_connect_line_set_ptr.begin()->first->line_type;
            line2_points = connect_line2.back_connect_line_set_ptr.begin()->first->total_line;
            line2_id = connect_line2.back_connect_line_set_ptr.begin()->first->id;
            split_right_line = connect_line2.back_connect_line_set_ptr.begin()->first;
        }
        // std::cout << __FILE__ << "," << __LINE__ << "," << " line1_type: " <<line1_type<<" ,line2_type:"<<line2_type<<" ,line1_id: "<<line1_id<<" ,line2_id:"<<line2_id<<std::endl; 
        // std::cout << __FILE__ << "," << __LINE__ << "," << " split_left_line: " <<split_left_line<<" ,split_right_line:"<<split_right_line<<std::endl; 
    }else if(is_split_merge == 2){
        if(connect_line1.front_connect_line_set_ptr.size()>0){
            line1_type = connect_line1.front_connect_line_set_ptr.begin()->first->line_type;
            line1_points = connect_line1.front_connect_line_set_ptr.begin()->first->total_line;
            line1_id = connect_line1.front_connect_line_set_ptr.begin()->first->id;
            split_left_line = connect_line1.front_connect_line_set_ptr.begin()->first;
        }
        if(connect_line2.front_connect_line_set_ptr.size()>0){
            line2_type = connect_line2.front_connect_line_set_ptr.begin()->first->line_type;
            line2_points = connect_line2.front_connect_line_set_ptr.begin()->first->total_line;
            line2_id = connect_line2.front_connect_line_set_ptr.begin()->first->id;
            split_right_line = connect_line2.front_connect_line_set_ptr.begin()->first;
        }
    }
    return true;
}

bool LaneModel::IsDrivableSplitMerge(BevLineInnerConnect& connect_line1, BevLineInnerConnect& connect_line2, int is_split_merge, const std::vector<BevLineInnerS>& bev_lines, 
                                     BevLineInnerS* &split_left_line, BevLineInnerS* &split_right_line, int dir){
    if(connect_line1.base_line_ptr == nullptr || connect_line1.base_line_ptr == nullptr){
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " connect_line1.base_line_ptr: " <<connect_line1.base_line_ptr<<" ,connect_line1.base_line_ptr:"<<connect_line1.base_line_ptr<<std::endl; 
#endif
        return false;
    }
    //connect_line1split左线， connect_line2split右线， dir 1左 2右，表示 split在自车的左右， 如果split在自车右边，那么split的左线是虚线才可行驶， 反之split在自车左边，那么split的右线是虚线才可行驶
    split_left_line = nullptr;
    split_right_line = nullptr;
    //case1, 分歧合流区域，看两条线是不是实线, is_split_merge 1-split,2-merge
    int line1_type = 2, line2_type = 2;//虚线，这里是找分歧合流段的type
    EFMRefLinePoints line1_points{}, line2_points{};
    int line1_id = 0, line2_id = 0;
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " is_split_merge: " <<is_split_merge<<std::endl; 
#endif
    if(is_split_merge == 1){
        if(connect_line1.back_connect_line_set_ptr.size()>0){
            line1_type = connect_line1.back_connect_line_set_ptr.begin()->first->line_type;
            line1_points = connect_line1.back_connect_line_set_ptr.begin()->first->total_line;
            line1_id = connect_line1.back_connect_line_set_ptr.begin()->first->id;
            split_left_line = connect_line1.back_connect_line_set_ptr.begin()->first;
        }
        if(connect_line2.back_connect_line_set_ptr.size()>0){
            line2_type = connect_line2.back_connect_line_set_ptr.begin()->first->line_type;
            line2_points = connect_line2.back_connect_line_set_ptr.begin()->first->total_line;
            line2_id = connect_line2.back_connect_line_set_ptr.begin()->first->id;
            split_right_line = connect_line2.back_connect_line_set_ptr.begin()->first;
        }
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " line1_type: " <<line1_type<<" ,line2_type:"<<line2_type<<" ,line1_id: "<<line1_id<<" ,line2_id:"<<line2_id<<std::endl; 
        // std::cout << __FILE__ << "," << __LINE__ << "," << " split_left_line id: " <<split_left_line->id<<" ,split_right_line id:"<<split_right_line->id<<std::endl; 
#endif
        if(dir == 1){
            if(line2_type != 2){
                return false;
            }            
        }else if(dir ==2){//如果split在自车右边，那么split的左线是虚线才可行驶
            if(line1_type != 2){
                return false;
            }             
        }else{
            if(line1_type != 2 && line2_type!=2){
                return false;
            }            
        }

    }else if(is_split_merge == 2){
        if(connect_line1.base_line_ptr->id != connect_line2.base_line_ptr->id){//自车位置车头前方的merge
            line1_type = connect_line1.base_line_ptr->line_type;
            line1_points = connect_line1.base_line_ptr->total_line;
            line1_id = connect_line1.base_line_ptr->id;
            split_left_line = connect_line1.base_line_ptr;

            line2_type = connect_line2.base_line_ptr->line_type;
            line2_points = connect_line2.base_line_ptr->total_line;
            line2_id = connect_line2.base_line_ptr->id;
            split_right_line = connect_line2.base_line_ptr;
        }else{
            if(connect_line1.front_connect_line_set_ptr.size()>0){
                line1_type = connect_line1.front_connect_line_set_ptr.begin()->first->line_type;
                line1_points = connect_line1.front_connect_line_set_ptr.begin()->first->total_line;
                line1_id = connect_line1.front_connect_line_set_ptr.begin()->first->id;
                split_left_line = connect_line1.front_connect_line_set_ptr.begin()->first;
            }
            if(connect_line2.front_connect_line_set_ptr.size()>0){
                line2_type = connect_line2.front_connect_line_set_ptr.begin()->first->line_type;
                line2_points = connect_line2.front_connect_line_set_ptr.begin()->first->total_line;
                line2_id = connect_line2.front_connect_line_set_ptr.begin()->first->id;
                split_right_line = connect_line2.front_connect_line_set_ptr.begin()->first;
            }            
        }

#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " line1_type: " <<line1_type<<" ,line2_type:"<<line2_type<<" ,line1_id: "<<line1_id<<" ,line2_id:"<<line2_id<<std::endl; 
        // std::cout << __FILE__ << "," << __LINE__ << "," << " split_left_line id: " <<split_left_line->id<<" ,split_right_line id:"<<split_right_line->id<<std::endl; 
#endif
        if(dir == 1){
            if(line2_type != 2){
                return false;
            }            
        }else if(dir ==2){//如果split在自车右边，那么split的左线是虚线才可行驶
            if(line1_type != 2){
                return false;
            }             
        }else{
            if(line1_type != 2 && line2_type!=2){
                return false;
            }            
        }
    }

    //case new, 如果左线有右侧的attach路沿， 或者右线有左侧的attach路沿，那么认为不可行驶
    // if(connect_line1.base_line_ptr->attached_curb_side == 2 || connect_line2.base_line_ptr->attached_curb_side == 1 || connect_line1.base_line_ptr->attached_curb_side == 3 || connect_line2.base_line_ptr->attached_curb_side == 3){
    if((split_right_line != nullptr && (split_right_line->attached_curb_side ==1 || split_right_line->attached_curb_side ==3))
       ||(split_left_line != nullptr && (split_left_line->attached_curb_side ==2 || split_left_line->attached_curb_side ==3)) ){
    //左线有右侧的attach路沿， 或者右线有左侧的attach路沿
#ifdef LM_LANEGROUP
        std::cout << __FILE__ << "," << __LINE__ << "," << " split_right_line->attached_curb_side:"<<split_right_line->attached_curb_side<<" ,split_left_line->attached_curb_side:"<<split_left_line->attached_curb_side<<std::endl; 
        std::cout << __FILE__ << "," << __LINE__ << "," << " attach curb lead to not driveable"<<std::endl; 
#endif
        return false;
    }

//     //case2,看两线之间是不是有路沿，有，认为不可行驶
//     if(line1_points.size()==0 || line2_points.size()==0){
//         return false;//异常
//     }
//     std::vector<EFMPoint> polygon{};
//     int p_index = 0;
//     for(; p_index<line1_points.size(); p_index+=4){
//         polygon.push_back(line1_points[p_index]);
//     }
//     if(p_index>line1_points.size()-1 && p_index!= line1_points.size()+3){
//         //最后一个点填充，且不重复填充
//         polygon.push_back(line1_points.back());
//     }
//     p_index =line2_points.size()-1;
//     for(; p_index>=0; p_index-=4){
//         polygon.push_back(line2_points[p_index]);
//     }
//     if(p_index<0 && p_index!= -4){
//         //最后一个点填充，且不重复填充
//         polygon.push_back(line2_points.front());
//     }
//     //如果一条线的一段距离都落在两条线之间
//     bool is_first_find = true;
//     for(int i = 0; i<bev_lines.size(); i++){
//         BevLineInnerS line = bev_lines[i];
// #ifdef LM_GERLANE
//                 std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<std::endl;
// #endif
//         if(line.id != line1_id && line.id != line2_id  && line.is_connected_f == false && line.total_line.size()>2 && line.is_inner_line_f == false){
//             double accum_s = 0;
//             //为了节约算力，不对线的每个点都算，先取首尾中三个点，都不在区间就pass
//             int mid_index = line.total_line.size()/2;
// // #ifdef RELOCATE_INNER
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " mid_index: " <<mid_index<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_front=[" <<line.total_line.front().x<<" ,"<<line.total_line.front().y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.front(), polygon)<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_mid=[" <<line.total_line.at(mid_index).x<<" ,"<<line.total_line.at(mid_index).y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.at(mid_index), polygon)<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_back=[" <<line.total_line.back().x<<" ,"<<line.total_line.back().y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.back(), polygon)<<std::endl;
// // #endif
//             if(CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.front(), polygon)== true || 
//                 CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.at(mid_index), polygon)== true ||
//                 CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.back(), polygon)== true){
// #ifdef LM_GERLANE
//                         std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,is between"<<std::endl;
// #endif
//                 if(line.lane_location_type == 21 || line.lane_location_type == 22 || line.lane_location_type == 24){
//                     return false;
//                 }
//             }
//         }
//     }
    if(is_split_merge == 1){
        connect_line1.split_erea_is_drivable = true;
        connect_line2.split_erea_is_drivable = true;
    }else if(is_split_merge ==2){
        connect_line1.merge_erea_is_drivable = true;
        connect_line2.merge_erea_is_drivable = true;
    }
    
    return true;
}

bool LaneModel::IsDrivableSplitMerge(const EFMRefLinePoints& line1_points, const EFMRefLinePoints& line2_points, int line1_type, int line2_type, int line1_id, int line2_id ,const std::vector<BevLineInnerS>& bev_lines, int dir, int line1_attached_curb_side, int line2_attached_curb_side){
    //line1 左线， line2 右线
    //is_split_merge 1split, 2merge
     //dir 1左 2右，表示 split/merge在自车的左右， 如果split在自车右边，那么split的左线是虚线才可行驶， 反之split在自车左边，那么split的右线是虚线才可行驶
    // case1 线型
#ifdef RELOCATE_INNER
    std::cout << __FILE__ << "," << __LINE__ << "," << " line1_type: " <<line1_type<<" ,line2_type:"<<line2_type<<" ,line1_id: "<<line1_id<<" ,line2_id:"<<line2_id <<" .dir:"<<dir
    <<",line1_attached_curb_side:"<<line1_attached_curb_side<<" ,line2_attached_curb_side"<<line2_attached_curb_side<<std::endl; 
#endif
    if(dir == 1){
        if(line2_type != 2){
            return false;
        }            
    }else if(dir ==2){//如果split在自车右边，那么split的左线是虚线才可行驶
        if(line1_type != 2){
            return false;
        }             
    }else{
        if(line1_type != 2 && line2_type!=2){
            return false;
        }            
    }
    //case new 
    //如果左线有右侧的attach路沿， 或者右线有左侧的attach路沿，那么认为不可行驶
    if(line1_attached_curb_side == 2 || line2_attached_curb_side == 1 || line1_attached_curb_side == 3 || line2_attached_curb_side == 3){
        //左线有右侧的attach路沿， 或者右线有左侧的attach路沿
        return false;
    }

#ifdef RELOCATE_INNER
    std::cout << __FILE__ << "," << __LINE__ << "," << " type is ok: " <<std::endl;
#endif
//     //case2,看两线之间是不是有路沿，有，认为不可行驶
//     if(line1_points.size()==0 || line2_points.size()==0){
//         return false;//异常
//     }
//     std::vector<EFMPoint> polygon{};
//     int p_index = 0;
//     for(; p_index<line1_points.size(); p_index+=4){
//         polygon.push_back(line1_points[p_index]);
//     }
//     if(p_index>line1_points.size()-1 && p_index!= line1_points.size()+3){
//         //最后一个点填充，且不重复填充
//         polygon.push_back(line1_points.back());
//     }
//     p_index =line2_points.size()-1;
//     for(; p_index>=0; p_index-=4){
//         polygon.push_back(line2_points[p_index]);
//     }
//     if(p_index<0 && p_index!= -4){
//         //最后一个点填充，且不重复填充
//         polygon.push_back(line2_points.front());
//     }
//     //如果一条线的一段距离都落在两条线之间
//     bool is_first_find = true;
//     for(int i = 0; i<bev_lines.size(); i++){
//         BevLineInnerS line = bev_lines[i];
// #ifdef RELOCATE_INNER
//                 // std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<std::endl;
// #endif
//         if(line.id != line1_id && line.id != line2_id  && line.is_connected_f == false && line.total_line.size()>2 && line.is_inner_line_f == false){
//             double accum_s = 0;
//             //为了节约算力，不对线的每个点都算，先取首尾中三个点，都不在区间就pass
//             int mid_index = line.total_line.size()/2;
// // #ifdef RELOCATE_INNER
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " mid_index: " <<mid_index<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_front=[" <<line.total_line.front().x<<" ,"<<line.total_line.front().y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.front(), polygon)<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_mid=[" <<line.total_line.at(mid_index).x<<" ,"<<line.total_line.at(mid_index).y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.at(mid_index), polygon)<<std::endl;
// //                     std::cout << __FILE__ << "," << __LINE__ << "," << " line.total_line_back=[" <<line.total_line.back().x<<" ,"<<line.total_line.back().y<<"], res:"<<(int)CommonMathMethod::DiscretePointsMath::InPolygon(line.total_line.back(), polygon)<<std::endl;
// // #endif
//             if(CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.front(), polygon)== true || 
//                 CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.at(mid_index), polygon)== true ||
//                 CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.back(), polygon)== true){
// #ifdef RELOCATE_INNER
//                         // std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,is between"<<std::endl;
// #endif
//                 if(line.lane_location_type == 21 || line.lane_location_type == 22 || line.lane_location_type == 24){
// #ifdef RELOCATE_INNER
//                     // std::cout << __FILE__ << "," << __LINE__ << "," << " line.lane_location_type: " <<line.lane_location_type<<std::endl;
// #endif
//                     return false;
//                 }
//             }
//         }
//     }
#ifdef RELOCATE_INNER
    std::cout << __FILE__ << "," << __LINE__ << "," << " inner is ok: " <<std::endl;
#endif
    return true;
}

bool LaneModel::IsTowLineSplit(const BevLineInnerConnect& left_line, const BevLineInnerConnect& right_line, bool& is_split, bool& is_merge){   
    std::vector<EFMPoint> left_side_line = left_line.line_point;
    std::vector<EFMPoint> right_side_line =  right_line.line_point;
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << "enter IsTowLineSplit: left_side_line.size():"<<left_side_line.size()<<" ,right_side_line.size():"<<right_side_line.size()<< std::endl;
#endif 
    is_split = false;
    is_merge = false;
    if(left_side_line.size()<2 || right_side_line.size()<2 || left_line.base_line_ptr == nullptr || right_line.base_line_ptr == nullptr){
        return false;
    }
    //计算两条线头尾的l
    EFMPoint left_line_front_p = left_side_line.front();
    EFMPoint left_line_back_p = left_side_line.back();
    EFMPoint right_line_front_p = right_side_line.front();
    EFMPoint right_line_back_p = right_side_line.back();
    double left_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(left_side_line);
    double right_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(right_side_line);

    PointSLd front_sl, back_sl;
    bool is_inside = false;
    CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_side_line, left_line_front_p, front_sl, is_inside);   
    if(is_inside == false && fabs(front_sl.s)>0.5){
        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_side_line, right_line_front_p, front_sl, is_inside); 
        if(is_inside == false && fabs(front_sl.s)>0.5){
            return false;
        }
    }

    is_inside = false;
    CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_side_line, left_line_back_p, back_sl, is_inside);   
    if(is_inside == false && fabs(back_sl.s - right_length)>0.5){
        // std::cout << __FILE__ << "," << __LINE__ << "," << "333333333333:"<<",back_sl.l::"<<back_sl.l<<", back_sl.s:"<<back_sl.s<< std::endl;
        CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_side_line, right_line_back_p, back_sl, is_inside); 
        if(is_inside == false && fabs(back_sl.s - left_length)>0.5){
            // std::cout << __FILE__ << "," << __LINE__ << "," << "444444444444444:"<<",back_sl.l::"<<back_sl.l<<", back_sl.s:"<<back_sl.s<< std::endl;
            return false;
        }
    }
#ifdef LM_LANEGROUP
    std::cout << __FILE__ << "," << __LINE__ << "," << " front_sl.l: " <<front_sl.l<<" ,back_sl.l:"<<back_sl.l<<std::endl;
#endif
    if((fabs(back_sl.l)-fabs(front_sl.l))>0.8 && fabs(front_sl.l)<2){
        is_split = true;
    }
    if((fabs(front_sl.l)-fabs(back_sl.l))>0.8 && fabs(back_sl.l)<2){
        is_merge = true;
    }
    return true;
}

bool LaneModel::FindLineInner(BevLineInnerConnect& left_line, BevLineInnerConnect& right_line, std::vector<BevLineInnerS>& bev_lines, int &choosed_inner_line_index,BevLineInnerConnect& inner_line_replace_res, int& replace_res){
#ifdef RELOCATE_INNER    
    std::cout << __FILE__ << "," << __LINE__ << "," << "start FindLineInner: " <<std::endl;
#endif
    replace_res = 0; //1-代替左线，2-代替右线
    inner_line_replace_res.base_line_ptr =nullptr;
    inner_line_replace_res.line_point.clear();
    inner_line_replace_res.front_connect_line_set_ptr.clear();
    inner_line_replace_res.back_connect_line_set_ptr.clear();
    std::vector<EFMPoint> left_side_line = left_line.line_point;
    std::vector<EFMPoint> right_side_line =  right_line.line_point;
    if(left_side_line.size()<2 || right_side_line.size()<2 || left_line.base_line_ptr == nullptr || right_line.base_line_ptr == nullptr){
        return false;
    }
    int left_line_id = left_line.base_line_ptr->id;
    int right_line_id = right_line.base_line_ptr->id; 
    std::vector<EFMPoint> polygon{};
    //取x的共同部分
    double common_x_start = std::max(left_side_line.front().x,right_side_line.front().x);
    double common_x_end = std::min(left_side_line.back().x,right_side_line.back().x);
    int p_index = 0;
    for(; p_index<left_side_line.size(); p_index++){
        if(left_side_line[p_index].x>= common_x_start && left_side_line[p_index].x<= common_x_end){
            polygon.push_back(left_side_line[p_index]);
        }       
    }
    p_index =right_side_line.size()-1;
    for(; p_index>=0; p_index--){
        if(right_side_line[p_index].x>= common_x_start && right_side_line[p_index].x<= common_x_end){
            polygon.push_back(right_side_line[p_index]);
        }
    }
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," <<"common_x_start: "<<common_x_start<<" ,common_x_end:"<<common_x_end<< " ,polygon.size(): " <<polygon.size()<<std::endl;
            std::stringstream ss, ss1;
            ss<< "polygon_x = [";
            ss1<< "polygon_y = [";
            for(int i = 0; i< polygon.size();i++){
                if(i== polygon.size()-1){
                    ss<<polygon[i].x;
                    ss1<<polygon[i].y;
                }else{
                    ss<<polygon[i].x<<",";
                    ss1<<polygon[i].y<<",";
                }
            }
            std::cout <<ss.str()<<"]"<<std::endl;
            std::cout <<ss1.str()<<"]"<<std::endl;
#endif
            //如果一条线的一段距离都落在两条线之间
    bool is_first_find = true;
    std::vector<int> inner_line_index_vec{};//保存介于线之间的bev_Lines的下标
    choosed_inner_line_index = -1;//x距离自车最近的index, bev_lines的index
    for(int i = 0; i<bev_lines.size(); i++){
        BevLineInnerS& line = bev_lines[i];
        bool is_side_line_f = false;
        if(line.id == left_line_id || line.id == right_line_id){
            is_side_line_f = true;
        }
        if(left_line.back_connect_line_set_ptr.size()>0){
            if(left_line.back_connect_line_set_ptr.begin()->first->id == line.id){
                is_side_line_f = true;
            }
        }
        if(right_line.back_connect_line_set_ptr.size()>0){
            if(right_line.back_connect_line_set_ptr.begin()->first->id == line.id){
                is_side_line_f = true;
            }
        }
#ifdef RELOCATE_INNER
                std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" bev_line.lane_location_type: "<<line.lane_location_type<<" ,left_line_id:"<<left_line_id<<" ,right_line_id:"<<right_line_id<<std::endl;
#endif
        if(line.id != left_line_id && line.id != right_line_id && line.lane_location_type !=22 && line.lane_location_type !=21 && line.lane_location_type !=24 &&
           line.total_line.size()>2 && line.is_connected_f == false && is_side_line_f == false && line.id<10000 && line.total_line.back().x>0){
            double accum_s = 0;
            //为了节约算力，不对线的每个点都算，先取首尾中三个点，都不在区间就pass
            int mid_index = line.total_line.size()/2;
#ifdef RELOCATE_INNER
                    // std::cout << __FILE__ << "," << __LINE__ << "," << " mid_index: " <<mid_index
                    // << "," << " line.total_line_front=[" <<line.total_line.front().x<<" ,"<<line.total_line.front().y<<"], res:"<<(int)CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.front(), polygon)
                    // << "," << " line.total_line_mid=[" <<line.total_line.at(mid_index).x<<" ,"<<line.total_line.at(mid_index).y<<"], res:"<<(int)CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.at(mid_index), polygon)
                    // << "," << " line.total_line_back=[" <<line.total_line.back().x<<" ,"<<line.total_line.back().y<<"], res:"<<(int)CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.back(), polygon)<<std::endl;
                    //     std::stringstream ss2, ss3;
                    //     ss2<< "line_x = [";
                    //     ss3<< "line_y = [";
                    //     for(int i = 0; i< line.total_line.size();i++){
                    //         if(i== line.total_line.size()-1){
                    //             ss2<<line.total_line[i].x;
                    //             ss3<<line.total_line[i].y;
                    //         }else{
                    //             ss2<<line.total_line[i].x<<",";
                    //             ss3<<line.total_line[i].y<<",";
                    //         }
                    //     }
                    //     std::cout <<ss2.str()<<"]"<<std::endl;
                    //     std::cout <<ss3.str()<<"]"<<std::endl;
#endif
            if(CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.front(), polygon)== true || 
               CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.at(mid_index), polygon)== true ||
               CommonTool::DiscretePointsMath::GetInstance()->InPolygon(line.total_line.back(), polygon)== true){
#ifdef RELOCATE_INNER
                        std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,is between"<<std::endl;
#endif
                        int start_inside_index = -1;
                        int end_inside_index = -1;
                        for(int p_i = 0;p_i<line.total_line.size();p_i++){
                            EFMPoint p =line.total_line[p_i];
                            if(CommonTool::DiscretePointsMath::GetInstance()->InPolygon(p, polygon)== true){
                                if(start_inside_index<0){
                                    start_inside_index = p_i;
                                }else{
                                    end_inside_index = p_i;
                                }
                                if(p_i>0 && p_i<line.total_line.size()){
                                    accum_s+=sqrt(pow(p.x - line.total_line[p_i-1].x, 2)+ pow(p.y - line.total_line[p_i-1].y, 2));
                                }
                                
                            }else{
                                if(start_inside_index>=0){                                    
                                    break;
                                }
                                
                            }
                        }
#ifdef RELOCATE_INNER
                            std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,accum_s:"<<accum_s<<",start_inside_index:"<<start_inside_index<<
                            ",end_inside_index:"<<end_inside_index<<std::endl;
#endif
                        if(accum_s >= 20 && start_inside_index>=0 && end_inside_index>= 0 && end_inside_index> start_inside_index){
#ifdef RELOCATE_INNER
                            std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,accum_s is enough"<<std::endl;
#endif
                            //一长段落在区间内，那么考虑是否替换原来的分配
                            //10米一个点，计算和左右线的平均l
                            double average_l_to_left = 0;
                            double average_l_to_right = 0;
                            double start_point_l_to_left = 0, end_point_l_to_left = 0;
                            double start_point_l_to_right = 0, end_point_l_to_right = 0;
                            int count = 0;
                            int p_i = start_inside_index;
                            for(;p_i <= end_inside_index && p_i < line.total_line.size() && p_i >=0; p_i+=4){
                                PointSLd to_left_l;
                                CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_side_line, line.total_line[p_i], to_left_l);
                                average_l_to_left += to_left_l.l;
                                PointSLd to_right_l;
                                CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_side_line, line.total_line[p_i], to_right_l);
                                average_l_to_right += to_right_l.l;
                                if(p_i == start_inside_index){
                                    start_point_l_to_left = to_left_l.l;
                                    start_point_l_to_right = to_right_l.l;
                                }
                                if(p_i == end_inside_index){
                                    end_point_l_to_left = to_left_l.l;
                                    end_point_l_to_right = to_right_l.l;
                                }
                                count ++;
                            }
                            //最后一个点需要纳入考虑
                            if(p_i>end_inside_index && p_i!=(end_inside_index+4)){
                                PointSLd to_left_l;
                                CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_side_line, line.total_line[end_inside_index], to_left_l);
                                average_l_to_left += to_left_l.l;
                                PointSLd to_right_l;
                                CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_side_line, line.total_line[end_inside_index], to_right_l);
                                average_l_to_right += to_right_l.l;

                                end_point_l_to_left = to_left_l.l;
                                end_point_l_to_right = to_right_l.l;
                                count ++;                                
                            }
                            if(count >0){
                                average_l_to_left = average_l_to_left/count;
                                average_l_to_right = average_l_to_right/count;
                                int left_side_line_location_type = left_line.base_line_ptr->lane_location_type;
                                int right_side_line_location_type = right_line.base_line_ptr->lane_location_type;
#ifdef RELOCATE_INNER
                                std::cout << __FILE__ << "," << __LINE__ << "," << " average_l_to_right: " <<average_l_to_right<<" ,average_l_to_left: "<<average_l_to_left<<std::endl;
#endif
                                //离一侧过近，不考虑，离路沿近除外
                                if((fabs(average_l_to_right)<0.5 && (left_side_line_location_type != 21 && left_side_line_location_type != 22 && left_side_line_location_type != 24))
                                   || (fabs(average_l_to_left)<0.5&& (right_side_line_location_type != 21 && right_side_line_location_type != 22 && right_side_line_location_type != 24))){
                                    continue;
                                }
                                //判断倾斜方向，判断是否可能交叉
                                //如果构成的车道宽度不够，那么还需要考虑是否是交叉,目前只考虑front
#ifdef RELOCATE_INNER
                                std::cout << __FILE__ << "," << __LINE__ << "," << " start_point_l_to_left: " <<start_point_l_to_left<<" ,end_point_l_to_left: "<<end_point_l_to_left<<std::endl;
                                std::cout << __FILE__ << "," << __LINE__ << "," << " start_point_l_to_right: " <<start_point_l_to_right<<" ,end_point_l_to_right: "<<end_point_l_to_right<<std::endl;
#endif
                                int left_line_location_type = left_line.base_line_ptr->lane_location_type;
                                int right_line_location_type = right_line.base_line_ptr->lane_location_type;
                                if(start_inside_index+1 < line.total_line.size() && start_inside_index>= 0 ){
                                    // double dir_length = sqrt(pow(line.total_line[start_inside_index].x - line.total_line[start_inside_index+1].x,2)+pow(line.total_line[start_inside_index].y - line.total_line[start_inside_index+1].y,2));
                                    // EFMPoint line_front_p_extern_dir((line.total_line[start_inside_index].x - line.total_line[start_inside_index+1].x)/dir_length,  (line.total_line[start_inside_index].y - line.total_line[start_inside_index+1].y)/dir_length);
                                    //判断inner线是朝左线还是朝右线倾斜
                                    // double k_left = fabs(start_point_l_to_left)/fabs(end_point_l_to_left);
                                    // double k_right = fabs(start_point_l_to_right)/fabs(end_point_l_to_right);
#ifdef RELOCATE_INNER
                                // std::cout << __FILE__ << "," << __LINE__ << "," << " k_left: " <<k_left<<" ,k_right:"<<k_right<<std::endl;
#endif
                                    if((((fabs(start_point_l_to_left)+0.8)<fabs(end_point_l_to_left)) && (fabs(start_point_l_to_left)<2.5))){//线向左倾斜,且交叉
#ifdef RELOCATE_INNER
                                        std::cout << __FILE__ << "," << __LINE__ << "," << "inner line split from left; "<<std::endl;
#endif
                                        //取左线和inner线做判断，是否可行驶
                                        InnerLineSubJudge(left_line, line, start_inside_index, end_inside_index, 1, average_l_to_left, average_l_to_right, left_side_line_location_type, right_side_line_location_type, bev_lines, 1, is_first_find, i, inner_line_index_vec,
                                                            choosed_inner_line_index, inner_line_replace_res, replace_res);
#ifdef RELOCATE_INNER
                                                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_res "<<replace_res<<" ,choosed_inner_line_index:"<<choosed_inner_line_index<<std::endl;   
                                                if(inner_line_replace_res.base_line_ptr != nullptr){
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.base_line_ptr->id "<<inner_line_replace_res.base_line_ptr->id<<std::endl;
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.back_connect_line_set_ptr.size() "<<inner_line_replace_res.back_connect_line_set_ptr.size()<<" ,back_connrct_id:";
                                                    for(auto iter:inner_line_replace_res.back_connect_line_set_ptr){
                                                        std::cout<<" ,"<<iter.first->id;
                                                    }
                                                    std::cout<<std::endl;
                                                } 
#endif
                                    }else if(((fabs(start_point_l_to_left)-0.8)>fabs(end_point_l_to_left)) && (fabs(end_point_l_to_left)<2.5)){
#ifdef RELOCATE_INNER
                                        std::cout << __FILE__ << "," << __LINE__ << "," << "inner line merge to left; "<<std::endl;
#endif
                                        //取左线和inner线做判断，是否可行驶
                                        InnerLineSubJudge(left_line, line, start_inside_index, end_inside_index, 1, average_l_to_left, average_l_to_right, left_side_line_location_type, right_side_line_location_type, bev_lines, 2, is_first_find, i, inner_line_index_vec,
                                                            choosed_inner_line_index, inner_line_replace_res, replace_res);
#ifdef RELOCATE_INNER
                                                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_res "<<replace_res<<" ,choosed_inner_line_index:"<<choosed_inner_line_index<<std::endl;   
                                                if(inner_line_replace_res.base_line_ptr != nullptr){
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.base_line_ptr->id "<<inner_line_replace_res.base_line_ptr->id<<std::endl;
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.back_connect_line_set_ptr.size() "<<inner_line_replace_res.back_connect_line_set_ptr.size()<<" ,back_connrct_id:";
                                                    for(auto iter:inner_line_replace_res.back_connect_line_set_ptr){
                                                        std::cout<<" ,"<<iter.first->id;
                                                    }
                                                    std::cout<<std::endl;
                                                } 
#endif
                                    }else if((((fabs(start_point_l_to_right)+0.8)<fabs(end_point_l_to_right))&& (fabs(start_point_l_to_right)<2.5))){
#ifdef RELOCATE_INNER
                                        std::cout << __FILE__ << "," << __LINE__ << "," << "inner line split from right ; "<<std::endl;
#endif
                                        InnerLineSubJudge(right_line, line, start_inside_index, end_inside_index,2, average_l_to_left, average_l_to_right, left_side_line_location_type, right_side_line_location_type, bev_lines, 1, is_first_find, i, inner_line_index_vec,
                                                            choosed_inner_line_index, inner_line_replace_res, replace_res);
#ifdef RELOCATE_INNER
                                                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_res "<<replace_res<<" ,choosed_inner_line_index:"<<choosed_inner_line_index<<std::endl;   
                                                if(inner_line_replace_res.base_line_ptr != nullptr){
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.base_line_ptr->id "<<inner_line_replace_res.base_line_ptr->id<<std::endl;
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.back_connect_line_set_ptr.size() "<<inner_line_replace_res.back_connect_line_set_ptr.size()<<" ,back_connrct_id:";
                                                    for(auto iter:inner_line_replace_res.back_connect_line_set_ptr){
                                                        std::cout<<" ,"<<iter.first->id;
                                                    }
                                                    std::cout<<std::endl;
                                                } 
#endif
                                    }else if((((fabs(start_point_l_to_right)-0.8)>fabs(end_point_l_to_right))&& (fabs(end_point_l_to_right)<2.5))){
#ifdef RELOCATE_INNER
                                        std::cout << __FILE__ << "," << __LINE__ << "," << "inner line merge to right"
                                             <<" ,((fabs(start_point_l_to_right)-0.6)>fabs(end_point_l_to_right)):"<<(int)((fabs(start_point_l_to_right)-0.6)>fabs(end_point_l_to_right))
                                             <<" ,(fabs(end_point_l_to_right)<2.5):"<<(int)(fabs(end_point_l_to_right)<2.5)<<std::endl;
#endif
                                        InnerLineSubJudge(right_line, line, start_inside_index, end_inside_index,2, average_l_to_left, average_l_to_right, left_side_line_location_type, right_side_line_location_type, bev_lines, 2, is_first_find, i, inner_line_index_vec,
                                                            choosed_inner_line_index, inner_line_replace_res, replace_res);
#ifdef RELOCATE_INNER
                                                std::cout << __FILE__ << "," << __LINE__ << "," << "replace_res "<<replace_res<<" ,choosed_inner_line_index:"<<choosed_inner_line_index<<std::endl;   
                                                if(inner_line_replace_res.base_line_ptr != nullptr){
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.base_line_ptr->id "<<inner_line_replace_res.base_line_ptr->id<<std::endl;
                                                    std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.back_connect_line_set_ptr.size() "<<inner_line_replace_res.back_connect_line_set_ptr.size()<<" ,back_connrct_id:";
                                                    for(auto iter:inner_line_replace_res.back_connect_line_set_ptr){
                                                        std::cout<<" ,"<<iter.first->id;
                                                    }
                                                    std::cout<<std::endl;
                                                } 
#endif
                                    }else{
                                        if(fabs(average_l_to_left)>2 && fabs(average_l_to_right)>2 && average_l_to_left*average_l_to_right<0){
#ifdef RELOCATE_INNER
                                            std::cout << __FILE__ << "," << __LINE__ << "," << " bev_line.id: " <<line.id<<" ,board is enough, is_first_find:"<<(int)is_first_find<<std::endl;
#endif
                                            if(is_first_find == false){
                                                //简单判断，x离自车近的, 保存的线和新来的线
                                                auto stored_line = bev_lines[choosed_inner_line_index].total_line;
                                                auto new_come_line = bev_lines[i].total_line;
                                                inner_line_index_vec.push_back(i);
                                                if(stored_line.front().x>=0 && new_come_line.front().x>=0){
                                                    if(stored_line.front().x>new_come_line.front().x){
                                                        choosed_inner_line_index=i;
                                                    }
                                                }else if(stored_line.front().x>=0 && new_come_line.front().x<=0 && new_come_line.back().x>0){
                                                    choosed_inner_line_index=i;
                                                }else if(stored_line.back().x<=0 && new_come_line.front().x>=0){
                                                    choosed_inner_line_index=i;
                                                }else if(stored_line.back().x<=0 && new_come_line.front().x<=0 && new_come_line.back().x>0){
                                                    choosed_inner_line_index=i;
                                                }
                                            }else{
                                                choosed_inner_line_index = i;
                                                inner_line_index_vec.push_back(i);
                                                is_first_find = false;
                                            }                                    
                                            bev_lines[i].is_inner_line_f = true;
                                            bev_lines[i].inner_point_index_pair = std::make_pair(start_inside_index, end_inside_index);
                                        }                                        
                                    }
    
                                    
                                }

                            }
                        }
                    }

                }
    }
#ifdef RELOCATE_INNER
    std::cout << __FILE__ << "," << __LINE__ << "," << " choosed_inner_line_index: " <<choosed_inner_line_index<< " ,replace_res:"<<replace_res<<std::endl;
    if(inner_line_replace_res.base_line_ptr != nullptr){
        std::cout << __FILE__ << "," << __LINE__ << "," << "inner_line_replace_res.base_line_ptr->id "<<inner_line_replace_res.base_line_ptr->id<<std::endl;
    }
#endif
    return true;
}

bool LaneModel::CalTwoLineDist(const EFMRefLinePoints& base_line, const EFMRefLinePoints& cal_line, double& cal_to_base_l, int gap_index){
    //gap_index， 距离多少个点计算一次
    cal_to_base_l = 0;
    int cal_p_inside_count = 0;
    double cal_to_base_averg_l = 0;
    PointSLd sl_cal_to_base;
    bool cal_is_inside_base=false;//
    //计算cal_line距离 baseline的距离
    int p_index = 0;
    for(; p_index<cal_line.size();p_index+=gap_index){
        EFMPoint cal_p = cal_line[p_index];
        cal_is_inside_base = false,
        CommonTool::DiscretePointsMath::GetInstance()-> CalPointSLBodyCoordinate(base_line,cal_p, sl_cal_to_base, cal_is_inside_base);
        if(cal_is_inside_base== true){
            cal_to_base_averg_l += sl_cal_to_base.l;
            cal_p_inside_count ++;
        }        
    }
    if(cal_p_inside_count>0){
        cal_to_base_averg_l = cal_to_base_averg_l/cal_p_inside_count;
    }else{
        return false;
    }
    cal_to_base_l = cal_to_base_averg_l;
    return true;
}

bool LaneModel::CalTwoLineDist(const EFMRefLinePoints& base_line, const EFMRefLinePoints& cal_line, double& cal_to_base_l, int gap_index, int& overlap_start_index, int& overlap_end_index){
    //gap_index， 距离多少个点计算一次
    cal_to_base_l = 0;
    int cal_p_inside_count = 0;
    double cal_to_base_averg_l = 0;
    PointSLd sl_cal_to_base;
    bool cal_is_inside_base=false;//
    //计算cal_line距离 baseline的距离
    int p_index = 0;
    overlap_start_index = -1;
    overlap_end_index = -1;
    double last_point_l =0;
    for(; p_index<cal_line.size();p_index+=gap_index){
        EFMPoint cal_p = cal_line[p_index];
        cal_is_inside_base = false,
        CommonTool::DiscretePointsMath::GetInstance()-> CalPointSLBodyCoordinate(base_line,cal_p, sl_cal_to_base, cal_is_inside_base);
        if(cal_is_inside_base== true){
            if(overlap_start_index>=0 && last_point_l*sl_cal_to_base.l<0){//l异号
                break;
            }
            cal_to_base_averg_l += sl_cal_to_base.l;
            cal_p_inside_count ++;
            if(overlap_start_index <0){
                overlap_start_index = p_index;
            }
            overlap_end_index = p_index;
            last_point_l = sl_cal_to_base.l;
        }        
    }
    if(cal_p_inside_count>0){
        cal_to_base_averg_l = cal_to_base_averg_l/cal_p_inside_count;
    }else{
        return false;
    }
    cal_to_base_l = cal_to_base_averg_l;
    return true;
}
// bool LaneModel::GenerateSideCurb(const std::vector<BevLineInnerConnect>& connect_lines, BevLaneElementGroupSet& bev_lane_group_set){
//     bool find_right_curb = false, find_left_curb = false;
//     BevLaneElement right_curb, left_curb;
//     BevLaneElementGroup right_curb_group{}, left_curb_group{};
//     for(auto& line:connect_lines){
//         if(line.base_line_ptr != nullptr && line.base_line_ptr->sl_to_ego.l>0 && find_right_curb == false && 
//             (line.base_line_ptr->lane_location_type == 21 || line.base_line_ptr->lane_location_type == 22 || line.base_line_ptr->lane_location_type == 24)){
//             MakeSideLine(line, right_curb.right_type_first, right_curb.right_type_sec, right_curb.right_line_points,right_curb.right_line_split_index,right_curb.right_line_merge_index);
//             //debug
//             right_curb.right_line_base_id = line.base_line_ptr->id;
//             if(line.front_connect_line_set_ptr.size()>0){
//                 right_curb.right_front_connect_id = line.front_connect_line_set_ptr.begin()->first->id;
//             }
//             if(line.back_connect_line_set_ptr.size()>0){
//                 right_curb.right_back_connect_id = line.back_connect_line_set_ptr.begin()->first->id;
//             }
//             find_right_curb = true;
//             right_curb_group.push_back(right_curb);
//             bev_lane_group_set.insert(bev_lane_group_set.begin(),std::make_pair(100,right_curb_group));
//         }

//         if(line.base_line_ptr != nullptr && line.base_line_ptr->sl_to_ego.l<0 && find_left_curb == false && 
//             (line.base_line_ptr->lane_location_type == 21 || line.base_line_ptr->lane_location_type == 22 || line.base_line_ptr->lane_location_type == 24)){
//             MakeSideLine(line, left_curb.left_type_first, left_curb.left_type_sec, left_curb.left_line_points, left_curb.left_line_split_index,left_curb.left_line_merge_index);
//             //debug
//             left_curb.left_line_base_id = line.base_line_ptr->id;
//             if(line.front_connect_line_set_ptr.size()>0){
//                 left_curb.right_front_connect_id = line.front_connect_line_set_ptr.begin()->first->id;
//             }
//             if(line.back_connect_line_set_ptr.size()>0){
//                 left_curb.right_back_connect_id = line.back_connect_line_set_ptr.begin()->first->id;
//             }
//             find_left_curb = true;
//             left_curb_group.push_back(left_curb);
//             bev_lane_group_set.push_back(std::make_pair(99,left_curb_group));
//         }
//     }
// #ifdef LM_LANEGROUP
//     std::cout << __FILE__ << "," << __LINE__ << "," << " right_curb_group.size():"<<right_curb_group.size()<<std::endl;
//     for(int i = 0; i< right_curb_group.size(); i++){
//         std::cout<< "left_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<right_curb_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< right_curb_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<right_curb_group[i].left_front_connect_id
//                                                 <<" right_line_base_id: "<<right_curb_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< right_curb_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<right_curb_group[i].right_front_connect_id
//                                                  <<" ,left_line_size"<<right_curb_group[i].left_line_points.size()<<" ,right_line_size"<<right_curb_group[i].right_line_points.size()<<std::endl;

//         {
//         std::stringstream ss, ss1;
//         ss<<"left_line_x = [";
//         ss1<<"left_line_y = [";
//         for(auto p: right_curb_group[i].left_line_points){
//             ss<< p.x<<" ";
//         }
//         for(auto p: right_curb_group[i].left_line_points){
//             ss1<< p.y<<" ";
//         }
//         ss<<"]"<<std::endl;
//         ss1<<"]"<<std::endl; 
//         std::cout<<ss.str();   
//         std::cout<<ss1.str();    
//         }
//         {
//         std::stringstream ss, ss1;
//         ss<<"right_line_x = [";
//         ss1<<"right_line_y = [";
//         for(auto p: right_curb_group[i].right_line_points){
//             ss<< p.x<<" ";
//         }
//         for(auto p: right_curb_group[i].right_line_points){
//             ss1<< p.y<<" ";
//         }
//         ss<<"]"<<std::endl;
//         ss1<<"]"<<std::endl; 
//         std::cout<<ss.str();   
//         std::cout<<ss1.str();    
//         }
//     }

//     std::cout << __FILE__ << "," << __LINE__ << "," << " left_curb_group.size():"<<left_curb_group.size()<<std::endl;
//     for(int i = 0; i< left_curb_group.size(); i++){
//         std::cout<< "left_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<left_curb_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< left_curb_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<left_curb_group[i].left_front_connect_id
//                                                 <<" right_line_base_id: "<<left_curb_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< left_curb_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<left_curb_group[i].right_front_connect_id
//                                                  <<" ,left_line_size"<<left_curb_group[i].left_line_points.size()<<" ,right_line_size"<<left_curb_group[i].right_line_points.size()<<std::endl;

//         {
//         std::stringstream ss, ss1;
//         ss<<"left_line_x = [";
//         ss1<<"left_line_y = [";
//         for(auto p: left_curb_group[i].left_line_points){
//             ss<< p.x<<" ";
//         }
//         for(auto p: left_curb_group[i].left_line_points){
//             ss1<< p.y<<" ";
//         }
//         ss<<"]"<<std::endl;
//         ss1<<"]"<<std::endl; 
//         std::cout<<ss.str();   
//         std::cout<<ss1.str();    
//         }
//         {
//         std::stringstream ss, ss1;
//         ss<<"right_line_x = [";
//         ss1<<"right_line_y = [";
//         for(auto p: left_curb_group[i].right_line_points){
//             ss<< p.x<<" ";
//         }
//         for(auto p: left_curb_group[i].right_line_points){
//             ss1<< p.y<<" ";
//         }
//         ss<<"]"<<std::endl;
//         ss1<<"]"<<std::endl; 
//         std::cout<<ss.str();   
//         std::cout<<ss1.str();    
//         }
//     }
// #endif
//     return true;
// }

bool LaneModel::GetEgoLaneIdx(const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, BevLineInnerS* ego_left,BevLineInnerS* ego_right,SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info,const BevLaneElementGroup* ego_bev_lane_group, const StoredInfo& last_cycle_info){
#ifdef LOC    
    std::cout << __FILE__ << "," << __LINE__ << "," << " lat_:"<<std::to_string(lane_loc.lat_) <<" ,lon_:"<<std::to_string(lane_loc.lon_)<<" ,link_id:"<<lane_loc.link_id_<<" ,loc offset:"<< lane_loc.lane_offset_<<std::endl;
#endif
    loc_res_info.loc_judge_dir_ = 0;
    bool use_left_curv = false;
    bool use_right_curv = false;
    for(auto& iter:sd_links_for_loc_only_from_left_){
        if(lane_loc.link_id_ == iter){
            use_left_curv = true;
        }
    }
    if(!use_left_curv){
        for(auto& iter:sd_links_for_loc_only_from_right_){
            if(lane_loc.link_id_ == iter){
                use_right_curv = true;
            }
        }
    }
    //connect_lines是从右到左的顺序, 找到离自车最近的右路沿和左路沿
    bool find_right_curb = false, find_left_curb = false;
    int right_curb_index = -1, left_curb_index = -1;
    int ego_left_index = -1, ego_right_index = -1;
    for(int i = 0; i<bev_lines_in_side_ego.size();i++){
        auto& line = bev_lines_in_side_ego[i];
        if(line!= nullptr && line->sl_to_ego.l>0 && 
            (line->lane_location_type == 21 || line->lane_location_type == 22 || line->lane_location_type == 24)){
            find_right_curb = true;
            right_curb_index = i;
        }

        if(line!= nullptr && line->sl_to_ego.l<0 && find_left_curb == false && 
            (line->lane_location_type == 21 || line->lane_location_type == 22 || line->lane_location_type == 24)){
            find_left_curb = true;
            left_curb_index = i;
        }
        if(ego_right!= nullptr &&ego_right->id == line->id){
            ego_right_index = i;
        }
        if(ego_left != nullptr && ego_left->id == line->id){
            ego_left_index = i;

        }
    }
#ifdef LOC    
    std::cout << __FILE__ << "," << __LINE__ << "," << " left_curb_index:"<<left_curb_index <<" ,right_curb_index:"<<right_curb_index<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " ego_left_index:"<<ego_left_index <<" ,ego_right_index:"<<ego_right_index<<std::endl;
#endif
    //case1 ,左右都有路沿，看自车离哪边近，用哪边的路沿，看自车到路沿有几根车道
    int lane_count = 0;
    uint64_t lane_id = 0;
    if(left_curb_index>=0 || right_curb_index>=0){
        if(ele_group.size() == 1){//sd只有一根，那么定位到这根车道
            if(GetLaneIdFromSD(ele_group, 1, loc_res_info.loc_lane_num_, lane_id) == true){
                lane_loc.lane_id_ = lane_id;
                
                return true;
            }            
        }
    }
    if(left_curb_index>=0 && right_curb_index>=0 && left_curb_index< bev_lines_in_side_ego.size() && right_curb_index<bev_lines_in_side_ego.size()){
        // 左右路沿都有，判断哪边的路沿质量高
        //先判断自车前方的长度，
#ifdef LOC  
    std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines_in_side_ego[right_curb_index]->FirstEndPoint:"<<bev_lines_in_side_ego[right_curb_index]->FirstEndPoint<<std::endl; 
    std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines_in_side_ego[left_curb_index]->FirstEndPoint:"<<bev_lines_in_side_ego[left_curb_index]->FirstEndPoint<<std::endl; 
    std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines_in_side_ego[right_curb_index]->SecEndPoint:"<<bev_lines_in_side_ego[right_curb_index]->SecEndPoint<<std::endl; 
    std::cout << __FILE__ << "," << __LINE__ << "," << " bev_lines_in_side_ego[left_curb_index]->SecEndPoint:"<<bev_lines_in_side_ego[left_curb_index]->SecEndPoint<<std::endl; 

#endif
        double left_curb_end_x = 0;
        double right_curb_end_x = 0;
        if(bev_lines_in_side_ego[right_curb_index]->sec_valid == true){
            right_curb_end_x = bev_lines_in_side_ego[right_curb_index]->SecEndPoint;
        }else{
            right_curb_end_x = bev_lines_in_side_ego[right_curb_index]->FirstEndPoint;
        }
        if(bev_lines_in_side_ego[left_curb_index]->sec_valid == true){
            left_curb_end_x = bev_lines_in_side_ego[left_curb_index]->SecEndPoint;
        }else{
            left_curb_end_x = bev_lines_in_side_ego[left_curb_index]->FirstEndPoint;
        }   

        if(use_left_curv == true){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " log use left:"<<std::endl; 
#endif            
            if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group,loc_res_info)==true){
                return true;
            }            
        }else if(use_right_curv == true){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " log use right:"<<std::endl; 
#endif            
            if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group,loc_res_info)==true){
                return true;
            }
        }

        if(right_curb_end_x > left_curb_end_x && fabs(right_curb_end_x - left_curb_end_x)>50){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " you侧 长:"<<std::endl; 
#endif
            if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
                return true;
            }else{
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 靠you侧没找到 ，左侧继续:"<<std::endl; 
#endif
                if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group,loc_res_info)==true){
                    return true;
                }                
            }
        }else if(left_curb_end_x > right_curb_end_x && fabs(left_curb_end_x - right_curb_end_x)>50){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 左侧 长:"<<std::endl; 
#endif
            if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group,loc_res_info)==true){
                return true;
            }else{
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 靠zuo侧没找到 ，you侧继续:"<<std::endl; 
#endif
                if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group,loc_res_info)==true){
                    return true;
                }                
            }
        }else if(fabs(bev_lines_in_side_ego[left_curb_index]->sl_to_ego.l) < fabs(bev_lines_in_side_ego[right_curb_index]->sl_to_ego.l)){
            //左侧近
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 左侧近:"<<std::endl; 
#endif
            if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
                return true;
            }else{
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 靠zuo侧没找到 ，you侧继续:"<<std::endl; 
#endif
                if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
                    return true;
                }                
            }
        }else{
            //右侧近
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " you侧近:"<<std::endl; 
#endif
            if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
                return true;
            }else{
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 靠you侧没找到 ，左侧继续:"<<std::endl; 
#endif
                if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
                    return true;
                }                
            }
        }

    //case2, 只有左侧路沿
    }else if(left_curb_index>=0){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << "只有左侧路沿:"<<std::endl;
#endif
        if(GetLaneIdViaLeftSide(ego_left_index, left_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
            return true;
        }
    //case3, 只有右侧路沿
    }else if(right_curb_index>=0){
#ifdef LOC  
            std::cout << __FILE__ << "," << __LINE__ << "," << " 只有右侧路沿:"<<std::endl; 
#endif
        if(GetLaneIdViaRightSide(ego_right_index, right_curb_index, bev_lines_in_side_ego, lane_loc, ele_group, loc_res_info)==true){
            return true;
        }
        
    }else{
    //case4, 左右路沿都没有
#ifdef LOC  
    std::cout << __FILE__ << "," << __LINE__ << "," << "case4 lane_num:"<<(int)last_cycle_info.lane_num<<std::endl;
#endif
    // ZTEXT("BEV_LINE", "lane_num: ", 16, 28, "lane_num: {}", (lane_count+1));
        //当前自车group左右
        int lane_num = 0;
        EgoLaneIdWithoutCurb(ego_bev_lane_group,last_cycle_info, lane_loc.lane_ids_.size(),lane_num);
        if(GetLaneIdFromSD(ele_group, lane_num, loc_res_info.loc_lane_num_,lane_id) == true){
            lane_loc.lane_id_ = lane_id;
            
            return true;
        }
    }

    return false;

}

bool LaneModel::GetLaneIdViaRightSide(int ego_right_index, int right_curb_index, const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info){
#ifdef LOC 
            std::cout << __FILE__ << "," << __LINE__ << "," << " GetLaneIdViaRightSide start"<<std::endl;
#endif   
    loc_res_info.loc_lane_num_ = 2;
    int lane_count = 0;
    uint64_t lane_id = 0;
    if(ego_right_index>=0 && ego_right_index>= right_curb_index){
        for(int i= right_curb_index; i<=(ego_right_index-1) && (i+1)<bev_lines_in_side_ego.size() && i>=0; i++){
            double l = 0;
            CalTwoLineDist(bev_lines_in_side_ego[i+1]->total_line, bev_lines_in_side_ego[i]->total_line, l, 2);
#ifdef LOC 
            std::cout << __FILE__ << "," << __LINE__ << "," << " i:"<<i <<" ,i+1:"<<i+1<<", l:"<<l<<std::endl;
#endif
            if(fabs(l)>1.8){
                lane_count++;
            }
        }
#ifdef LOC  
        std::cout << __FILE__ << "," << __LINE__ << "," << "case1you lane_count:"<<lane_count<<std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << "case1 lane_num:"<<(lane_count+1)<<std::endl;
#endif
        int lane_num = lane_count+1;
        if(lane_num>ele_group.size()){
            lane_num = ele_group.size();
        }
            // ZTEXT("BEV_LINE", "lane_num: ", 16, 28, "lane_num: {}", (lane_count+1));
        if(GetLaneIdFromSD(ele_group, lane_num, loc_res_info.loc_lane_num_,lane_id) == true){
            lane_loc.lane_id_ = lane_id;
            // last_cycle_info_.lane_num = static_cast<uint8_t>(lane_count+1);
            return true;
        }
    }
    return false;
}

bool LaneModel::GetLaneIdViaLeftSide(int ego_left_index, int left_curb_index, const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info){
#ifdef LOC 
            std::cout << __FILE__ << "," << __LINE__ << "," << " GetLaneIdViaLeftSide start"<<","<<std::endl;
#endif  
    loc_res_info.loc_lane_num_ = 1;  
    int lane_count = 0;
    uint64_t lane_id = 0;
    if(ego_left_index>=0 && ego_left_index<= left_curb_index){
        //  std::cout << __FILE__ << "," << __LINE__ << "," << " 1111111111"<<std::endl;
        for(int i= left_curb_index -1; i>=ego_left_index && i>=0 && (i+1)< bev_lines_in_side_ego.size(); i--){
            double l = 0;
            CalTwoLineDist(bev_lines_in_side_ego[i+1]->total_line, bev_lines_in_side_ego[i]->total_line, l, 2);
#ifdef LOC 
            std::cout << __FILE__ << "," << __LINE__ << "," << " i:"<<i <<" ,i+1:"<<i+1<<", l:"<<l<<std::endl;
#endif
            if(fabs(l)>1.8){
                lane_count++;
            }
        }
#ifdef LOC  
        std::cout << __FILE__ << "," << __LINE__ << "," << "case1zuo lane_count:"<<lane_count<<std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << "case1 lane_num:"<<(lane_count+1)<<std::endl;
#endif
        int lane_num = ele_group.size() - lane_count;
        if(lane_num <= 0){
            lane_num = 1;
        }
            // ZTEXT("BEV_LINE", "lane_num: ", 16, 28, "lane_num: {}", ele_group.size()-lane_count);
        if(GetLaneIdFromSD(ele_group, lane_num, loc_res_info.loc_lane_num_, lane_id) == true){
            lane_loc.lane_id_ = lane_id;
            // last_cycle_info_.lane_num = static_cast<uint8_t>(lane_count+1);
            return true;
        }
    }
    return false;

}

bool LaneModel::GetLineType(const BevLineInnerConnect& side_line, double target_x, int search_type, int& line_type){
    if(side_line.base_line_ptr == nullptr){
        return false;
    }
    //先找有没有连接的线
    //search_type ==1找大于target_x的linetype, 否则找小于target_x的linetype
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "target_x "<<target_x<<std::endl;
#endif
    if(search_type == 1){
        if(side_line.back_connect_line_set_ptr.size()>0){
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "side_line.back_connect_id "<<side_line.back_connect_line_set_ptr.begin()->first->id<<" ,side_line.back_connect_.front().x:"<<side_line.back_connect_line_set_ptr.begin()->first->total_line.front().x<<std::endl;
#endif
            if(side_line.back_connect_line_set_ptr.begin()->first->total_line.size()>0 && side_line.back_connect_line_set_ptr.begin()->first->total_line.front().x <(target_x+1)){//用连接的线的type
                if( side_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point == side_line.back_connect_line_set_ptr.begin()->first->line_type){
                    line_type =side_line.back_connect_line_set_ptr.begin()->first->line_type;
                }else{
                    if(static_cast<double>(side_line.back_connect_line_set_ptr.begin()->first->typ_chg_point)>(target_x +1.0)){
                        line_type =side_line.back_connect_line_set_ptr.begin()->first->line_type;
                    }else{
                        line_type =side_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point;
                    }                                                                
                }
            }else{
                if(side_line.base_line_ptr->typ_aft_chg_point == side_line.base_line_ptr->line_type){
                    line_type =side_line.base_line_ptr->line_type;
                }else{
                    if(static_cast<double>(side_line.base_line_ptr->typ_chg_point)>(target_x +1.0)){
                        line_type =side_line.base_line_ptr->line_type;
                    }else{
                        line_type =side_line.base_line_ptr->typ_aft_chg_point;
                    }
                }                
            }
        }else{
            if(side_line.base_line_ptr->typ_aft_chg_point == side_line.base_line_ptr->line_type){
                line_type =side_line.base_line_ptr->line_type;
            }else{
                if(static_cast<double>(side_line.base_line_ptr->typ_chg_point)>(target_x +1.0)){
                    line_type =side_line.base_line_ptr->line_type;
                }else{
                    line_type =side_line.base_line_ptr->typ_aft_chg_point;
                }
            }
        }
       
    }else{//找小于target_x的最贴近target_x的 linetype
        if(side_line.base_line_ptr->total_line.size()>0 && side_line.base_line_ptr->total_line.back().x> target_x){
            if(side_line.base_line_ptr->typ_aft_chg_point == side_line.base_line_ptr->line_type){
                line_type =side_line.base_line_ptr->line_type;
            }else{
                if(static_cast<double>(side_line.base_line_ptr->typ_chg_point)>(target_x)){
                    line_type =side_line.base_line_ptr->line_type;
                }else{
                    line_type =side_line.base_line_ptr->typ_aft_chg_point;
                }
            }             
        }else if(side_line.back_connect_line_set_ptr.size()>0){
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "side_line.back_connect_id "<<side_line.back_connect_line_set_ptr.begin()->first->id<<" ,side_line.back_connect_.front().x:"<<side_line.back_connect_line_set_ptr.begin()->first->total_line.front().x<<std::endl;
#endif
            if(side_line.back_connect_line_set_ptr.begin()->first->total_line.size()>0 && side_line.back_connect_line_set_ptr.begin()->first->total_line.front().x <target_x){//用连接的线的type
                if( side_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point == side_line.back_connect_line_set_ptr.begin()->first->line_type){
                    line_type =side_line.back_connect_line_set_ptr.begin()->first->line_type;
                }else{
                    if(static_cast<double>(side_line.back_connect_line_set_ptr.begin()->first->typ_chg_point)>(target_x +1.0)){
                        line_type =side_line.back_connect_line_set_ptr.begin()->first->line_type;
                    }else{
                        line_type =side_line.back_connect_line_set_ptr.begin()->first->typ_aft_chg_point;
                    }                                                                
                }
            }
        }else{
            if(side_line.base_line_ptr->typ_aft_chg_point == side_line.base_line_ptr->line_type){
                line_type =side_line.base_line_ptr->line_type;
            }else{
                if(static_cast<double>(side_line.base_line_ptr->typ_chg_point)>(target_x)){
                    line_type =side_line.base_line_ptr->line_type;
                }else{
                    line_type =side_line.base_line_ptr->typ_aft_chg_point;
                }
            }             
        }
    }

    return true;
}

bool LaneModel::InnerLineSubJudge(BevLineInnerConnect& side_line, BevLineInner& inner_line, int inner_line_start_inside_index, int inner_line_end_inside_index, int side_line_dir, double inner_to_left_l, double inner_to_right_l, int left_side_line_location_type, int right_side_line_location_type,
                                    std::vector<BevLineInnerS>& bev_lines, int inner_line_lean_dir, bool& is_first_find, int inner_line_index_in_bev, std::vector<int>& inner_line_index_vec,
                                    int &choosed_inner_line_index,BevLineInnerConnect& inner_line_replace_res, int& replace_res){
    //side_line_dir: 1-side_line是inner_line的左线， 2-side_line是inner_line的右线
    //inner_line_lean_dir: 1-innerline的头靠近side线，尾远离side线(split)， 2-innerlineinnerline的头远离side线，尾靠近side线(merge)
    if(inner_line_start_inside_index<0 || inner_line_start_inside_index>=inner_line.total_line.size()
       || inner_line_end_inside_index<0 || inner_line_end_inside_index>=inner_line.total_line.size()){
        return false;
    }
    //取side线和inner线做判断，是否可行驶
    EFMRefLinePointsSection line_section;
    int inner_line_tar_index = -1;
    if(inner_line_lean_dir ==1){//split,取start点，
        inner_line_tar_index = inner_line_start_inside_index;
    }else{
        inner_line_tar_index = inner_line_end_inside_index;
    }
    if(CommonTool::DiscretePointsMath::GetInstance()-> SaperateLineIntoTwoPart(side_line.line_point, inner_line.total_line[inner_line_tar_index], line_section) == true){
        // double length1_tmp=CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line_section[1]);
        // double length0_tmp=CommonTool::DiscretePointsMath::GetInstance()-> LineLength(line_section[0]);
        //判断两线间是否可以行驶
        if(line_section[1].size()>0 && line_section[0].size()>0){
            double line_type_x = line_section[1].front().x;//要找到大于这个距离的linetype，或找到小于这个x的第一个的lineType
            int line_le_attached_curb_side = 0;
            int line_ri_attached_curb_side = 0;
            int line_le_type = 1, line_ri_type = 1;
            int line_le_id = 0;
            int line_ri_id = 0;
            EFMRefLinePoints line_le_points{};
            EFMRefLinePoints line_ri_points{};
            // BevLineInnerS* base_ptr = side_line.base_line_ptr;
            if(side_line_dir == 1){//side_line是inner_line的左线
                line_le_id = side_line.base_line_ptr->id;
                GetLineType(side_line, line_type_x, inner_line_lean_dir,line_le_type);
                GetLineAttachCurbSide(side_line, line_type_x, line_le_attached_curb_side);
                // line_le_attached_curb_side = side_line.base_line_ptr->attached_curb_side;
                if(inner_line_lean_dir == 1){//如果ineer和side组成split, side取[1]
                    line_le_points = line_section[1];
                }else{
                    line_le_points = line_section[0];
                }
                line_ri_id = inner_line.id;
                line_ri_type = inner_line.line_type;
                line_ri_attached_curb_side = inner_line.attached_curb_side;
                line_ri_points = inner_line.total_line;
            }else{//2-side_line是inner_line的右线
                line_ri_id = side_line.base_line_ptr->id;
                GetLineType(side_line, line_type_x, inner_line_lean_dir,line_ri_type);
                GetLineAttachCurbSide(side_line, line_type_x, line_ri_attached_curb_side);
                // line_ri_attached_curb_side = side_line.base_line_ptr->attached_curb_side;
                if(inner_line_lean_dir == 1){//如果ineer和side组成split, side取[1]
                    line_ri_points = line_section[1];
                }else{
                    line_ri_points = line_section[0];
                }
                line_le_id = inner_line.id;
                line_le_type = inner_line.line_type;
                line_le_attached_curb_side = inner_line.attached_curb_side;   
                line_le_points = inner_line.total_line;             
            }
            
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "line_type_x "<<line_type_x<<std::endl;
#endif
            //line type, 边线所连接好的线， inner线是未连接的线
            //先找有没有连接的线
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "line_le_type "<<line_le_type<<" ,line_ri_type:"<<line_ri_type<<std::endl;
#endif
            bool is_driveable= false;
            if(side_line.base_line_ptr->lane_location_type == 21|| side_line.base_line_ptr->lane_location_type== 22|| side_line.base_line_ptr->lane_location_type ==24){
                is_driveable = false;
            }else{
                int dir_tmp = 0;
                //中间线在左右两线的dir要判断，对于任何的lane group,如果inner线和右线的平局宽度小，那么线是在右方
                if(fabs(inner_to_left_l)>fabs(inner_to_right_l)){
                    dir_tmp = 2;//you
                }else{
                    dir_tmp = 1;//zuo
                }
                is_driveable=IsDrivableSplitMerge(line_le_points, line_ri_points, line_le_type, line_ri_type, line_le_id, line_ri_id, bev_lines,dir_tmp, line_le_attached_curb_side,line_ri_attached_curb_side);
            }
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << "," << "line_le_id: "<<line_le_id<<" ,line_ri_id: "<<line_ri_id<<std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << "is_driveable "<<(int)is_driveable<<std::endl; 
#endif
            if(is_driveable == false){
                
                if(side_line_dir ==1 ){//如果side线和inner线区域不可行驶，side线是inner线左线， inner线要作为车道线取代左线
                    replace_res = 1;
                }else{//如果side线和inner线区域不可行驶，side线是inner线右线， inner线要作为车道线取代右线
                    replace_res = 2;
                }
                //替换的线需要考虑拼接，
                // is_first_find = false;
                if(inner_line_lean_dir == 1){//如果是split的inner_line_lean_dir==1，line_section[0]+inner,
                    inner_line_replace_res.base_line_ptr = &inner_line;
                    CutLineInSplitMerge(line_section[0],true);
                    inner_line_replace_res.line_point = line_section[0];
                    inner_line_replace_res.split_point_index = inner_line_replace_res.line_point.size()-1;
                    inner_line_replace_res.line_point.insert(inner_line_replace_res.line_point.end(),inner_line.total_line.begin()+inner_line_start_inside_index, 
                                                            inner_line.total_line.begin()+inner_line_end_inside_index); 
                    side_line.split_point_index = line_section[0].size()-1;                    

                }else{// 如果是merge的inner_line_lean_dir==2，inner+line_section[1]
                    inner_line_replace_res.base_line_ptr = &inner_line;
                    CutLineInSplitMerge(line_section[1],false);
                    
                    inner_line_replace_res.line_point.assign(inner_line.total_line.begin()+inner_line_start_inside_index, 
                                                            inner_line.total_line.begin()+inner_line_end_inside_index); 
                    inner_line_replace_res.merge_point_index = inner_line_replace_res.line_point.size()-1;
                    inner_line_replace_res.line_point.insert(inner_line_replace_res.line_point.end(), line_section[1].begin(), line_section[1].end());                                                         
                    side_line.merge_point_index = line_section[0].size(); 
                }

                // inner_line.is_inner_line_f = true;
                // inner_line.inner_point_index_pair = std::make_pair(inner_line_start_inside_index, inner_line_end_inside_index); 
            }else{
                //可行驶，不替换
                // if(side_line_dir ==1 ){//如果side线和inner线区域可行驶，side线是inner线左线， inner线要作为车道线取代右线
                //     replace_res = 2;
                // }else{//如果side线和inner线区域可行驶，side线是inner线右线， inner线要作为车道线取代左线
                //     replace_res = 1;
                // }
                // inner_line_replace_res.base_line_ptr = &inner_line;
                // inner_line_replace_res.line_point.assign(inner_line.total_line.begin()+inner_line_start_inside_index, 
                //                                          inner_line.total_line.begin()+inner_line_end_inside_index);                                                        
                //new add 20250925, 对可行驶的结果进行再判断，如果线很长，且构成的车道宽度很小，那么认为不是车道，舍弃

                double lane_length = 0;//计算inner line 长度
                double lane_broad = 0;//
                for(int i =inner_line_start_inside_index;i<inner_line.total_line.size() && i>=0 && i<=inner_line_end_inside_index;i++){
                    double dist = sqrt(pow(inner_line.total_line[i].x-inner_line.total_line[i-1].x,2)+ pow(inner_line.total_line[i].y-inner_line.total_line[i-1].y,2));
                    lane_length+= dist;
                }
                EFMRefLinePoints inner_points{};
                inner_points.assign(inner_line.total_line.begin()+inner_line_start_inside_index, inner_line.total_line.begin()+inner_line_end_inside_index);
                CalTwoLineDist(side_line.line_point, inner_points,  lane_broad, 1);
                if(lane_length>=50 && fabs(lane_broad)<1){
#ifdef RELOCATE_INNER
            std::cout << __FILE__ << "," << __LINE__ << ",舍弃可行驶" << "lane_length: "<<lane_length<<" ,lane_broad: "<<lane_broad<<std::endl;

#endif
                }else{
                    if(is_first_find == false){
                        //简单判断，x离自车近的, 保存的线和新来的线
                        auto stored_line = bev_lines[choosed_inner_line_index].total_line;
                        auto new_come_line = bev_lines[inner_line_index_in_bev].total_line;
                        inner_line_index_vec.push_back(inner_line_index_in_bev);
                        if(stored_line.front().x>=0 && new_come_line.front().x>=0){
                            if(stored_line.front().x>new_come_line.front().x){
                                choosed_inner_line_index=inner_line_index_in_bev;
                            }
                        }else if(stored_line.front().x>=0 && new_come_line.front().x<=0 && new_come_line.back().x>0){
                            choosed_inner_line_index=inner_line_index_in_bev;
                        }else if(stored_line.back().x<=0 && new_come_line.front().x>=0){
                            choosed_inner_line_index=inner_line_index_in_bev;
                        }else if(stored_line.back().x<=0 && new_come_line.front().x<=0 && new_come_line.back().x>0){
                            choosed_inner_line_index=inner_line_index_in_bev;
                        }
                    }else{
                        choosed_inner_line_index = inner_line_index_in_bev;
                        inner_line_index_vec.push_back(inner_line_index_in_bev);
                        is_first_find = false;
                    }                                    
                    inner_line.is_inner_line_f = true;
                    inner_line.inner_point_index_pair = std::make_pair(inner_line_start_inside_index, inner_line_end_inside_index);                    
                }

                                                       
            }
// #ifdef RELOCATE_INNER
//             std::cout << __FILE__ << "," << __LINE__ << "," << "replace_res "<<replace_res<<" ,choosed_inner_line_index:"<<choosed_inner_line_index<<std::endl;   
// #endif
        }

    } 
    return true;
}

bool LaneModel::EgoLaneIdWithoutCurb(const BevLaneElementGroup* ego_bev_lane_group, const StoredInfo& last_cycle_info, int cur_lane_size,int& cur_lane_num){
    // const SEhpOutputPath *ego_path_ptr = nullptr;
    // for(auto& path : paths.ehp_output_path_list){
    //     if(path.path_id_ == loc.path_id_ ){
    //         ego_path_ptr = &path;
    //     }
    // }
    // if(ego_path_ptr == nullptr){
    //     cur_lane_num = 0;
    //     return false; // no path found
    // }
    if(ego_bev_lane_group == nullptr){
        return false;
    }
    bool left_line_changed = true, right_line_changed = true;
    if(ego_bev_lane_group->size()>0){
        if(ego_bev_lane_group->front().right_front_connect_id != 0 && right_line_changed == true){
            for(auto& id: last_cycle_info.ego_right_line_ids){
                if(id == ego_bev_lane_group->front().right_front_connect_id){
                    right_line_changed = false;
                    break;
                }

            }
        }
        if(ego_bev_lane_group->front().right_line_base_id != 0 && right_line_changed == true){
            for(auto& id: last_cycle_info.ego_right_line_ids){
                if(id == ego_bev_lane_group->front().right_line_base_id){
                    right_line_changed = false;
                    break;
                }

            }
        }
        if(ego_bev_lane_group->front().right_back_connect_id != 0 && right_line_changed == true){
            for(auto& id: last_cycle_info.ego_right_line_ids){
                if(id == ego_bev_lane_group->front().right_back_connect_id){
                    right_line_changed = false;
                    break;
                }

            }
        }

        if(ego_bev_lane_group->back().left_front_connect_id != 0 && left_line_changed == true){
            for(auto& id: last_cycle_info.ego_left_line_ids){
                if(id == ego_bev_lane_group->back().left_front_connect_id){
                    left_line_changed = false;
                    break;
                }

            }
        }
        if(ego_bev_lane_group->back().left_line_base_id != 0 && left_line_changed == true){
            for(auto& id: last_cycle_info.ego_left_line_ids){
                if(id == ego_bev_lane_group->back().left_line_base_id){
                    left_line_changed = false;
                    break;
                }

            }
        }
        if(ego_bev_lane_group->back().left_back_connect_id != 0 && left_line_changed == true){
            for(auto& id: last_cycle_info.ego_left_line_ids){
                if(id == ego_bev_lane_group->back().left_back_connect_id){
                    left_line_changed = false;
                    break;
                }

            }
        }
    }
    bool lane_size_changed = false;
    
    if(last_cycle_info.lane_num != cur_lane_size){
        lane_size_changed = true;
    }

    //case   left_line_changed && right_line_not changed   
    if(left_line_changed == true && right_line_changed == false){
        cur_lane_num = last_cycle_info.lane_num;
    }else if(left_line_changed == false && right_line_changed == true){
        cur_lane_num = last_cycle_info.lane_num+1;
    }else{
        cur_lane_num = last_cycle_info.lane_num;
    }
    return true;
}

bool LaneModel::GetLaneIdFromSD(const SDLaneElementGroupSet& ele_group, int lane_num, uint8_t& loc_lane_num, uint64_t& lane_id){
    //lane_num 从右到左 1->2->3 ...
#ifdef LOC
    std::cout << __FILE__ << "," << __LINE__ << "," << "GetLaneIdFromSD start:"<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "lane_num:"<<lane_num<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "sd ele_groupSet.size():"<<ele_group.size()<<std::endl;
    for(int i =0; i<ele_group.size();i++){
        std::stringstream ss, ss1, ss2;
        std::cout << __FILE__ << "," << __LINE__ << ", group["<<i <<" ]: lanePos:"<<(int)ele_group[i].first<<std::endl;
        for(int j=0;j< ele_group[i].second.size();j++){
            std::cout <<" ,lane element:["<<j<<"]: ";
            ss<<" ,lane element:["<<j<<"]. lane_type: ";
            ss1<<" ,lane element:["<<j<<"]. lane_merge: ";
            ss2<<" ,lane element:["<<j<<"]. lane_split: ";
            for(auto & iter: ele_group[i].second[j].lane_nums){
                std::cout<<" ,"<<(int)iter;
            }
            for(auto & iter:ele_group[i].second[j].lane_types){
                ss<<" ,"<<(int)iter;
            }
            for(auto & iter:ele_group[i].second[j].lane_merges){
                ss1<<" ,"<<(int)iter;
            }
            for(auto & iter:ele_group[i].second[j].lane_splits){
                ss2<<" ,"<<(int)iter;
            }
            std::cout <<std::endl;
            std::cout<<ss.str()<<std::endl;
            std::cout<<ss1.str()<<std::endl;
            std::cout<<ss2.str()<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "remain_dist :"<< ele_group[i].second[j].remain_dist<< " ,first_merge_index: "<<ele_group[i].second[j].first_merge_index<<
    " ,first_split_index: "<<ele_group[i].second[j].first_split_index<<std::endl;
        }
        std::cout <<std::endl;
    }
#endif

    lane_id = 0;  
    loc_lane_num  = 0;
    if(lane_num >ele_group.size() || lane_num <=0){
        return false;
    }

    for(auto& group: ele_group){
        if(group.first == lane_num){
            if(group.second.size()>0 && group.second.front().lane_ids.size()>0){
                lane_id = group.second.front().lane_ids.front();
                loc_lane_num = lane_num;
#ifdef LOC
                std::cout << __FILE__ << "," << __LINE__ << "," << "true lane_id:"<<lane_id<<std::endl;
#endif
                return true;
            }
        }
    }
#ifdef LOC
    std::cout << __FILE__ << "," << __LINE__ << "," << "false lane_id:"<<lane_id<<std::endl;
#endif
    if(lane_id == 0){
        return false;
    }
    return true;
}

bool LaneModel::GetLineAttachCurbSide(const BevLineInnerConnect& side_line, double target_x, int& attach_curb_side){
    // std::cout << __FILE__ << "," << __LINE__ << "," << "GetLineAttachCurbSide "<<", target_x "<<target_x<<std::endl;
    attach_curb_side = 0;
    if(side_line.base_line_ptr == nullptr || side_line.base_line_ptr->total_line.size()<=0){
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << "side_line.id "<<side_line.base_line_ptr->id<<" ,side_line.base_line_ptr->total_line.back().x:"<<side_line.base_line_ptr->total_line.back().x<<std::endl;
    if(side_line.base_line_ptr->total_line.back().x >= target_x){
        if(side_line.back_connect_line_set_ptr.size()>0 && side_line.back_connect_line_set_ptr.begin()->first->total_line.size()>0){
            if(side_line.back_connect_line_set_ptr.begin()->first->total_line.front().x <=target_x){
                attach_curb_side = side_line.back_connect_line_set_ptr.begin()->first->attached_curb_side;
            }
        }else{
            attach_curb_side = side_line.base_line_ptr->attached_curb_side;
        }

    }else{
        //base line不在target_x里
        if(side_line.back_connect_line_set_ptr.size()>0 && side_line.back_connect_line_set_ptr.begin()->first->total_line.size()>0){
            // std::cout << __FILE__ << "," << __LINE__ << "," << "side_line.connect_back_id "<<side_line.back_connect_line_set_ptr.begin()->first->id<<" ,side_line.back_connect_line_set_ptr.begin()->first->total_line.back().x:"<<side_line.back_connect_line_set_ptr.begin()->first->total_line.back().x<<std::endl;
            if(side_line.back_connect_line_set_ptr.begin()->first->total_line.back().x >=target_x){
                attach_curb_side = side_line.back_connect_line_set_ptr.begin()->first->attached_curb_side;
            }
        }

    }

    return true;
}

bool LaneModel::GetLaneLineLengthToEgo(const BevLaneElement& lane_ele, double& left_line_front_length, double& right_line_front_length){
    left_line_front_length = 0;
    int left_line_first_zero_index = -1;//第一个x大于0的点
    right_line_front_length = 0;
    int right_line_first_zero_index = -1;//第一个x大于0的点
    for(int i=0;i<lane_ele.left_line_points.size();i++){
        if(left_line_first_zero_index>=0){
            left_line_front_length += sqrt(pow(lane_ele.left_line_points[i].x-lane_ele.left_line_points[i-1].x,2)+ 
                                            pow(lane_ele.left_line_points[i].y-lane_ele.left_line_points[i-1].y,2));
        }else{
            if(lane_ele.left_line_points[i].x >=0){
                left_line_first_zero_index = i;
                left_line_front_length += lane_ele.left_line_points[i].x;
            }
        }
    }

    for(int i=0;i<lane_ele.right_line_points.size();i++){
        if(right_line_first_zero_index>=0){
            right_line_front_length += sqrt(pow(lane_ele.right_line_points[i].x-lane_ele.right_line_points[i-1].x,2)+ 
                                            pow(lane_ele.right_line_points[i].y-lane_ele.right_line_points[i-1].y,2));
        }else{
            if(lane_ele.right_line_points[i].x >=0){
                right_line_first_zero_index = i;
                right_line_front_length += lane_ele.right_line_points[i].x;
            }
        }
    }
    return true;
}

bool LaneModel::ExtendLane(BevLaneElementGroup& lane_group, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset,const std::vector<std::pair<int,double>>& road_split_dir_vec){
    //当lane边线自车前方小于50m时，按照route的走形延长
    const double k_efm_lane_line_extension_thred = 80.0; 
    if(lane_group.size()==1){
        if(lane_group[0].left_line_points.size()>0 && lane_group[0].right_line_points.size()>0){
            double left_line_front_length = 0;
            double right_line_front_length = 0;
            GetLaneLineLengthToEgo(lane_group[0], left_line_front_length, right_line_front_length);

            //选长的一条，长的小于50的话，延长
            if(right_line_front_length > left_line_front_length){
                if(right_line_front_length <= k_efm_lane_line_extension_thred){
                    LaneLineExtension(lane_group[0].right_line_points, right_line_front_length, path_point_body_with_offset);
                }
            }else{
                if(left_line_front_length <= k_efm_lane_line_extension_thred){
                    LaneLineExtension(lane_group[0].left_line_points, left_line_front_length, path_point_body_with_offset);
                }
            }
           
        }
#ifdef LM_LANEGROUP
    {
    std::stringstream ss, ss1,ss2;
    ss<<"exten_left_line_x = [";
    ss1<<"exten_left_line_y = [";
    for(int j=0; j<lane_group[0].left_line_points.size();j++){
        if(j == lane_group[0].left_line_points.size()-1){
            ss<< lane_group[0].left_line_points[j].x;
        }else{
            ss<< lane_group[0].left_line_points[j].x<<" ,";
        }            
    }
    for(int j=0; j<lane_group[0].left_line_points.size();j++){
        if(j == lane_group[0].left_line_points.size()-1){
            ss1<< lane_group[0].left_line_points[j].y;
        }else{
            ss1<< lane_group[0].left_line_points[j].y<<" ,";
        }            
    }
    ss<<"]"<<std::endl;
    ss1<<"]"<<std::endl;
    ss2<< std::endl; 
    std::cout<<ss.str();   
    std::cout<<ss1.str();    
    }

    {
    std::stringstream ss, ss1,ss2;
    ss<<"exten_right_line_x = [";
    ss1<<"exten_right_line_y = [";
    for(int j=0; j<lane_group[0].right_line_points.size();j++){
        if(j == lane_group[0].right_line_points.size()-1){
            ss<< lane_group[0].right_line_points[j].x;
        }else{
            ss<< lane_group[0].right_line_points[j].x<<" ,";
        }            
    }
    for(int j=0; j<lane_group[0].right_line_points.size();j++){
        if(j == lane_group[0].right_line_points.size()-1){
            ss1<< lane_group[0].right_line_points[j].y;
        }else{
            ss1<< lane_group[0].right_line_points[j].y<<" ,";
        }            
    }
    ss<<"]"<<std::endl;
    ss1<<"]"<<std::endl;
    ss2<< std::endl; 
    std::cout<<ss.str();   
    std::cout<<ss1.str();    
    }
#endif
    }else if(lane_group.size()>1){
        //step1:判断 lane_group里是否有道路分歧：相邻lane的id不同，就是道路分歧,并记录匹配上的sd split的方向
        int sd_path_road_split_dir = 0;
        int road_split_lane_index = -1; //road split在这个lane的左侧
        for(int i = 1; i<lane_group.size(); i++){
            if(lane_group[i-1].left_base_line_ptr == nullptr || lane_group[i].right_base_line_ptr == nullptr) break;
            //从右侧判断
            if(lane_group[i-1].left_base_line_ptr->id != lane_group[i].right_base_line_ptr->id){
                double road_split_s = 0;
                if(lane_group[i-1].left_base_line_ptr->total_line.size()>0 && lane_group[i].right_base_line_ptr->total_line.size()>0){
                    if(lane_group[i-1].left_base_line_ptr->total_line.front().x > lane_group[i].right_base_line_ptr->total_line.front().x){
                        road_split_s = lane_group[i-1].left_base_line_ptr->total_line.front().x;
                    }else{
                        road_split_s = lane_group[i].right_base_line_ptr->total_line.front().x;
                    }
                }
                for(auto& iter: road_split_dir_vec){
                    if(fabs(road_split_s -iter.first) <15){//在一定范围内，认为是sd和bev匹配上
                        sd_path_road_split_dir = iter.first;
                        road_split_lane_index = i-1;
                        break;
                    }                    
                }

            }
            if(sd_path_road_split_dir>0 && road_split_lane_index >=0){
                break;
            }
        }
        //step2:存在道路分歧，那么根据道路分歧的方向，延长对应的方向上的lane
        if(sd_path_road_split_dir>0 && road_split_lane_index >=0 && road_split_lane_index< lane_group.size()){
            
            int lane_start_index = -1;
            int lane_end_index = -1;
            if(sd_path_road_split_dir == 1){//path向左, lane_group左侧的lane延长
                lane_start_index = 0;
                lane_end_index = road_split_lane_index;
            }else{
                lane_start_index = road_split_lane_index+1;
                lane_end_index = lane_group.size()-1;                    
            }
            if(lane_start_index>=0 && lane_end_index>=lane_start_index && lane_end_index<lane_group.size()){
                for(int j= lane_start_index; j<=lane_end_index ; j++){
                    double left_line_front_length = 0;
                    double right_line_front_length = 0;
                    GetLaneLineLengthToEgo(lane_group[lane_start_index], left_line_front_length, right_line_front_length);   
                    //选长的一条，长的小于50的话，延长
                    if(right_line_front_length > left_line_front_length){
                        if(right_line_front_length <= k_efm_lane_line_extension_thred){
                            LaneLineExtension(lane_group[lane_start_index].right_line_points, right_line_front_length, path_point_body_with_offset);
                        }
                    }else{
                        if(left_line_front_length <= k_efm_lane_line_extension_thred){
                            LaneLineExtension(lane_group[lane_start_index].left_line_points, left_line_front_length, path_point_body_with_offset);
                        }
                    }                     
                }
            }
        }      

    }
    return true;
}

void LaneModel::LaneLineExtension(EFMRefLinePoints& lane_line, double line_dist, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset) {
    
    // 定义阈值
    // const double k_efm_lane_line_extension_dis_threshold = 60.0;  // 阈值60米
    const double k_efm_lane_line_extension_max_dis = 100.0;       // 最大延长长度100米
    const double k_efm_lane_line_extension_gap = 2.5;             // 散点间隔0.1米
    const int k_efm_lane_line_extension_min_point_num = 4;        // 所延长的线至少有4个点

    //找到距离大于lane 长边的 最远点的 route行点
    auto GetRoutePoints = [&path_point_body_with_offset,k_efm_lane_line_extension_max_dis](double tar_dist)->std::vector<EFMPoint>{
        std::vector<EFMPoint> res_route_points{};
        for(auto& iter:path_point_body_with_offset){
            if(iter.second >=k_efm_lane_line_extension_max_dis){
                break;
            }else if(iter.second >(tar_dist)){
                res_route_points.push_back(iter.first);
            }
        }
        return res_route_points;
    };


    // 将 map_lane.lanes[2].left_line.linePoints 转换为 EFMRefLinePoints
    EFMRefLinePoints routing_line;
    routing_line.clear();
    routing_line = GetRoutePoints(line_dist);

    // 延长车道线
    auto extend_line = [line_dist, k_efm_lane_line_extension_max_dis]
                        (EFMRefLinePoints& lane_line, EFMRefLinePoints& routing_line) {
        if (lane_line.empty() || routing_line.empty()) return;
        
        if (line_dist >= k_efm_lane_line_extension_max_dis) return;
        //计算lane_line最后一个点和routing_line第一个点的deltaX和deltaY,然后平移
        double delta_x = routing_line.front().x - lane_line.back().x;
        double delta_y = routing_line.front().y - lane_line.back().y;
        for(auto& p:routing_line){
            EFMPoint res(p.x-delta_x, p.y-delta_y);
            lane_line.push_back(res);
        }
    };

    // 延长所有车道线
    extend_line(lane_line, routing_line);

    //做个b样条差值平滑
    std::vector<double> between_s{};
    std::vector<int> inert_num{};    
    for(int i= 0;i<lane_line.size();i++){
        if(i==0){
            // between_s.push_back(0);
        }else{
            double s_tmp = sqrt(pow(lane_line[i].x - lane_line[i - 1].x, 2) +
                                    pow(lane_line[i].y - lane_line[i - 1].y, 2));
            between_s.push_back(s_tmp);    
            if(s_tmp>3){
                inert_num.push_back(s_tmp /2.5 ) ;               
            }else{
                inert_num.push_back(0);
            }
                    
        }
    }
    std::vector<Point2D<double>> outputs;
    std::vector<double> curves;
    BSpline::CubicBSplineInterpolate(lane_line, inert_num, outputs, curves);
    BSpline::CubicBSplineSmooth(outputs,lane_line,true);

}

 void LaneModel::ConnectLinePostProc(std::vector<BevLineInnerConnect>& connected_lines, std::unordered_map<int,int>& multi_line_index){
    //查找，back的有没有出现在base中，有，把base的这条删掉,
    // new 20251013, 删除时还需，两条线的重叠区域的长度，以及重叠区域的平均间距； 哪两条，比如base 1, back connect 2； base 2,  那么计算1和2的间距
    std::vector<std::pair<int, int>> back_connect_ids{};
    for(auto& line: connected_lines){
        if(line.back_connect_line_set_ptr.size()>0){
            back_connect_ids.push_back(std::make_pair(line.back_connect_line_set_ptr.begin()->first->id, line.base_line_ptr->id));
        }       
    }

// #ifdef LM_CONNECT_RES
//         std::cout << __FILE__ << "," << __LINE__ << "," << "back_connect_ids.size():"<<back_connect_ids.size()<<std::endl;
//     for(auto back_id: back_connect_ids){
//         std::cout<<" ,back_id_base_id: "<<back_id.second<< ", back_id:"<<back_id.first<<std::endl;
//     }
// #endif

    for (auto it = connected_lines.begin(); it != connected_lines.end(); ) {
// #ifdef LM_CONNECT_RES
//     std::cout << __FILE__ << "," << __LINE__ << "," << "connected_lines->base_line_ptr->id:"<<it->base_line_ptr->id;
//     if(it->back_connect_line_set_ptr.size()>0){
//         std::cout <<" , back id: "<< it->back_connect_line_set_ptr.begin()->first->id;
//     }
//     std::cout<<std::endl;
// #endif
        bool delete_done = false;
        for(auto back_id: back_connect_ids){
        // std::cout<<" ,back_id_base_id: "<<back_id.second << ", back_id:"<<back_id.first<<std::endl;
            if ((*it).base_line_ptr->id == back_id.first) {
                //计算间距
                int overlap_start_index=-1, overlap_end_index = -1;
                double dist_l = 0;
                BevLineInnerConnect * line_ptr = nullptr;
                for(auto& line: connected_lines){
                    if(line.base_line_ptr->id ==  back_id.second){
                        line_ptr = &line;
                        break;
                    }
                }
                if(line_ptr != nullptr){
        // std::cout << __FILE__ << "," << __LINE__ << "," << "line_ptr->base_line_ptr->id:"<<line_ptr->base_line_ptr->id<<" ,it->base_line_ptr->id:"<<it->base_line_ptr->id<<std::endl;
                    CalTwoLineDist(line_ptr->base_line_ptr->total_line, it->base_line_ptr->total_line, dist_l, 1,overlap_start_index, overlap_end_index);
                }
                
                // std::cout << __FILE__ << "," << __LINE__ << "," << "overlap_start_index:"<<overlap_start_index<< " ,overlap_end_index:"<<overlap_end_index<<" ,dist_l:"<<dist_l<<std::endl;                  
                if(overlap_start_index>=0 && overlap_end_index>=0 && (overlap_end_index-overlap_start_index)>3 && dist_l>0.5){
                    continue;
                }
                it = connected_lines.erase(it);  // erase 返回下一个有效的迭代器
                delete_done = true;
                break;
            }
        }
        if(delete_done){
        }else{
            ++it;
        }

    }

    multi_line_index.clear();
    //重做split和merge的index
    std::map<size_t, int> base_line_id_index_map{};
    for(int i=0;i<connected_lines.size();i++){
        if(base_line_id_index_map.find(connected_lines[i].base_line_ptr->id)!= base_line_id_index_map.end()){//找到重复的
            multi_line_index.emplace(std::make_pair(i, base_line_id_index_map.at(connected_lines[i].base_line_ptr->id)));
            multi_line_index.emplace(std::make_pair(base_line_id_index_map.at(connected_lines[i].base_line_ptr->id), i));
        }
        base_line_id_index_map.emplace(std::make_pair(connected_lines[i].base_line_ptr->id,i));
    }
}
}
