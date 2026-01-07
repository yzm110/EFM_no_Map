#pragma once

#include "CompileConfig.h"
// #include "common/framework.h"
// #ifdef __QNX__
// #include "gptp/gptp.h"
// #endif
#include "CommonDataType.h"
#include <unordered_map>
#include <sstream>
#include "EHPData.h"
#include <optional>
#include "trans_coordinate.h"
#include <memory>
#include <limits>
#include <numeric>
#include "bspline.h"

namespace NoMapEFM{
#define LINE_CUT_DISTANCE 4  // m
class LaneModel{
    public:
    inline static std::vector<uint64_t> sd_links_for_loc_only_from_left_ = {11008555596651, 9500012384131, 11008555596551, 11008480037636, 9500421612806, 9500421612805, 11016949165882,9500421452786,9500421552515,9500421552514, 9500421640898,9500421640899,9500421566989,9500421566988,11003008537935,11003008537936,9500421386997,11003029411560
                                                                            ,11005923573302,11005923573303,11005923573304,11005923573305,9500421563333};
    // inline static std::vector<uint64_t> sd_links_for_lane_size_4_ = {11017440455957,11006248726585,11006248726586,11003871673354,11003871673355,11003871658814};

    inline static std::vector<uint64_t> sd_links_for_loc_only_from_right_ = {11004889929017, 11003008598304};
    bool Execute(std::vector<BevLineInnerS>& bev_lines, BevLaneElementGroupSet& bev_lane_group_set, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info,
                        const StoredInfo& last_cycle_info, const MapRawDataMap& sd_data_index, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, const std::vector<std::pair<int,double>>& road_split_dir_vec);
        
    private:
    //计算基本属性
    void CalBevLineAttribute(std::vector<BevLineInnerS>& bev_lines);
    void FixBevAbnormalAtt(BevLineInnerS& bev_line); //修复bev线型的异常情况
    //排序
    void SortBevLine(std::vector<BevLineInnerS>& bev_lines,std::vector<BevLineInnerS*>& bev_lines_in_side_ego,
                            std::vector<BevLineInnerS*>& bev_lines_not_in_side_ego);
    //连线
    bool CutLineInSplitMerge(EFMRefLinePoints& line, bool is_split);
    void ConnectLine(std::vector<BevLineInner>& bev_lines, BevLineInner& base_line,std::vector<BevLineInnerConnect>& connect_lines_res, int& is_multi_lines);
    int LineRelativeLoc(EFMRefLinePoints line1, EFMRefLinePoints line2, std::pair<EFMRefLinePoints,int>& line_res01, 
                        std::pair<EFMRefLinePoints,int>& line_res23, bool is_line2_side_curv);
    int IsNeedConnect(EFMPoint tar_point, EFMPoint tar_point_dir,EFMRefLinePoints line, bool is_front);
    void MultiLinesLeftRight(std::vector<std::pair<std::pair<EFMRefLinePoints,int>,BevLineInner*>>& connect_set);
    void ConnectBevLines(std::vector<BevLineInnerS*>& base_lines,std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines_res, std::unordered_map<int,int>& multi_line_index);
    bool FixConnectLines(std::vector<BevLineInnerConnect>& connected_lines);
    //生成车道组
    bool GenerateLaneGroupSet(std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines, const std::unordered_map<int,int>& multi_line_index, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, const std::vector<std::pair<int,double>>& road_split_dir_vec,
                                BevLaneElementGroupSet& lane_group_set, int& ego_lane_left_index, int& ego_lane_right_index);
    bool GenerateLaneGroup(std::vector<BevLineInnerS>& bev_lines, std::vector<BevLineInnerConnect>& connected_lines, const std::unordered_map<int,int>& multi_line_index, int nearest_right, int nearest_left, int ego_dir, BevLaneElementGroup& res_lane_group, int& res_right_line_index, int& res_left_line_index);
    //包含split， merge的connect线，拿到split或merge的线
    bool GetSplitMergeLine(BevLineInnerConnect& connect_line1, BevLineInnerConnect& connect_line2, int is_split_merge, BevLineInnerS* &split_left_line, BevLineInnerS* &split_right_line);
    bool IsDrivableSplitMerge(BevLineInnerConnect& connect_line1, BevLineInnerConnect& connect_line2, int is_split_merge, const std::vector<BevLineInnerS>& bev_lines,BevLineInnerS* &split_left_line, BevLineInnerS* &split_right_line,int dir);
    bool IsDrivableSplitMerge(const EFMRefLinePoints& line1_points, const EFMRefLinePoints& line2_points, int line1_type, int line2_type, int line1_id, int line2_id ,const std::vector<BevLineInnerS>& bev_lines, int dir, int line1_attached_curb_side, int line2_attached_curb_side);
    bool CalTwoLineDist(const EFMRefLinePoints& base_line, const EFMRefLinePoints& cal_line, double& cal_to_base_l, int gap_index);
    bool CalTwoLineDist(const EFMRefLinePoints& base_line, const EFMRefLinePoints& cal_line, double& cal_to_base_l, int gap_index, int& overlap_start_index, int& overlap_end_index);
    bool GenerateInnerLane(const BevLineInnerConnect& connect_line_left, const BevLineInnerConnect& connect_line_right, std::vector<BevLineInnerS>& bev_lines, 
                            int choosed_inner_line_index,const BevLineInnerConnect& replace_line, int replace_res, BevLaneElementGroup& lane_group);
    bool GenerateInnerLaneFirstTime( BevLineInnerConnect& connect_line_left,  BevLineInnerConnect& connect_line_right, std::vector<BevLineInnerS>& bev_lines, 
                                  int choosed_inner_line_index, BevLineInnerConnect& replace_line, int replace_res, BevLaneElementGroup& lane_group);
    bool FindLineInner( BevLineInnerConnect& left_line, BevLineInnerConnect& right_line, std::vector<BevLineInnerS>& bev_lines, int &choosed_inner_line_index ,BevLineInnerConnect& inner_line_replace_res, int& replace_res);
    //判断inner_line和sideline之间是否可行驶，是否需要替换side line
    bool InnerLineSubJudge(BevLineInnerConnect& side_line, BevLineInner& inner_line, int inner_line_start_inside_index, int inner_line_end_inside_index, int side_line_dir, double inner_to_left_l, double inner_to_right_l, int left_side_line_location_type, int right_side_line_location_type,
                                    std::vector<BevLineInnerS>& bev_lines, int inner_line_lean_dir, bool& is_first_find, int inner_line_index_in_bev, std::vector<int>& inner_line_index_vec,
                                    int &choosed_inner_line_index,BevLineInnerConnect& inner_line_replace_res, int& replace_res);
    bool JudgeInnerLineAndReplaceLine(const BevLineInnerConnect& replace_line, const BevLineInnerConnect& inner_line, int& rep_to_inner_dir);
    //找到x大于target_x的线的type
    bool GetLineType(const BevLineInnerConnect& side_line, double target_x, int search_type,int& line_type);
    //找到x大于target_x的线的attach_curb_side
    bool GetLineAttachCurbSide(const BevLineInnerConnect& side_line, double target_x, int& attach_curb_side);
    bool GenerateLaneElement(const BevLineInnerConnect& line_left, const BevLineInnerConnect& line_right, BevLaneElement& lane);
    bool MakeSideLine(const BevLineInnerConnect& bev_line, std::vector<LineTypeInfo>& types, EFMRefLinePoints &line_points);
    // bool GenerateSideCurb(const std::vector<BevLineInnerConnect>& connect_lines, BevLaneElementGroupSet& bev_lane_group_set);
    //判断两线是不是split
    bool IsTowLineSplit(const BevLineInnerConnect& left_line, const BevLineInnerConnect& right_line, bool& is_split, bool& is_merge);
    void PlotConnectLines(const std::vector<BevLineInnerConnect>& connected_lines);
    void PlotEgoLaneGroup(const BevLaneElementGroup& lane_group);
    void PlotLeftLaneGroup(const BevLaneElementGroup& lane_group);
    void PlotRightLaneGroup(const BevLaneElementGroup& lane_group);

    //merge的index做成
    bool MultiLineIndexPostProcess(std::vector<BevLineInnerConnect>& connect_lines ,std::unordered_map<int,int>& multi_line_index);

    //根据当前BEV数据和SD数据，判断自车所处车道，做成顺序如下：
    //1，判断自车与右路沿的
    //3，无路沿，且车道数对不上，根据merge/split属性判断自车所在车道
    //以上都无法判断，则返回false
    bool GetEgoLaneIdx(const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, BevLineInnerS* ego_left,BevLineInnerS* ego_right,SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info,const BevLaneElementGroup* ego_bev_lane_group, const StoredInfo& last_cycle_info);
    bool GetLaneIdViaLeftSide(int ego_left_index, int left_curb_index, const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info);
    bool GetLaneIdViaRightSide(int ego_right_index, int right_curb_index, const std::vector<BevLineInnerS*>& bev_lines_in_side_ego, SEhpOutputLoc& lane_loc, SDLaneElementGroupSet& ele_group, LocJudgeInfo& loc_res_info);
    //
    bool GetLaneIdFromSD(const SDLaneElementGroupSet& ele_group, int lane_num, uint8_t& loc_lane_num,uint64_t& lane_id);
    bool EgoLaneIdWithoutCurb(const BevLaneElementGroup* ego_bev_lane_group, const StoredInfo& last_cycle_info, int cur_lane_size,int& cur_lane_num);

    //延长
    bool GetLaneLineLengthToEgo(const BevLaneElement& lane_ele, double& left_line_front_length, double& right_line_front_length);
    bool ExtendLane(BevLaneElementGroup& lane_group, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, const std::vector<std::pair<int,double>>& road_split_dir_vec);
    void LaneLineExtension(EFMRefLinePoints& lane_line, double line_dist, const std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset);
    std::vector<BevLineInnerS*> bev_lines_in_side_ego_;//认为自车落在线区间的线，并且左右排好序的，[0]是最右
    std::vector<BevLineInnerS*> bev_lines_not_in_side_ego_;//未落在自车区间的线，未排序
    std::vector<BevLineInnerConnect> connected_lines_;//连好的线
    std::unordered_map<int,int> multi_line_index_;//connected_lines_里哪两条线组成merge或者split,
    int ego_lane_left_index_ = -1; //自车所在车道的index,在connected_lines_里的
    int ego_lane_right_index_ = -1; //自车所在车道的index,在connected_lines_里的

    void ConnectLinePostProc(std::vector<BevLineInnerConnect>& connected_lines, std::unordered_map<int,int>& multi_line_index);//将inside的范围拓宽之后，需要处理下重复连接的线，back和front都有的优先删除front连接的线

    std::shared_ptr<const MapRawDataMap> sd_data_index_;
};

}
