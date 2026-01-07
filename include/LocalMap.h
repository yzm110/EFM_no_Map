#pragma once

#include <vector>
#include <algorithm>
#include <deque>

#include "CompileConfig.h"
// #include "common/framework.h"
// #ifdef __QNX__
// #include "gptp/gptp.h"
// #endif
#include "CommonDataType.h"
#include "EHPData.h"
#include "Calibration.h"
#include "CommonMathMethod.h"
#include "log_manager.h"


namespace NoMapEFM{

struct LaneIdx{
public:
    LaneIdx() {
        sd_lane_idx_ = std::make_pair(-1, -1); //-1表示无效值
        bev_lane_idx_ = std::make_pair(-1, -1); //-1表示无效值
    }
    ~LaneIdx() {}

    std::pair<int32_t, int32_t> sd_lane_idx_;
    std::pair<int32_t, int32_t> bev_lane_idx_;
};


class LocalMap{
    public:
    bool Execute(BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group, NodeInfo& node_info, 
                SEhpOutputLoc& lane_loc, SEhpOutputPathList& paths);
    //获取自车所在车道的idx
    LaneIdx GetEgoLaneIdx() { return ego_lane_idx_;}
    //获取左车道的idx
    LaneIdx GetLeftLaneIdx() { return left_lane_idx_;}
    //获取右车道的idx
    LaneIdx GetRightLaneIdx() { return right_lane_idx_;}
    //获取左左车道的idx
    LaneIdx GetLeftLeftLaneIdx() { return left_left_lane_idx_;}
    //获取右右车道的idx
    LaneIdx GetRightRightLaneIdx() { return right_right_lane_idx_;}
    //获取推荐车道的idx
    LaneIdx GetRefLaneIdx() { return ref_lane_idx_; }
    //对比前后帧中心线变化，修改本帧中心线，以保证连续性
    bool SmoothCenterLineForLastCycle(SEhpOutputLoc last_cycle_loc, std::deque<std::pair<int, EFMRefLinePoints>> last_cycle_center_lines,
                                     SEhpOutputLoc& curr_cycle_loc, std::deque<std::pair<int, EFMRefLinePoints>>& curr_cycle_center_lines);

    private:
    bool MakeCenterLines(BevLaneElement& ego_lane, BevLaneElement& left_lane, BevLaneElement& right_lane, uint8_t prior_dir, bool is_off_ramp);
    EFMRefLinePoints GenerateCenterLine(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, int lane_id);
    EFMRefLinePoints offsetBevLine(const EFMRefLinePoints& BevLine, double offset);
    void MergeCdnEgoLaneStitch(const BevLaneElement& ego_lane, int lane_merge_dir, float s_end, EFMRefLinePoints& egolane_Points);
    void MergeSplitCdnEgoLaneOffset(const BevLaneElement& ego_lane, int lane_merge_dir, float merge_s_start, float merge_s_end,
                                    int lane_split_dir, float split_s_start, float split_s_end, EFMRefLinePoints& egolane_Points);
    EFMPoint LinearInterp(EFMPoint p1, EFMPoint p2, double x);
    void IsWideLane(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, int change_lane_flag, bool& is_wide_lane, int& offset_dir);
    void CenterLineStitch(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, EFMRefLinePoints& Centerline);
    EFMPoint interpolateAtLength(const EFMRefLinePoints& line, const std::vector<double>& arc_lengths, double target_length);
    double PointLineDistance(EFMPoint p1, EFMPoint p2, EFMPoint p3);

    void InitBevEle(BevLaneElementGroupSet& bev_lane_group_set);


    //根据当前BEV数据和SD数据，判断自车所处车道，做成顺序如下：
    //1，判断车道数是否一致，如果一致，一致则可以直接得到自车所在车道
    //2，判断自车与两边路沿的距离，根据宽度推测自车所在车道
    //3，无路沿，且车道数对不上，根据merge/split属性判断自车所在车道
    //以上都无法判断，则返回false
    bool GetEgoLaneIdx(const BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group);
    
    //判断上一帧自车所在车道位置，以及自车的左右边线ID，与本帧自车左右边线的ID比较，判断自车是否变道，
    //所在结果可能存在三种结果：和上帧一致、上帧左车道、上帧右车道
    //如果上帧为无效值或者车道ID无法关联，则返回false
    bool GetEgoLaneIdxForLastLane(const BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group, LaneIdx& last_ego_lane_idx);

    //判断自车是否在目标车道，确定是否需要做成变道的推荐信息
    bool MakeNodeInfoForChangeLane(SDLaneElementGroupSet& ele_group);

    //对每个group生成中心线
    bool MakeCenterLine(int32_t lane_idx ,BevLaneElementGroup& bev_ele_group);

    //做成车道的中心线通用做法
    bool MakeCenterLineCommon(BevLaneElement& bev_ele);

    //做成自车左/右边车道的中心线，左右的判断是：is_left
    bool MakeCenterLineSide(BevLaneElementGroup& bev_ele_group, int32_t lane_idx);

    //判断前方是否是merge，前方车道线未向交，但是宽度越来越小，也认为是merge
    //merge_dir，merge方向：0 无merge；1 向左merge；2 向右merge
    //left_cross_point， right_cross_point：merge点在左右边线的位置，取point的index
    //left_narrow_pos， right_narrow_pos：merge点去前车辆不能通行位置，通常宽度小于2米(可配置)，取point的index
    bool GetMergeDir(BevLaneElement& bev_ele, uint8_t& merge_dir, int32_t& left_merge_point, 
                     int32_t& right_merge_point, int32_t& left_narrow_pos, int32_t& right_narrow_pos, int32_t lane_idx);
    //参数同GetMergeDir
    bool GetMergePoint(BevLaneElement& bev_ele, int32_t& left_merge_point, 
                       int32_t& right_merge_point, int32_t& left_narrow_pos, int32_t& right_narrow_pos);

    //判断后方是否是split，后方车道线未向交，但是宽度越来越小，也认为是split
    //split_dir，split方向：0 无split；1 向左split；2 向右split
    //left_split_point， right_split_point：split点在左右边线的位置，取point的index
    //left_split_end_pos， right_split_end_pos：split点之后的左右车道线连接位置：1，需考虑平化度；2，取边线上可变道点后50米(可调)；3，满足车辆可行驶宽度；取point的index
    bool GetSplitDir(BevLaneElement& bev_ele, uint8_t& split_dir, int32_t& left_split_point, 
                     int32_t& right_split_point, int32_t& left_split_end_pos, int32_t& right_split_end_pos);

    //参数同GetSplitDir
    bool GetSplitPoint(EFMRefLinePoints& left_line, EFMRefLinePoints& right_line, int32_t& left_split_point, int32_t& right_split_point);

    //找到当前group下，两边车道线完整，满足2个条件：1，边线最长；2，车道宽度满足车辆通行，通常2.5米
    bool GetMainLaneIdxForGroup(BevLaneElementGroup& bev_ele_group, int32_t& main_lane_idx);

    //做成左右车道线重叠的部分
    bool MakeOverlapLane(BevLaneElementGroup& bev_ele_group);

    //根据left_x_start和left_x_end，右车道插值出该点
    bool InterpolateBevLine(EFMRefLinePoints& line_points, double left_x_start, int32_t& start_index, double left_x_end, 
                            int32_t& end_index, EFMRefLinePoints& side_line_points);

    //检查做成的中心线是否有问题
    //1，是否有曲率过大，无法控车的情况，如果有，直接修正
    //2，是否有中心线不连续的情况，如果有，直接修正
    //3，是否有中心线不合法的情况，不合法包括：距离路沿过近(压/小于阀值)；距离实线过近(压/距离小于阀值)；
    //如果出现3的场景，需把起关联的车道线全部返回
    bool CheckCenterLine(BevLaneElementGroup& bev_ele_group, std::vector<int32_t>& target_line, bool& center_line_valid);

    //针对检查3的场景，根据关联的车道线，进行修正中心线
    bool ChangeCenterLine(BevLaneElementGroup& bev_ele_group, std::vector<int32_t>& target_line);

    //获取当前车道线的类型信息
    LineTypeInfo GetlineTypeForIdx(const EFMRefLinePoints& line_points, std::vector<LineTypeInfo>& line_type_infos, int32_t s_pos, int32_t e_pos);

    //判断是否是虚拟合流
    bool IsVirtualMerge(BevLaneElement& bev_ele, BevLaneElement& side_bev_ele, int32_t& merge_point, int32_t& narrow_pos, bool is_left);

    //根据merge方向和合流点，做成合流的中心线
    bool MakeCenterLineForMerge(BevLaneElement& bev_ele, uint8_t merge_dir, int32_t left_merge_point, 
                                int32_t right_merge_point, int32_t left_narrow_pos, int32_t right_narrow_pos, 
                                BevLaneElement& side_bev_ele);

    //根据变道方向和变道点，做成中心线，适用自车道
    bool MakeCenterLineForChangeLane(BevLaneElement& bev_ele, uint8_t prior_dir);

    //获取分歧车道在分歧点前的公用部分
    bool GetMainLinePoint(const BevLaneElementGroup& bev_ele_group, EFMRefLinePoints& main_left_line_points, 
                          EFMRefLinePoints& main_right_line_points, int32_t lane_idx, bool& is_wide,
                          std::vector<LineTypeInfo>& main_left_types, std::vector<LineTypeInfo>& main_right_types, uint8_t& main_line);

    //生成中心线通用做法
    //merge_dir：0，表示无merge；1，表示向左merge；2，表示向右merge
    //main_line：0，表示自动选择边线；1，表示使用左线偏移；2，表示使用右线偏移
    EFMRefLinePoints GetCenterLineCommon(double& lane_width_last_cycle, EFMRefLinePoints& left_points, EFMRefLinePoints& right_points,std::vector<LineTypeInfo> left_types,
                                               std::vector<LineTypeInfo> right_types, uint8_t merge_dir, uint8_t main_line, int32_t lane_idx, bool is_neighbor_lane);

    //获取大于X方向的所有lane
    bool GetLinePointUpX(const EFMRefLinePoints& left_line_points, const EFMRefLinePoints& right_line_points, 
                                EFMRefLinePoints& up_left_line_points, EFMRefLinePoints& up_right_line_points, double x);

    //删除起点宽度merge_lane_start_width的线
    void DeleteLineForStartMerge(EFMRefLinePoints& left_line_points, EFMRefLinePoints& right_line_points);
    
    //分歧点中心线做成
    bool AlterLineForMainLine(BevLaneElement& bev_ele, EFMRefLinePoints& center_line_points, const BevLaneElementGroup& bev_ele_group);

    //做成自车道信息，node信息
    bool MakeRefLine(BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group);

    //获取合流点的偏移量
    bool GetPointOffset(const EFMRefLinePoints& line_points, int32_t s_merge_point, int32_t e_merge_point, 
                        double& s_merge_offset, double& e_merge_offset);

    //根据merge修改node_info_信息
    bool MakeNodeInfoForMerge(uint8_t merge_dir, double s_merge_offset, double e_merge_offset);

    //根据ele_group，获取SD的main_lane_idx
    bool GetEleGroupMainLane(const SDLaneElementGroup& ele_group, int32_t& main_lane_idx, 
                             const BevLaneElementGroup& bev_group, int32_t bev_main_lane_idx);
    
    //获取bev_group中，和ele_group中main_lane_idx对应的车道信息
    bool GetBevMappingEle(SDLaneElementGroup& ele_group, int32_t ele_main_lane_idx, 
                                const BevLaneElementGroup& bev_group, int32_t bev_main_lane_idx, 
                                std::vector<std::pair<int32_t, int32_t>>& bev_mapping_eles,
                                int32_t& bev_group_dest_idx, int32_t& ele_group_dest_idx);
    //根据split信息，修改node_info_信息
    bool MakeNodeInfoForSplit(const SDLaneElementGroup& ele_group, int32_t ele_main_lane_idx, 
                              int32_t ele_group_dest_idx, double s_split_offset, double e_split_offset);

    //查找非自车的最近旁车道，并返回车道idx，判断是做成左侧还是右侧车道
    bool GetSideLane(int32_t ele_ego_ele_idx, const std::vector<std::pair<int32_t, int32_t>>& bev_mapping_eles, 
                     int32_t& side_lane_idx, bool& is_left);

    //查找过自车的所有边线
    bool GetDistEgoLines(const BevLaneElementGroup& bev_ele_group, std::map<double, 
        std::vector<EFMRefLinePoints>>& dist_ego_lines_map, double& min_dist, double& max_dist);

    //重新2.5一个点取样
    bool ReSamplingPoint(BevLaneElement& bev_ele);
    void ReSamplingPointOne(EFMRefLinePoints& line_points);

    //超宽车道时，判断自车所在的车道
    bool GetNearIdxForMainLine(const BevLaneElementGroup& bev_ele_group, EFMRefLinePoints& main_left_line_points, 
                               EFMRefLinePoints& main_right_line_points, int32_t& cur_nearest_index);
    //获取指定X后面的linetype，没有则返回初始值
    bool GetBevLineType(std::vector<LineTypeInfo>& types, double start_x, BstdsLineType& re_type);

    //获取左右车道的中心线
    EFMRefLinePoints GetCenterLineCommonOne(EFMRefLinePoints& left_points,EFMRefLinePoints& right_points,
                                            double all_right_dkappas,double all_left_dkappas,
                                            uint8_t main_line,double left_min_distance, double right_min_distance);

    bool ego_wide_lane_ = false; //自车道超宽标志位
    //以下接口待地图ok后重新匹配
    uint8_t change_lane_flag_; //0：不需要变道；1：向左变道；2：向右变道；
    uint8_t follow_lane_flag_; //0：不需要寻线；1：向左寻线；2：向右寻线；

    BevLaneElementGroup bev_ego_ele_; //自车所在的车道组，包含所有的车道信息
    BevLaneElementGroup bev_left_ele_; //左侧车道组，包含所有的车道信息
    BevLaneElementGroup bev_right_ele_; //右侧车道组，包含所有的车道信息
    BevLaneElementGroup bev_left_left_ele_; //左左侧车道组，包含所有的车道信息
    BevLaneElementGroup bev_right_right_ele_; //右右侧车道组，包含所有的车道信息

    //LaneIdx ego_lane_idx_;
    NodeInfo node_info_;
    uint8_t prior_dir_; // 0: 无方向；1：向左；2：向右
    SEhpOutputLoc lane_loc_;
    SDLaneElementGroup ego_sd_group_; //自车所在的车道组，包含所有的车道信息
    SEhpOutputPathList paths_; //车道组对应的路径信息

    LaneIdx left_lane_idx_; //左侧车道的index
    LaneIdx right_lane_idx_; //右侧车道的index
    LaneIdx left_left_lane_idx_; //左左侧车道的index
    LaneIdx right_right_lane_idx_; //右右侧车道的index
    LaneIdx ref_lane_idx_; //推荐车道的index
    LaneIdx ego_lane_idx_;  //自车车道的index
};

}
