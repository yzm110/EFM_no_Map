#pragma once

#include "CompileConfig.h"
#include "ComponentEFMTask.h"
#include "CommonDataType.h"
#include <cmath>
#include "LocalMap.h"
#include <algorithm>
#include <map>
#include <deque>
#include <limits>
#include "LocalMap.h"

namespace NoMapEFM{

class WrapperOutput{
    public:
    typedef struct BevLaneIds_S{
        BevLaneIds_S()
        : left_line_base_id(0)
        , right_line_base_id(0)
        , left_front_connect_id(0)
        , left_back_connect_id(0)
        , right_front_connect_id(0)
        , right_back_connect_id(0){}

        BevLaneIds_S(int le_base, int le_back, int le_front, int ri_base, int ri_back, int ri_front)
        : left_line_base_id(le_base)
        , right_line_base_id(ri_base)
        , left_front_connect_id(le_front)
        , left_back_connect_id(le_back)
        , right_front_connect_id(ri_front)
        , right_back_connect_id(ri_back){}

        bool operator==(const BevLaneIds_S& other) const {
            return left_line_base_id == other.left_line_base_id &&
                right_line_base_id == other.right_line_base_id &&
                left_front_connect_id == other.left_front_connect_id &&
                left_back_connect_id == other.left_back_connect_id &&
                right_front_connect_id == other.right_front_connect_id &&
                right_back_connect_id == other.right_back_connect_id;
        }
        bool operator!=(const BevLaneIds_S& other) const {
            return left_line_base_id != other.left_line_base_id ||
                right_line_base_id != other.right_line_base_id ||
                left_front_connect_id != other.left_front_connect_id ||
                left_back_connect_id != other.left_back_connect_id ||
                right_front_connect_id != other.right_front_connect_id ||
                right_back_connect_id != other.right_back_connect_id;
        }

        int left_line_base_id;
        int right_line_base_id;
        int left_front_connect_id;//线本身的vector的front()和back()
        int left_back_connect_id;
        int right_front_connect_id;
        int right_back_connect_id;        
    }BevLaneIds;

    typedef struct SpeedInfo{
        SpeedInfo()
        : value(0)
        , link_id(0)
        , start_offset(0)
        , end_offset(0){}

        SpeedInfo(uint8_t val, uint64_t id,  uint32_t s_offset,  uint32_t e_offset)
         : value(val)
        , link_id(id)
        , start_offset(s_offset)
        , end_offset(e_offset){}
        bool operator==(const SpeedInfo& other) const {
            return value == other.value &&
                link_id == other.link_id &&
                start_offset == other.start_offset &&
                end_offset == other.end_offset;
        }
        uint8_t value;
        uint64_t link_id;
        uint32_t start_offset;
        uint32_t end_offset;
    }SpeedInfoS;

    inline static std::map<int, std::deque<BevLaneIds>> lane_ids_map{};
    inline static int last_ego_lane_id_bev = 0;
    inline static BevLaneIds last_ego_lane_info;
    inline static std::deque<SpeedInfoS> changed_speed_links{};
    inline static std::vector<uint64_t> sd_links_for_lane_decision_loc_lane_num2 = {11008555596651, 9500012384131, 11008555596551, 11004889929017,11003008598304, 11003008598305, 9500421556275, 11005922138713, 11005922138714, 9500421556272, 9500421556274, 11003154805707, 11003154805708, 11016949144513};
    //右侧定位到2，那么右侧不可行驶
    inline static std::vector<uint64_t> sdLinks_for_lane_decision_loc_lane_num2_from_right = {11004889929016, 11004889929017, 11003008598304, 11003008598305, 9500421556275, 11005922138713, 11005922138714, 9500421556272, 9500421556274, 11003154805707, 11003154805708, 11016949144513};

    inline static std::deque<std::pair<int, EFMRefLinePoints>> last_cycle_center_lines_{};
    inline static SEhpOutputLoc last_cycle_loc_;
    inline static std::vector<uint64_t> sd_links_for_speed_limit_loc_lane_num2and1={11016949150637, 11005923573302, 11005923573303, 11005923573304 } ;

    bool Execute(const BevLaneElementGroupSet& bev_lane_group_set, const SDLaneElementGroupSet& EHP_ele_group,
                 const SEhpOutputLinkList& link_list, const LaneIdx& EgoLaneIdx, const LaneIdx& LeftLaneIdx,
                 const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx, const LaneIdx& RightRightLaneIdx,
                 const SEhpOutputLoc& loc, const LaneIdx& RefLaneIdx, const NodeInfo& node,
                 const SEhpOutputPathList& paths, LocJudgeInfo loc_res_info, datatype_efm::s_MapLane_t& map_lane);
    bool Execute(const std::vector<BevLineInnerS>& bev_lines, datatype_efm::s_MapLane_t& map_lane); //l2用   
    private:
    //L2用
    bool FillLaneSideLine(const BevLineInnerS& bev_line, datatype_efm::s_LineMkr_t& lane_line);
    bool FillCurb(const BevLineInnerS& bev_line, datatype_efm::s_CurbInfo_t &out_curb);
    uint8_t CamLineTypeToEfm(uint8_t type_in);

    //NOA用
    uint8_t LineTypeMatch(int bev_line_type);
    bool MakeSideLineTypeSinglePart(const std::vector<LineTypeInfo>& line_types, datatype_efm::s_LineMkr_t& side_line);
    bool MakeSideLineType(const LineTypeInfo& type_first, const LineTypeInfo& type_sec,
                          datatype_efm::s_LineMkr_t& side_line);
    bool MakeSideLinePoint(const EFMRefLinePointsSection& marker_section, datatype_efm::s_LineMkr_t& side_line);
    bool MakeSideLinePointBeyondEgo(const EFMRefLinePoints& marker_points,datatype_efm::s_LineMkr_t& side_line);
    bool MakeLaneSideLine(const BevLaneElement& lane_ele, datatype_efm::s_Lane_t& lane);
    bool MakeLaneCenterLine(const BevLaneElement& lane_ele, datatype_efm::s_Lane_t& lane);
    bool MakeMapLane(const BevLaneElementGroupSet& bev_lane_group_set, const SDLaneElementGroupSet& EHP_ele_group,
                     const LaneIdx& EgoLaneIdx, const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx,
                     const LaneIdx& LeftLeftLaneIdx, const LaneIdx& RightRightLaneIdx, const SEhpOutputLoc& loc,
                     const LaneIdx& RefLaneIdx, const SEhpOutputLinkList& link_list, const SEhpOutputPathList& paths,
                     datatype_efm::s_MapLane_t& map_lane);
    bool MakePosition(const SEhpOutputLoc& loc, datatype_efm::s_EfmPosition_t& position);
    bool MakeMergeSplit(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx,
                        const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx,
                        const LaneIdx& RightRightLaneIdx, datatype_efm::s_MapLane_t& map_lane);
    bool MakeOneMergeSplit(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& LaneIdx,
                           datatype_efm::s_Lane_t& lane);
    uint8_t MergeTypeMatch(uint8_t type_in);
    uint8_t SplitTypeMatch(uint8_t type_in);
    uint8_t LaneTypeMatch(uint8_t type_in);
    bool MakeOneLaneType(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& LaneIdx,
                         datatype_efm::s_Lane_t& lane);
    bool MakeLaneType(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx, const LaneIdx& LeftLaneIdx,
                      const LaneIdx& RightLaneIdx, const LaneIdx& LeftLeftLaneIdx, const LaneIdx& RightRightLaneIdx,
                      datatype_efm::s_MapLane_t& map_lane);
    uint8_t SpecialSitMatch(uint8_t type_in);
    bool MakeOdd(const SEhpOutputPath* ego_path, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeLaneDecision(const SDLaneElementGroupSet& EHP_ele_group, const LaneIdx& EgoLaneIdx,
                          const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const SEhpOutputLoc& loc, LocJudgeInfo loc_res_info,datatype_efm::s_MapLane_t& map_lane);
    bool MakeLppInfo(const SDLaneElementGroupSet& EHP_ele_group, const SEhpOutputPathList& paths,const SEhpOutputLoc& loc, const SEhpOutputLinkList& link_list,
                            const LaneIdx& EgoLaneIdx,const LaneIdx& LeftLaneIdx, const LaneIdx& RightLaneIdx, const NodeInfo& node,LocJudgeInfo loc_res_info,datatype_efm::s_MapLane_t& map_lane);
    bool MakeLaneSize(const SDLaneElementGroupSet& EHP_ele_group_set, const SEhpOutputLoc& loc,
                      const LaneIdx& EgoLaneIdx, uint8_t& driveable_lane_size, uint8_t& fixed_lane_num);
    bool MakeRampInfo(const SEhpOutputPath* ego_path, const SEhpOutputLoc& loc, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeTunnelCount(const SEhpOutputPath* ego_path, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeSpeedLimit(const SEhpOutputPath* ego_path, const SEhpOutputLinkList& link_list, const SEhpOutputLoc& loc, uint8_t loc_lane_num,double exit_start, double exit_end, 
                        datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeRoadType(const SEhpOutputLinkList& link_list, const SEhpOutputLoc& loc, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeNode(const NodeInfo& node, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeDistanceByPassMerge(const SEhpOutputPath* ego_path, datatype_efm::s_MapLppInfo_t& map_lpp_info);
    bool MakeCurvature(const SEhpOutputLinkList& link_list, const SEhpOutputPathList& paths, const SEhpOutputLoc& loc,
                       datatype_efm::s_MapLane_t& map_lane);
    bool PlotOneLane(std::string lane_name, datatype_efm::s_Lane_t& lane);
    bool InitMapLane(datatype_efm::s_MapLane_t& map_lane);
    bool InitOneLineMkr(datatype_efm::s_LineMkr_t& lane_line);
    void MakeLaneId(const BevLaneElement& bev_lane, const SEhpOutputLoc& loc, datatype_efm::s_Lane_t & lane_out);

    //中心线的固定在这里
    void PostProcess(datatype_efm::s_MapLane_t& map_lane, SEhpOutputLoc& loc);

    private:
    LocalMap* local_map_ptr_;
};

}
