#pragma once

#include <vector>
#include <algorithm>

#include "CompileConfig.h"
// #include "common/framework.h"
// #ifdef __QNX__
// #include "gptp/gptp.h"
// #endif
#include "CommonDataType.h"
#include "EHPData.h"
#include "Calibration.h"


namespace NoMapEFM{

class SdMapScene{
    public:
    bool Execute(SEhpOutputLoc& loc, SEhpOutputPathList& paths, SEhpOutputLinkList& links, 
                 SDLaneElementGroupSet& ele_group, MapRawDataMap& sd_index);

    //返回修改后定位
    SEhpOutputLoc& GetLoc() { return loc_; }

    private:    
    // 获取自车所在的link和path，并获取自车所在的lane
    bool GetLocLane(SEhpOutputLink& loc_link);

    // 将自车所在的lane信息做成group
    bool MakeSDGroup(SEhpOutputLink& link_info, LinkLaneInfo& lane_info, SDLaneElementGroup& one_ele_group);

    // 对车道进行全局推荐判断
    bool MakeRefLane(SDLaneElementGroupSet& ele_group);

    // 将lane_type转换为LaneType
    LaneType TransLaneType(LANE_TYPE lane_type, LINK_FOW fow);

    // 将lane_change_type转换为EfmMergeType
    EfmMergeType TransMergeType(LaneChangeType lane_change_type, LinkLaneInfo& lane_info);

    // 将lane_change_type转换为EfmSplitType
    EfmSplitType TransSplitType(LaneChangeType lane_change_type, LinkLaneInfo& lane_info);

    // 获取下一车道
    bool GetNextLane(SDLaneElementGroup& one_ele_group);

    // 获取车道组的推荐车道
    bool GetRefEleForGroup(SDLaneElementGroup& one_ele_group, std::vector<int32_t>& ref_idxs);

    // 获取last_link_path_id与下一个link的连接车道
    bool GetConnectToNextLinkLanes(uint64_t last_link_path_id, uint64_t next_link_path_id, std::vector<uint8_t>& connect_lanes);

    //同一个group下面，bev_max_offset范围内，只保留一个车道组
    void MakeFilterEleGroup();
    void MakeFilterEleOneGroup(SDLaneElementGroup one_ele_group, SDLaneElementGroup& one_ele_filter_group);

    //更新link下的车道的信息
    void UpdateLinksLanes(SEhpOutputLinkList& links);

    SEhpOutputLoc loc_;
    SEhpOutputPathList paths_;
    SEhpOutputLinkList links_;
    SDLaneElementGroupSet ele_group_;
    MapRawDataMap sd_index_;
    std::vector<uint64_t> main_path_links_;
    SDLaneElementGroupSet ele_filter_group_;
};

}
