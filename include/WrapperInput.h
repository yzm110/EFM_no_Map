#pragma once

#include "CompileConfig.h"
#include "ComponentEFMTask.h"
#include "CommonDataType.h"
#include "EHPData.h"
#include "trans_coordinate.h"

namespace NoMapEFM{

class WrapperInput{
    public:
    bool Execute(const SEhpOutputLinkList& links, const SEhpOutputPathList& paths, const SEhpOutputLoc& lane_loc, const MapRawDataMap& sd_data_index,const datatype_fusion::s_FusionLanes_t& lanes_msg, 
                std::vector<BevLineInnerS>& bev_lines, std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset, std::vector<std::pair<int,double>>& road_split_dir_vec);
        
    private:
    void BevLinePolyToPoints(const datatype_fusion::s_FusionLanes_t& lanes_msg, std::vector<BevLineInnerS>& bev_lines);
    void CombineLines(std::vector<BevLineInnerS>& bev_lines);
    bool GetPath(uint32_t path_id, const SEhpOutputPathList& raw_paths, SEhpOutputPath& path_res);
    bool CombinePathPoint(const SEhpOutputLoc& lane_loc, const std::vector<SEhpOutputLink>& link_list, const SEhpOutputPath& path, const MapRawDataMap& data_index, 
                                    std::vector<std::pair<EFMPoint,double>>& path_point_body_with_offset);
    bool JudgeRoadSplitDir(const SEhpOutputLoc& lane_loc, const std::vector<SEhpOutputLink>& link_list,  const SEhpOutputPathList& raw_paths, const SEhpOutputPath& ego_path, const MapRawDataMap& data_index,std::vector<std::pair<int,double>>& road_split_dir_vec);
};

}