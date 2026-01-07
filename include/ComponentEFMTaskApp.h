#pragma once

#include "ComponentEFMTask.h"
#include "LaneFusion.h"
#include "LaneModel.h"
#include "SdMapScene.h"
#include "LocalMap.h"
#include "WrapperInput.h"
#include "WrapperOutput.h"
#include "CommonDataType.h"
#include "Calibration.h"
#include "ConfigFile.h"
#include <stdint.h>

using namespace apollo::cyber;
using namespace efm_exe;

class ComponentEFMTaskApp final:public ComponentEFMTask{
    public:
        ComponentEFMTaskApp();
        ~ComponentEFMTaskApp() = default;
        void Init() override;
        void Release() override;
        const char* Name() override;
        int32_t compute(const datatype_perception::s_PerceptionFront_t& perception_data, 
                        const datatype_fusion::s_FusionLanes_t& fusion_lane_data, 
                        const datatype_ehp::s_SDPosition_t& sd_position_data, 
                        const datatype_ehp::s_SDPaths_t& sd_paths_data, 
                        const datatype_ehp::s_SDLinks_t& sd_links_data, 
                        datatype_efm::s_MapLane_t& output_map_lane) override;
    
    private:
    //EFM输入，BEV车道线处理，多项式转点等
    std::shared_ptr<NoMapEFM::WrapperInput> wrapper_input_;
    //前后帧线的融合
    std::shared_ptr<NoMapEFM::LaneFusion> lane_fusion_;
    //车道线的拼接，分配，延长，id固化等
    std::shared_ptr<NoMapEFM::LaneModel> lane_model_;
    //中心线生成，车道推荐
    std::shared_ptr<NoMapEFM::LocalMap> local_map_;
    //sd信息处理
    std::shared_ptr<NoMapEFM::SdMapScene> sd_map_scene_;
    //EFM输出
    std::shared_ptr<NoMapEFM::WrapperOutput> wrapper_output_;
    
    bool LoadConfig(const std::string file_name);
    void PlotBevLines(const datatype_fusion::s_FusionLanes_t& lanes_msg);
    void GetSDInfo(const datatype_ehp::s_SDPosition_t& sd_position_data, 
                   const datatype_ehp::s_SDPaths_t& sd_paths_data, const datatype_ehp::s_SDLinks_t& sd_links_data);
    bool CreateSDDateIndex();
    //保存的上周期的信息
    bool StoreInfo(const NoMapEFM::SEhpOutputLoc& lane_loc, const BevLaneElementGroup *ego_bev_lane_group);

    bool NotNOA();
    void PlotGroup();

    std::vector<BevLineInnerS> bev_lines_;//保存原始的BEV线
    BevLaneElementGroupSet bev_lane_group_set_;//车道的合集
    BevLaneElement ego_lane_, left_lane_, right_lane_; //最终的三车道模型
    NoMapEFM::IniFile config_file_;
    StoredInfo last_cycle_info_; //保存上周期的信息

    //EHP data
    NoMapEFM::SEhpOutputLinkList EHP_links_;
    NoMapEFM::SEhpOutputPathList EHP_paths_;
    NoMapEFM::SEhpOutputLoc EHP_loc_;
    SDLaneElementGroupSet EHP_ele_group_;
    MapRawDataMap sd_index_;

    std::vector<std::pair<EFMPoint,double>> path_point_body_with_offset_;//first-主path的轨迹点(自车坐标)，second-到自车位置的距离(m)
    std::vector<std::pair<int,double>> road_split_dir_vec_; //first-主path在道路分歧处的方向；1-左；2-右； second-距离自车的距离（m）;
    LocJudgeInfo loc_res_info_;
        
};

CLASS_LOADER_REGISTER_CLASS(ComponentEFMTaskApp,ComponentEFMTask)
