#pragma once

#include "CompileConfig.h"
// #include "common/framework.h"
// #ifdef __QNX__
// #include "gptp/gptp.h"
// #endif
#include "ComponentEFMTask.h"
#include "CommonDataType.h"
#include "datatype_ehp/impl_type_s_sdposition_t.h"
#include "Calibration.h"

namespace NoMapEFM{

class LaneFusion{
    public:
    bool Execute(std::vector<BevLineInnerS>& bev_lines, const datatype_ehp::s_SDPosition_t& position, const datatype_fusion::s_FusionLanes_t& lanes_msg);
        
    private:
    //c1236
    void lane_fusion_(std::vector<BevLineInnerS>& bev_lines_, const datatype_ehp::s_SDPosition_t& position_msg, const datatype_fusion::s_FusionLanes_t& lanes_msg);
    std::vector<BevLineInnerS> data_base_zombie_lines_;
    std::vector<BevLineInnerS> prev_frame_active_lines_;//不包括僵尸轨迹的上帧的，处理之后的线（关联及融合滤波处理）
    BevLineInnerS extended_center_line_lastcycle;//存放路口的历史中心线
    BevLineInnerS bev_line_left_lastcycle;  // 左侧当前车道线的上个周期值
    BevLineInnerS bev_line_right_lastcycle; // 右侧当前车道线的上个周期值
    BevLineInnerS left_topology_line_lastcycle;  // 左侧拓扑车道线的上个周期值
    BevLineInnerS right_topology_line_lastcycle; // 右侧拓扑车道线的上个周期值
    void LF_LineBodyToWGS84(BevLineInnerS& line_, const DoublePosePoint& ego_pos_wgs84);
    void LF_LineWGS84ToBody(BevLineInnerS& line_, const DoublePosePoint& ego_pos_wgs84);
    BevLineInnerS LF_ConnectIntersectionLine(const BevLineInnerS& line_location, const BevLineInnerS& line_ahead, const DoublePosePoint& ego_pos_wgs84);
    bool LaneFusionZombieLineAssociation(std::vector<BevLineInnerS>& data_base_Associated_lines_, std::vector<BevLineInnerS>& data_base_zombie_lines_, const BevLineInnerS& prev_line_);
    bool LaneFusionCurrentLineAssociation(const BevLineInnerS& prev_line_, const BevLineInnerS& Bev_Lane_, const std::vector<int>& event_line_id_vec_);
    bool LaneFusionCloseLineAssociation(std::vector<BevLineInnerS>& data_base_Associated_lines_, std::vector<BevLineInnerS>& new_lines_, const BevLineInnerS& prev_line_);
    void LaneFusionLineFusion(std::vector<BevLineInnerS>& bev_lines_, std::vector<BevLineInnerS>& data_base_zombie_lines_, std::vector<BevLineInnerS>& prev_frame_active_lines_, const DoublePosePoint& ego_pos_wgs84_, const std::vector<int>& event_line_id_vec_);
    void LaneFusionPointsConvert(BevLineInnerS& line);
    void LaneFusionMeanProcess(BevLineInnerS& line);//处理量测值
    void LaneFusionVarProcess(BevLineInnerS& line);//处理量测方差
    void Kalman_filter_init_OneDim( KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s, const SCALAR_STATE_STRUCT* const p_initial_state_s );
    void Kalman_filter_prediction_OneDim( KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s, const float process_variance_f32 );
    void Kalman_filter_update_OneDim(KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s, const KALMAN_MODEL_ONEDIM_STRUCT* const p_meas_feature_s);
    void LaneFusionNoneOverlapProcess(const int start_overlap_point_index,
                                                const float delta_start_point_pre,
                                                const float delta_start_point_cur,
                                                const int end_overlap_point_index,
                                                const float delta_end_point_pre,
                                                const float delta_end_point_cur,
                                                BevLineInnerS& prev_line,
                                                const BevLineInnerS& current_line);
    //十字路口
    void intersection_topology_(std::vector<BevLineInnerS>& bev_lines_, const DoublePosePoint& position_msg);
    EFMRefLinePoints IT_calculateExtendedCenterLine(const LaneFusionPointsType& left_points, const LaneFusionPointsType& right_points, float extension_length);

    //路沿外车道线删除
    void inhibit_out_road_edge_line(std::vector<BevLineInnerS>& bev_lines_);
};

}
