#ifndef DNP_DECISION_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define DNP_DECISION_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <zxlog/zxlog.h>

#include "fem_pos_deviation_sqp_osqp_interface.h"
#include "calibration.h"
#include "CommonDataType.h"
#include "CompileConfig.h"
#include "CommonMathMethod.h"
#include "mean_filter.h"
#include <zplot/zplot.h>

namespace earth {
namespace shell {
namespace framework {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
   public:
    typedef struct BoundSection{
        int start_index;
        int end_index;
        uint8_t scene; //0-normal;1-continuous split,3-road split; 
        BoundSection(int x, int y, uint8_t z)
        : start_index (x)
        , end_index (y)
        , scene(z){};
        ~BoundSection() = default;
    }BoundSection_s;

    ReferenceLineProvider() {}

    bool Execute(const EFMRefLinePoints& raw_point2d, std::vector<std::pair<double, double>>& opt_xy,
                 const std::vector<double>& left_boundary, const std::vector<double>& right_boundary,
                 const RefLineSmoothConfig& config, const std::vector<uint8_t>& left_bounds_type,
                 const std::vector<uint8_t>& right_bounds_type, const EFMRefLine& path, int ego_index);

   std::vector<double> opt_kappas() { return opt_kappas_; }
   std::vector<double> opt_accumulated_s() { return opt_accumulated_s_; }
    ~ReferenceLineProvider() {}

   private:
    void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    bool CalBoundsBroadenSection(const std::vector<std::pair<double, double>>& xy_points,
                                 const std::vector<double>& raw_kappas, const RefLineSmoothConfig& config,
                                 std::vector<std::pair<int, int>>& sections_v);

    double calBoundsValue(const int index, const std::vector<double>& left_boundary,
                          const std::vector<double>& right_boundary, const RefLineSmoothConfig& config);

    bool sectionPostProc(std::vector<std::pair<int, int>>& sections_v);

    double zero_x_ = 0.0;

    double zero_y_ = 0.0;

    std::vector<std::pair<int, int>> fix_section_index_v_;  // 调整平滑后参考线的边界的起始终止index
    void FixSmoothSectionBounds(std::vector<double>& bounds, const std::vector<std::pair<int, int>> smooth_index_v,
                                const std::vector<double>& left_boundary, const std::vector<double>& right_boundary,
                                const RefLineSmoothConfig& config);

    bool CalSevereBendSection(std::vector<std::pair<double, double>>& xy_points, const std::vector<double>& raw_kappas,
                              const RefLineSmoothConfig& config, std::vector<std::pair<int, int>>& sections_v);

    void FixSevereBendSectionBounds(std::vector<double>& bounds, const RefLineSmoothConfig& config,
                                    const std::vector<std::pair<int, int>> index_v);

    void CalMergeSplitSection(const EFMRefLine& path, const RefLineSmoothConfig& config, int point_num, int ego_index,
                              std::vector<BoundSection_s>& section_index_vec,std::vector<BoundSection_s> &section_index_vec_no_need_smooth);

    void FixMergeSplitSectionBounds(std::vector<double>& bounds, const RefLineSmoothConfig& config,
                                    const std::vector<BoundSection_s> index_v);

    void CalStartEndIndex(int ego_index, double tar_s, double front_s, double back_s,
                                             int &start_index, int &end_index);
    void RemoveDeviatePoints(const double limit_extend_ratio,const std::vector<double>& reference_list,
                           std::vector<double>& list);
    void SplitCurvatureCase(const SmoothSplitInfo& smooth_split_info,double& split_front_s);
    double GetDkappasSmoothExtendDist(double dkappas);
    void SplitRoadCheck(const SmoothSplitInfo& smooth_split_info, double& split_front_s);
    void CutStartEndIndex(std::vector<BoundSection_s>& section_cut);
    void SplitLineTypeCheck(const SmoothSplitInfo& smooth_split_info, const RefLineSmoothConfig &config ,double& split_front_s);
    void FixMergeSplitSectionBoundsByLineType(const std::vector<double> &left_boundary, const std::vector<double> &right_boundary,const std::vector<uint8_t>& left_bounds_type, const std::vector<uint8_t>& right_bounds_type, const std::vector<BoundSection_s> &section_index_vec,const RefLineSmoothConfig &config
                                                ,std::vector<double>& upper_bounds, std::vector<double>& lower_bounds);

    bool JudgeSplitLaneDir(const SmoothSplitInfo& smooth_split_info, uint8_t& dir);
    void FixBoundsByLaneDir(const std::vector<double> &left_boundary, const std::vector<double> &right_boundary,const std::vector<uint8_t> &left_bounds_type,const std::vector<uint8_t> &right_bounds_type, 
                            const std::vector<BoundSection_s> &index_v,const RefLineSmoothConfig &config,
                                                const std::vector<uint8_t> &lane_dir,std::vector<double>& upper_bounds, std::vector<double>& lower_bounds);
    double BoundLatch(double boundary_l, double config_bound, double to_bound_gap);

    bool StraightSplitBigSmooth(int ego_index, double split_s, const SmoothSplitInfo& smooth_split_info,int point_num,BoundSection_s& big_smooth_section);
    std::vector<uint8_t>  split_lane_dir_;//1- split的左车道，2-split的右车道
    std::vector<BoundSection_s> split_lane_section_; //和split_lane_dir_配套使用，用于硬约束
    std::vector<double> opt_kappas_, opt_accumulated_s_;                                  
};

}  // namespace framework
}  // namespace shell
}  // namespace earth
#endif
