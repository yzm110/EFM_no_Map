#include "reference_line_provider.h"

// #include <gflags/gflags.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace earth {
namespace shell {
namespace framework {

bool ReferenceLineProvider::Execute(const EFMRefLinePoints &raw_point2d, std::vector<std::pair<double, double>> &opt_xy,
                                    const std::vector<double> &left_boundary, const std::vector<double> &right_boundary,
                                    const RefLineSmoothConfig &config, const std::vector<uint8_t> &left_bounds_type,
                                    const std::vector<uint8_t> &right_bounds_type, const EFMRefLine &path,
                                    int ego_index) {
    if (raw_point2d.size() == 0) {
        return false;
    }
    std::vector<std::pair<double, double>> raw_point2d_valid{};
    for (auto point : raw_point2d) {
        raw_point2d_valid.push_back(std::make_pair(point.x, point.y));
    }
    
    NormalizePoints(&raw_point2d_valid);
    std::vector<BoundSection_s> merge_split_section_vec{};
    std::vector<BoundSection_s> split_section_no_need_smooth{};
    // std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
    // CommonMathMethod::DiscretePointsMath::ComputePathProfile(raw_point2d, &raw_headings, &raw_accumulated_s,
    //                                                          &raw_kappas, &raw_dkappas);

    CalMergeSplitSection(path, config, raw_point2d.size(), ego_index, merge_split_section_vec, split_section_no_need_smooth);
    std::vector<double> upper_bounds(raw_point2d_valid.size(), config.bounds_val);
    std::vector<double> lower_bounds(raw_point2d_valid.size(), config.bounds_val);
    std::vector<double> bounds(raw_point2d_valid.size(), config.bounds_val);
    std::vector<double> opt_x;
    std::vector<double> opt_y;
    //拓宽
    FixMergeSplitSectionBounds(bounds, config, merge_split_section_vec);
    //d对拓宽后的再修正
    //FixMergeSplitSectionBounds(bounds, config, split_section_no_need_smooth);
    upper_bounds = bounds;
    lower_bounds = bounds;
    FixMergeSplitSectionBoundsByLineType(left_boundary,right_boundary, left_bounds_type, right_bounds_type,merge_split_section_vec, config,upper_bounds, lower_bounds);
#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << ","<<"bounds: ";
    for(int i=0;i<bounds.size();i++ ){
        std::cout<<"| "<<i<<", "<<bounds[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<"left_bounds_type: ";
    for(int i=0;i<left_bounds_type.size();i++ ){
        std::cout<<"| "<<i<<", "<<(int)left_bounds_type[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<"upper_bounds: ";
    for(int i=0;i<upper_bounds.size();i++ ){
        std::cout<<"| "<<i<<", "<<upper_bounds[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<"right_bounds_type: ";
    for(int i=0;i<right_bounds_type.size();i++ ){
        std::cout<<"| "<<i<<", "<<(int)right_bounds_type[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<"lower_bounds: ";
    for(int i=0;i<lower_bounds.size();i++ ){
        std::cout<<"| "<<i<<", "<<lower_bounds[i]<<" ";
    }
    std::cout<<std::endl;
#endif
    FixBoundsByLaneDir(left_boundary, right_boundary, left_bounds_type,right_bounds_type, split_lane_section_,config,split_lane_dir_, upper_bounds, lower_bounds);
#ifdef SMOOTH
    std::cout<< "split_lane_section_.size(): "<<merge_split_section_vec.size()<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " split_lane_section_: ";
    for (auto pair : split_lane_section_) {
        std::cout << " ,<" << pair.start_index << " ," << pair.end_index <<" ,"<<(int)pair.scene<< ">, ";
    }
    std::cout <<std::endl;
    std::cout<< "split_lane_dir_.size(): "<<split_lane_dir_.size()<<std::endl;
    std::cout<<"split_lane_dir_: ";
    for(int i=0;i<split_lane_dir_.size();i++ ){
        std::cout<<", "<<(int)split_lane_dir_[i];
    }  
    std::cout<<std::endl;  
    std::cout << __FILE__ << "," << __LINE__ << ","<<"upper_bounds final: ";
    for(int i=0;i<upper_bounds.size();i++ ){
        std::cout<<"| "<<i<<", "<<upper_bounds[i]<<" ";
    }
    std::cout<<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<<"lower_bounds final: ";
    for(int i=0;i<lower_bounds.size();i++ ){
        std::cout<<"| "<<i<<", "<<lower_bounds[i]<<" ";
    }
    std::cout<<std::endl;    
#endif    
    // LOGI("config.bounds_val:: {}", config.bounds_val);
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " bounds.size(): " << bounds.size() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " config.bounds_val: " << config.bounds_val << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " raw_point2d_valid.size(): " << raw_point2d_valid.size() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " config.curvature_constraint: " << config.curvature_constraint << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " config.weight_fem_pos_deviation: " << config.weight_fem_pos_deviation << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " config.weight_path_length: " << config.weight_path_length << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " config.weight_ref_deviation: " << config.weight_ref_deviation << std::endl;

    FemPosDeviationSqpOsqpInterface solver;
    solver.set_ref_points(raw_point2d_valid);
    solver.set_bounds_around_refs(upper_bounds, lower_bounds);
    solver.set_curvature_constraint(config.curvature_constraint);
    solver.set_weight_fem_pos_deviation(config.weight_fem_pos_deviation);
    solver.set_weight_path_length(config.weight_path_length);
    solver.set_weight_ref_deviation(config.weight_ref_deviation);

    if (!solver.Solve()) {
        std::cout << __FILE__ << "," << __LINE__ << "," << " solver failure: " << std::endl;
        LOGE("solver failure!!!");
        return false;
    }

    // const std::vector<std::pair<double, double>>& opt_xy_t = solver.opt_xy();
    // for (int i = 0; i < opt_xy_t.size(); i++) {
    //   opt_xy.emplace_back(opt_xy_t[i]);
    // }

    opt_xy = solver.opt_xy();
    DeNormalizePoints(&opt_xy);
    // TODO(Jinyun): unify output data container
    opt_x.resize(opt_xy.size());
    opt_y.resize(opt_xy.size());
    for (size_t i = 0; i < opt_xy.size(); ++i) {
        opt_x[i] = opt_xy[i].first;
        opt_y[i] = opt_xy[i].second;
    }

    // std::cout << "OSQP END!!!" << std::endl;
    // plot
    EFMRefLinePoints xy_points{};
    // std::cout << "ego_index: " << ego_index << std::endl;
    for (int i = ego_index; i < opt_xy.size() && ego_index < opt_xy.size(); i++) {
        xy_points.push_back({opt_xy[i].first, opt_xy[i].second});
    }
    std::vector<double> opt_accumulated_s, opt_kappas, opt_dkappas, opt_headings;
    CommonMathMethod::DiscretePointsMath::ComputePathProfile(xy_points, &opt_headings, &opt_accumulated_s, &opt_kappas,
                                                             &opt_dkappas);
    
    std::vector<double> opt_kappas_temp{}, opt_kappas_result{}, opt_kappas_input{};
    opt_kappas_input = opt_kappas;
    opt_kappas_temp.clear();
    opt_kappas_result.clear();
    opt_kappas_temp.resize(opt_kappas.size(), 0.0);
    opt_kappas_result.resize(opt_kappas.size(), 0.0);

    if(opt_kappas_input.size() > 3){
        DX11_NOA::MeanFilter mean_filter(p_mean_filter_window_size);
        mean_filter.FilterList(opt_kappas_input, false, opt_kappas_temp);
        RemoveDeviatePoints(p_limit_extend_ratio, opt_kappas_temp,opt_kappas_input);
        mean_filter.set_window_size(p_mean_filter_window_size);
        mean_filter.FilterList(opt_kappas_input, false, opt_kappas_result);
    }
    
    opt_kappas_ = opt_kappas_result;
    opt_accumulated_s_ = opt_accumulated_s;

    // ZPLOTF("curvpoint0", "-blue2", opt_kappas[0]);
    // ZPLOTF("curvpoint0", "-green2",opt_kappas_result[0]);
    // ZPLOTF("curvpoint2", "-blue2", opt_kappas[2]);
    // ZPLOTF("curvpoint2", "-green2", opt_kappas_result[2]);
    // ZPLOTF("curvpoint4", "-blue2", opt_kappas[4]);
    // ZPLOTF("curvpoint4", "-green2", opt_kappas_result[4]);
    // ZPLOTF("curvpoint6", "-blue2", opt_kappas[6]);
    // ZPLOTF("curvpoint6", "-green2",opt_kappas_result[6]);
    // ZPLOTF("curvpoint8", "-blue2", opt_kappas[8]);
    // ZPLOTF("curvpoint8", "-green2", opt_kappas_result[8]);
    // ZPLOTF("curvpoint10", "-blue2", opt_kappas[10]);
    // ZPLOTF("curvpoint10", "-green2", opt_kappas_result[10]);
    // ZPLOTF("curvpoint12", "-blue2", opt_kappas[12]);
    // ZPLOTF("curvpoint12", "-green2", opt_kappas_result[12]);
    // ZPLOTF("curvpoint20", "-blue2", opt_kappas[20]);
    // ZPLOTF("curvpoint20", "-green2", opt_kappas_result[20]);
    // ZPLOTF("curvpoint30", "-blue2", opt_kappas[30]);
    // ZPLOTF("curvpoint30", "-green2", opt_kappas_result[30]);
    return true;
}

void ReferenceLineProvider::NormalizePoints(std::vector<std::pair<double, double>> *xy_points) {
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point) {
        point.first = point.first - zero_x_;
        point.second = point.second - zero_y_;
    });
}

void ReferenceLineProvider::DeNormalizePoints(std::vector<std::pair<double, double>> *xy_points) {
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point) {
        point.first = point.first + zero_x_;
        point.second = point.second + zero_y_;
    });
}

/**
 * @brief 根据平滑之前点的曲率变化率
 * 判断参考线的哪些区间需要扩宽平滑的边界约束,针对S线型
 *
 * @return true
 * @return false
 */
bool ReferenceLineProvider::CalBoundsBroadenSection(const std::vector<std::pair<double, double>> &xy_points,
                                                    const std::vector<double> &raw_kappas,
                                                    const RefLineSmoothConfig &config,
                                                    std::vector<std::pair<int, int>> &sections_v) {
    sections_v.clear();
    int start_index = 0;
    int end_index = 0;
    bool find_start_index_f = false;
    // bool find_end_index_f = false;
    double p_KappasUpperLimt = config.s_shape_kappa_up_limt;    // 0.032;
    double p_KappasLowerLimt = config.s_shape_kappa_down_limt;  // 0.03;
    int p_front_index = int(config.s_shape_s_front / 2.5);      // 55
    int p_back_index = int(config.s_shape_s_back / 2.5);        // 30
    if (xy_points.size() != raw_kappas.size()) {
        return false;
    }
    for (int i = 0; i < raw_kappas.size(); i++) {
        if (abs(raw_kappas[i]) > p_KappasUpperLimt && find_start_index_f == false) {
            // DNP_LOG_INFO( "111";
            // DNP_LOG_INFO("i = {}", i);
            start_index = i;  // MAX((i - 10), 0);  //往前取5m作为起始点
            find_start_index_f = true;
            // find_end_index_f = false;
            end_index = 0;
            continue;
        }
        if (find_start_index_f == true) {
            // 如果找到了一个曲率变化大的点
            // 向后再找20m,即40个点,点数不够的取到最后一个，
            // 看曲率是否又反向变化了
            for (int j = 0; j < c_min(p_front_index, raw_kappas.size() - i); j++) {
                // DNP_LOG_INFO( "222";
                // DNP_LOG_INFO("i+j = {}", i + j);
                if (abs(raw_kappas[i + j]) > p_KappasLowerLimt && raw_kappas[i + j] * raw_kappas[start_index] < 0) {
                    // DNP_LOG_INFO("333");
                    // find_end_index_f = true;
                    end_index = i + j + p_front_index;  // 向后延伸5m
                    sections_v.push_back(std::make_pair(c_max(start_index - p_back_index, 0), end_index));
                    i += j;
                    find_start_index_f = false;
                    break;
                } else if (j == p_front_index - 1 || (j == raw_kappas.size() - i)) {
                    // 向后找20m都没有再满足条件，或者到了参考线最后一个点
                    // DNP_LOG_INFO("444");
                    i += j;
                    find_start_index_f = false;
                    // find_end_index_f = false;
                    break;
                }
            }
        }
    }
    // DNP_LOG_INFO("end CalBoundsBroadenSection ";
    // DNP_LOG_INFO("smooth_index_v.size() before stich {}", sections_v.size());
    // std::stringstream ss1;
    // ss1 << "\n"
    //     << "smooth_index_v  before stich :";
    // for (int i = 0; i < sections_v.size(); i++) {
    //   ss1 << "[" << i << "].first :" << sections_v[i].first << ", "
    //       << " .second : " << sections_v[i].second << "  ";
    // }
    // DNP_LOG_INFO(ss1.str());
    sectionPostProc(sections_v);
    // DNP_LOG_INFO("smooth_index_v.size() {}", sections_v.size());
    // std::stringstream ss2;
    // ss2 << "\n"
    //     << "smooth_index_v :";
    // for (int i = 0; i < sections_v.size(); i++) {
    //   ss2 << "[" << i << "].first :" << sections_v[i].first << ", "
    //       << " .second : " << sections_v[i].second;
    // }
    // ss2 << std::endl;
    // DNP_LOG_INFO(ss2.str());
    return true;
}

double ReferenceLineProvider::calBoundsValue(const int index, const std::vector<double> &left_boundary,
                                             const std::vector<double> &right_boundary,
                                             const RefLineSmoothConfig &config) {
    // 该方程用于计算需要填充的边界值，超宽车道场景
    if (index >= left_boundary.size() || index >= right_boundary.size()) {
        // 防溢出
        return 0.5;
    } else if (abs(left_boundary[index] + right_boundary[index]) > 2) {
        return abs(left_boundary[index] + right_boundary[index]) * 0.5;
    } else {
        return config.s_shape_bounds_val;
    }
}

bool ReferenceLineProvider::sectionPostProc(std::vector<std::pair<int, int>> &sections_v) {
    // 将首尾index有交集的section拼接成一段
    if (sections_v.size() < 2) {
        return false;
    }
    std::vector<std::pair<int, int>> sections_v_temp;
    int cur_start_index = -1;
    int cur_end_index = -1;
    int last_start_index = -1;
    int last_end_index = -1;
    int very_first_start_index = -1;
    sections_v_temp.emplace_back(std::make_pair(sections_v[0].first, sections_v[0].second));
    for (int i = 1; i < sections_v.size(); i++) {
        // DNP_LOG_INFO( "i " << i;
        int cur_section_index = static_cast<int>(sections_v_temp.size()) - 1;
        // DNP_LOG_INFO( "cur_section_index " << cur_section_index;
        if (sections_v_temp[cur_section_index].second >= sections_v[i].first) {
            // DNP_LOG_INFO("111");
            if (sections_v_temp[cur_section_index].second >= sections_v[i].second) {
                // 大于等于下一段的end index，那么直接考虑下一段
                //  DNP_LOG_INFO("222");
                continue;
            } else {
                sections_v_temp[cur_section_index].second = sections_v[i].second;
                // DNP_LOG_INFO("333");
                // DNP_LOG_INFO(sections_v_temp[cur_section_index].second);
            }
        } else {
            // 两段不交叉，保存下来
            //  DNP_LOG_INFO("444");
            sections_v_temp.emplace_back(std::make_pair(sections_v[i].first, sections_v[i].second));
        }
    }
    sections_v.clear();
    sections_v.insert(sections_v.begin(), sections_v_temp.begin(), sections_v_temp.end());
    return true;
}

void ReferenceLineProvider::FixSmoothSectionBounds(std::vector<double> &bounds,
                                                   const std::vector<std::pair<int, int>> smooth_index_v,
                                                   const std::vector<double> &left_boundary,
                                                   const std::vector<double> &right_boundary,
                                                   const RefLineSmoothConfig &config) {
    double val = 0;
    for (auto &iter : smooth_index_v) {
        for (int i = iter.first; i <= iter.second && i < bounds.size(); i++) {
            val = calBoundsValue(i, left_boundary, right_boundary, config);
            bounds[i] = val;
        }
    }
}

bool ReferenceLineProvider::CalSevereBendSection(std::vector<std::pair<double, double>> &xy_points,
                                                 const std::vector<double> &raw_kappas,
                                                 const RefLineSmoothConfig &config,
                                                 std::vector<std::pair<int, int>> &sections_v) {
    // DNP_LOG_INFO("CalSevereBendSection");
    sections_v.clear();
    int start_index = 0;
    int end_index = 0;
    bool find_start_index_f = false;
    // bool find_end_index_f = false;
    double p_KappasUpperLimt = config.severe_bend_kappa_up_limt;    // 0.032;
    double p_KappasLowerLimt = config.severe_bend_kappa_down_limt;  // 0.015;
    int p_front_index = int(config.severe_bend_s_front / 2.5);      // 16
    int p_back_index = int(config.severe_bend_s_back / 2.5);        // 16
    // raw_kappas.size() should equal to raw_dkappas.size() and xy_points.size()
    if (xy_points.size() != raw_kappas.size()) {
        return false;
    }
    for (int i = 0; i < raw_kappas.size(); i++) {
        if (abs(raw_kappas[i]) > p_KappasUpperLimt && find_start_index_f == false) {
            // DNP_LOG_INFO( "111";
            // DNP_LOG_INFO("i = {}", i);
            start_index = i;  // MAX((i - 10), 0);  //往前取5m作为起始点
            find_start_index_f = true;
            // find_end_index_f = false;
            end_index = 0;
            continue;
        }
        if (find_start_index_f == true) {
            // 如果找到了一个曲率变化大的点
            // 向后再找10m,即20个点,点数不够的取到最后一个，
            // 看曲率是否都比较小
            for (int j = 0; j < c_min(p_front_index, raw_kappas.size() - i); j++) {
                // DNP_LOG_INFO( "222";
                // DNP_LOG_INFO("i+j = {}", i + j);
                if (abs(raw_kappas[i + j]) > p_KappasLowerLimt) {
                    // DNP_LOG_INFO("333");
                    i += j;
                    // 如果两点间距离较近，可以再往前评估下
                    if (i - start_index <= (p_front_index * 0.5)) {
                        // DNP_LOG_INFO("i= {}  continue", i);
                        continue;
                    }
                    find_start_index_f = false;
                    break;
                } else if (j == p_front_index - 1 || (j == raw_kappas.size() - i)) {
                    // 向后找20m都是比较直的情况，或者到了参考线最后一个点
                    // DNP_LOG_INFO("444");
                    i += j;
                    find_start_index_f = false;
                    end_index = c_min(i + j + p_front_index, static_cast<int>(raw_kappas.size()) - 1);  // 向后延伸8m
                    sections_v.push_back(std::make_pair(c_max(start_index - p_back_index, 0), end_index));
                    // find_end_index_f = false;
                    break;
                }
            }
        }
    }
    sectionPostProc(sections_v);
    return true;
}

void ReferenceLineProvider::FixSevereBendSectionBounds(std::vector<double> &bounds, const RefLineSmoothConfig &config,
                                                       const std::vector<std::pair<int, int>> index_v) {
    for (auto &iter : index_v) {
        for (int i = iter.first; i <= iter.second && i < bounds.size(); i++) {
            bounds[i] = config.severe_bend_bounds_val;
        }
    }
}

void ReferenceLineProvider::CalMergeSplitSection(const EFMRefLine &path, const RefLineSmoothConfig &config,
                                                 int point_num, int ego_index,
                                                 std::vector<BoundSection_s> &section_index_vec,
                                                 std::vector<BoundSection_s> &section_index_vec_no_need_smooth) {
    // merge
    section_index_vec.clear();
    section_index_vec_no_need_smooth.clear();
    if (point_num <= 0) {
        return;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " point_num: " << point_num << std::endl;
    for (auto merge_s : path.merge_end_s_vec) {
        if (merge_s.first > 150) {
            continue;
        }
        int start_index = 0;
        int end_index = 0;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " merge_s: " << merge_s << std::endl;
        //merge需要考虑延长merge点前的距离，至少到link的长度
        double final_merge_back_s = std::max(config.p_merge_back_s, merge_s.second);
        CalStartEndIndex(ego_index, merge_s.first, config.p_merge_front_s, final_merge_back_s, start_index, end_index);
        start_index = std::max(0, start_index);
        start_index = std::min(point_num - 1, start_index);
        end_index = std::max(0, end_index);
        uint8_t merge_scene = 8;
        if(end_index>=(point_num - 1)){
            merge_scene = 18;//表示线不够宽，需要现在拓宽的end点，防止线跳
        }
        end_index = std::min(point_num - 1, end_index);
        BoundSection_s section_tmp(start_index, end_index,merge_scene);
        section_index_vec.push_back(section_tmp);
        // section_index_vec.push_back(std::make_pair(start_index, end_index));
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << " section_index_vec.back().start_index: " << section_index_vec.back().start_index
        //           << " section_index_vec.back().end_index: " << section_index_vec.back().end_index 
        //           << "section_index_vec.back().scene:"<<(int)section_index_vec.back().scene<< std::endl;
    }

    // split
    split_lane_dir_.clear();
    split_lane_section_.clear();
    for (int i = 0; i<path.smooth_split_info.size(); i++) {
        double split_s = path.smooth_split_info[i].dist;
        std::vector<double> link_length = path.smooth_split_info[i].link_length;
        if(path.smooth_split_info[i].link_length.empty() || path.smooth_split_info[i].before_link_length.empty()){
            continue;
        }
        if (split_s > 150) {
            continue;
        }
        int start_index = 0;
        int end_index = 0;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " split_s: " << split_s << std::endl;
        int scene = 0;//0-normal;2-dkappas;3-road split
        double final_split_front_s = std::max(config.p_split_front_s, link_length.front()); 
        // std::cout << __FILE__ << "," << __LINE__ << "," << "before continu split, final_split_front_s: " << final_split_front_s << std::endl;  
        uint8_t lane_dir = 0;
        JudgeSplitLaneDir(path.smooth_split_info[i], lane_dir);
        split_lane_dir_.push_back(lane_dir);
        // 短距离连续split
        double contnu_split_front_s = 0;
        if((i +1) < path.smooth_split_info.size()){
            double dist_tmp = path.smooth_split_info[i+1].dist - split_s;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "dist_tmp: " << dist_tmp << std::endl; 
            //std::cout << __FILE__ << "," << __LINE__ << "," << "config.p_continuous_split_length_s: " << config.p_continuous_split_length_s << std::endl; 
            if(dist_tmp < config.p_continuous_split_length_s){
                //scene = 1;
                contnu_split_front_s = std::max(final_split_front_s, dist_tmp); 
                if(dist_tmp < config.p_short_continuous_split_length_s){
                    contnu_split_front_s = std::max(config.p_short_continuous_split_front_s, dist_tmp); 
                    //1125新增，
                    CalStartEndIndex(ego_index, split_s, contnu_split_front_s, 0, start_index, end_index);
                    start_index = std::max(0, start_index);
                    start_index = std::min(point_num - 1, start_index);
                    end_index = std::max(0, end_index);
                    end_index = std::min(point_num - 1, end_index);
                    BoundSection_s section_tmp_c_s(start_index, end_index,13);
                    section_index_vec.push_back(section_tmp_c_s);
        
                }
            }
        }
        //std::cout << __FILE__ << "," << __LINE__ << "," << "contnu_split_front_s: " << contnu_split_front_s << std::endl;
        //front 考虑车道宽度
        double front_s_consider_width = 0;
        for(int width_index =0;width_index<path.smooth_split_info[i].max_width.size() && width_index<path.smooth_split_info[i].min_width.size() 
                    && width_index<path.smooth_split_info[i].link_length.size();width_index++){
            if(path.smooth_split_info[i].max_width[width_index] >= 430 ){
                front_s_consider_width +=path.smooth_split_info[i].link_length[width_index];
            }else{
                break;
            }
        }
        // std::cout << __FILE__ << "," << __LINE__ << "," << "front_s_consider_width: " << front_s_consider_width << std::endl;
        final_split_front_s = std::max(final_split_front_s, front_s_consider_width);
        final_split_front_s = std::min(final_split_front_s, config.p_split_front_s);
        //考虑split back的长度，如果车道宽度大，那么加长
        double back_s = config.p_split_back_s;
        for(int width_index =0;width_index<path.smooth_split_info[i].before_split_max_width.size() && width_index<path.smooth_split_info[i].before_link_length.size()
                    && width_index<path.smooth_split_info[i].before_split_min_width.size();width_index++){
            if(path.smooth_split_info[i].before_split_max_width[width_index] >= 430 ){
                double max_w = path.smooth_split_info[i].before_split_max_width[width_index];
                double min_w = path.smooth_split_info[i].before_split_min_width[width_index];
                double link_length = path.smooth_split_info[i].before_link_length[width_index];
                if(min_w>200&& min_w<400 && link_length>1){
                    double tmp = ((max_w -400 )/(max_w-min_w)) * link_length;
                    // std::cout << __FILE__ << "," << __LINE__ << "," << "tmp : " << tmp << std::endl;
                    back_s += tmp;
                }else{
                    back_s +=path.smooth_split_info[i].before_link_length[width_index];
                }
                
            }else{
                break;
            }
        }
        back_s = std::max(config.p_split_back_s, back_s);
        back_s = std::min(25.0,back_s);
        // std::cout << __FILE__ << "," << __LINE__ << "," << "before SplitCurvatureCase final_split_front_s: " << final_split_front_s << std::endl; 
        //front 考虑曲率
        double split_front_s_dkappas = 0;
        SplitCurvatureCase(path.smooth_split_info[i],split_front_s_dkappas);
        final_split_front_s = std::max(split_front_s_dkappas,final_split_front_s);
        // std::cout << __FILE__ << "," << __LINE__ << "," << "split_front_s_dkappas: " << split_front_s_dkappas << std::endl; 
        BoundSection_s big_smooth_section(0, 0,99);
        if(StraightSplitBigSmooth(ego_index, split_s, path.smooth_split_info[i],point_num, big_smooth_section) && path.smooth_split_info[i].road_class == 1){
            section_index_vec.push_back(big_smooth_section);
        }
        //front 考虑线型和道路分歧
        double split_front_s_road = 0;
        SplitRoadCheck(path.smooth_split_info[i], split_front_s_road);
        // std::cout << __FILE__ << "," << __LINE__ << "," << "split_front_s_line_road: " << split_front_s_line_road << std::endl;  
        if(split_front_s_road>1){
            scene = 3;//road split硬约束
            back_s = 3;
            final_split_front_s = std::min(final_split_front_s,split_front_s_road);
        }
        // std::cout << __FILE__ << "," << __LINE__ << "," << " final_split_front_s: " << final_split_front_s << std::endl;    
        // std::cout << __FILE__ << "," << __LINE__ << "," << " back_s: " << back_s << std::endl;         
        CalStartEndIndex(ego_index, split_s, final_split_front_s, back_s, start_index, end_index);
        start_index = std::max(0, start_index);
        start_index = std::min(point_num - 1, start_index);
        end_index = std::max(0, end_index);
        end_index = std::min(point_num - 1, end_index);
        BoundSection_s section_tmp(start_index, end_index,scene);
        section_index_vec.push_back(section_tmp);
        //1111 new add, split方向硬约束， scene ==11，12, 如果是连续split,不要搞硬约束
        CalStartEndIndex(ego_index, split_s, final_split_front_s, 0, start_index, end_index);
        start_index = std::max(0, start_index);
        start_index = std::min(point_num - 1, start_index);
        end_index = std::max(0, end_index);
        end_index = std::min(point_num - 1, end_index);
        BoundSection_s section_tmp_split_dir(start_index, end_index,5);
        if(contnu_split_front_s>0.5){
            section_tmp_split_dir.scene =11;
        }else{
            //对于连续split的第二个split, 也需要硬约束
            if(split_lane_section_.size() > 0){
                if(split_lane_section_.back().scene ==11){
                    section_tmp_split_dir.scene =12;//用于表示之后没有连续split，连续split到这里为止
                }
            }
        }
        split_lane_section_.push_back(section_tmp_split_dir);
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << " section_index_vec.back().start_index: " << section_index_vec.back().start_index
        //           << " section_index_vec.back().end_index: " << section_index_vec.back().end_index 
        //           << "section_index_vec.back().scene:"<<(int)section_index_vec.back().scene<< std::endl;
        if(contnu_split_front_s>0.5){
            int continu_split_end_index = 0;
            int continu_split_start_index = 0;
            CalStartEndIndex(ego_index, split_s, contnu_split_front_s, 0, continu_split_start_index, continu_split_end_index); 
            BoundSection_s section_tmp2(continu_split_start_index, continu_split_end_index,1);
            section_index_vec.push_back(section_tmp2);          
        }

        //考虑大曲率下的线型，抑制平滑
        double line_type_front_s_no_need_smooth = 0;
        SplitLineTypeCheck(path.smooth_split_info[i],config,line_type_front_s_no_need_smooth);
        // std::cout << __FILE__ << "," << __LINE__ << "," << " line_type_front_s_no_need_smooth: " << line_type_front_s_no_need_smooth << std::endl; 
        if(line_type_front_s_no_need_smooth>0.5){
            int end_index = 0;
            int start_index = 0;
            CalStartEndIndex(ego_index, split_s, line_type_front_s_no_need_smooth, 3, start_index, end_index); 
            BoundSection_s section_tmp2(start_index, end_index,4);
            section_index_vec_no_need_smooth.push_back(section_tmp2);              
        }
        

    }
    //对于道路的硬约束，需要把远于硬约束的拓宽点限制到硬约束处

#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << "," << " section_index_vec: ";
    for (auto pair : section_index_vec) {
        std::cout << " ,<" << pair.start_index << " ," << pair.end_index <<" ,"<<(int)pair.scene<< ">, ";
    }
    std::cout <<std::endl;
#endif
    CutStartEndIndex(section_index_vec);
    //0307 DX11-67771
    //如果merge和split的bound连续，那么要把他们断开
    for(int i = 1; i < section_index_vec.size();i++ ){
        if(section_index_vec[i-1].scene == 8 && section_index_vec[i].scene == 0){
            if(section_index_vec[i-1].end_index+1 ==section_index_vec[i].start_index){
                // std::cout << __FILE__ << "," << __LINE__ << "," << "ininininin: "<<std::endl;
                section_index_vec[i-1].end_index = std::max(section_index_vec[i-1].start_index, section_index_vec[i-1].end_index -1);
            }else if(section_index_vec[i-1].end_index == section_index_vec[i].start_index){
                section_index_vec[i-1].end_index = std::max(section_index_vec[i-1].start_index, section_index_vec[i-1].end_index -2);
            }
        }
    }
#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << "," << "final section_index_vec: ";
    for (auto pair : section_index_vec) {
        std::cout << " ,<" << pair.start_index << " ," << pair.end_index <<" ,"<<(int)pair.scene<< ">, ";
    }
    std::cout <<std::endl;
#endif
#ifdef SMOOTH
    std::cout << __FILE__ << "," << __LINE__ << "," << "section_index_vec_no_need_smooth: ";
    for (auto pair : section_index_vec_no_need_smooth) {
        std::cout << " ,<" << pair.start_index << " ," << pair.end_index <<" ,"<<(int)pair.scene<< ">, ";
    }
    std::cout <<std::endl;
#endif
    return;
}

void ReferenceLineProvider::SplitCurvatureCase(const SmoothSplitInfo& smooth_split_info,double& split_front_s){
    //step 1， 计算split后第一段link的曲率变化率，如果大于一定值，那么延长extend的距离
    // 对split处第一段link,曲率变化率大于p_split_dkappas_end的，则查表，split_front_s = 点的s+查表结果； 小于p_split_dkappas_end的不退，split_front_s保持p_split_front_s
    // 对split处的第二段以及之后的link, 曲率变化率大于p_split_dkappas_end的，则查表，split_front_s = 点的s+查表结果； 小于p_split_dkappas_end的退
    split_front_s = 0;
    double dkappas = 0;
    double start_s = 0;//split 起点距离自车的s
    double p_split_dkappas_end = 0.0008;
    if(smooth_split_info.lane_curvpoints.size()>0){
        if(smooth_split_info.lane_curvpoints.front().size()>0){
            start_s = smooth_split_info.lane_curvpoints.front().front().first;
        }else{
            return;
        }
    }else{
        return;
    }
    //std::cout << __FILE__ << "," << __LINE__ << "," << " start_s: "<<start_s<<std::endl;
    std::vector<std::pair<double,double>> last_cuve_info{};
    std::pair<double,double> last_offset_kappas= std::make_pair(1000000.0, 0.0);
    for(int i=0;i<smooth_split_info.lane_curvpoints.size();i++){
        std::vector<std::pair<double,double>> cuve_info = smooth_split_info.lane_curvpoints[i];
        int curve_index = 0;
        //跨link的点，用下一段link的第一个点和上一段link的倒数第二个点求
        if(last_cuve_info.size()>1){
             last_offset_kappas = *(last_cuve_info.rbegin()+1);
        }else if(last_cuve_info.size()>0){
            last_offset_kappas = last_cuve_info.back();
        }else{
            last_offset_kappas= std::make_pair(1000000.0, 0.0);
        }
        
        if(i == 0){
            curve_index = 1;
        }else{
            curve_index = 0;
        }
        //std::cout << __FILE__ << "," << __LINE__ << "," << " i: "<<i <<" ,curve_index: "<<curve_index<<std::endl;
        for(; curve_index<cuve_info.size();curve_index++){
            //std::cout << __FILE__ << "," << __LINE__ << "," << " i: "<<i <<" ,curve_index: "<<curve_index<<std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << " last_offset_kappas.second: "<<last_offset_kappas.second <<" ,last_offset_kappas.first: "<<last_offset_kappas.first<<std::endl;
            if(curve_index == 0 ){
                if(last_offset_kappas.first<1000000.0 && (std::abs(cuve_info[curve_index].first - last_offset_kappas.first)>0.001)){
                    //std::cout << __FILE__ << "," << __LINE__ << "," << " cuve_info[curve_index].second: "<<cuve_info[curve_index].second <<" ,cuve_info[curve_index].first: "<<cuve_info[curve_index].first<<std::endl;
                    //std::cout << __FILE__ << "," << __LINE__ << "," << " last_offset_kappas.second: "<<last_offset_kappas.second <<" ,last_offset_kappas.first: "<<last_offset_kappas.first<<std::endl;
                    dkappas = (cuve_info[curve_index].second - last_offset_kappas.second)/(cuve_info[curve_index].first - last_offset_kappas.first);
                    //std::cout << __FILE__ << "," << __LINE__ << "," << " fz: "<<(cuve_info[curve_index].second - last_offset_kappas.second) <<" ,fm: "<<(cuve_info[curve_index].first - last_offset_kappas.first)<<std::endl;
                    if(std::abs(dkappas)< p_split_dkappas_end ){
                        if(i != 0){
                            //std::cout << __FILE__ << "," << __LINE__ << "," << "std::abs(dkappas)< p_split_dkappas_end"<<std::endl;
                            //std::cout << __FILE__ << "," << __LINE__ << "," << "split_front_s break: "<<split_front_s<<std::endl;
                            break;                                
                        }
                    }else{
                        if((cuve_info[curve_index].first - last_offset_kappas.first)> 0){
                            split_front_s = GetDkappasSmoothExtendDist(dkappas)+cuve_info[curve_index].first - start_s;
                        }                         
                    }                  
                }
            }else{
                if(curve_index-1 >= 0 && curve_index-1<cuve_info.size()){
                    //std::cout << __FILE__ << "," << __LINE__ << "," << " cuve_info[curve_index].second: "<<cuve_info[curve_index].second <<" ,cuve_info[curve_index].first: "<<cuve_info[curve_index].first<<std::endl;
                    //std::cout << __FILE__ << "," << __LINE__ << "," << " cuve_info[curve_index-1].second: "<<cuve_info[curve_index-1].second <<" ,cuve_info[curve_index-1].first: "<<cuve_info[curve_index-1].first<<std::endl;
                    if(std::abs(cuve_info[curve_index].first - cuve_info[curve_index-1].first)>0.001){
                        dkappas = (cuve_info[curve_index].second - cuve_info[curve_index-1].second)/(cuve_info[curve_index].first - cuve_info[curve_index-1].first);
                        //std::cout << __FILE__ << "," << __LINE__ << "," << " fz: "<<(cuve_info[curve_index].second - cuve_info[curve_index-1].second) <<" ,fm: "<<(cuve_info[curve_index].first - cuve_info[curve_index-1].first)<<std::endl;
                        if(std::abs(dkappas)< p_split_dkappas_end ){
                            if(i != 0){
                                //std::cout << __FILE__ << "," << __LINE__ << "," << "std::abs(dkappas)< p_split_dkappas_end"<<std::endl;
                                //std::cout << __FILE__ << "," << __LINE__ << "," << "split_front_s break: "<<split_front_s<<std::endl;
                                break;                                
                            }

                        }else{
                            if((cuve_info[curve_index].first - cuve_info[0].first)> 0){
                                split_front_s = GetDkappasSmoothExtendDist(dkappas)+cuve_info[curve_index].first - start_s;
                            }                             
                        }
               
                    } 
                }
            }
            //std::cout << __FILE__ << "," << __LINE__ << "," << "dkappas: "<<dkappas<<std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "GetDkappasSmoothExtendDist(dkappas): "<<GetDkappasSmoothExtendDist(dkappas)<<std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << "," << "split_front_s: "<<split_front_s<<std::endl;                    
        }
        last_cuve_info = cuve_info;
        
        //std::cout << __FILE__ << "," << __LINE__ << "," << " i: "<<i <<" ,last_offset_kappas.first: "<<last_offset_kappas.first
        //<<" ,last_offset_kappas.second: "<<last_offset_kappas.second<<std::endl;
    }
    split_front_s = std::min(100.0,split_front_s);
}

double ReferenceLineProvider::GetDkappasSmoothExtendDist(double dkappas) {
    double dkappas_val = std::abs(dkappas);
    if (dkappas_val > 0.0015) {
        return 50.0;
    } else if (dkappas_val > 0.0012) {
        return 40.0;
    }else{
        return 28.0;
    }
    return 0.0;
}

void ReferenceLineProvider::SplitRoadCheck(const SmoothSplitInfo& smooth_split_info, double& split_front_s){
    //对split_front_final_s处的line type ,以及是否是道路分歧进行判断；
    //1. 如果是道路分歧，那么需要缩减到道路分歧前， 
    //2. 如果边线是实现，也要缩减split_front_final_s
    split_front_s = 0;
    //判断是否道路分歧
    bool is_road_split = false;
    if(smooth_split_info.link_id.size()>smooth_split_info.side_link_id.size()){
        is_road_split = true;
        for(int link_index = 0; link_index < smooth_split_info.side_link_id.size() &&link_index < smooth_split_info.side_link_length.size();link_index++){
            split_front_s += smooth_split_info.side_link_length[link_index];
        }
        return;
    }

}

void ReferenceLineProvider::SplitLineTypeCheck(const SmoothSplitInfo& smooth_split_info, const RefLineSmoothConfig &config ,double& split_front_s){
    split_front_s = 0;
    // 考虑边线类型,正的曲率考虑右边线，负的曲率考虑左边线
    uint8_t line_type = 0;
    double p_curve_take_line_thred = config.p_curve_take_line_thred;
    double split_start_s = smooth_split_info.dist;
    int curv_points_num = 0;
    double accum_curve = 0;
    bool reach_length_f = false;
    std::vector<uint8_t> left_line_type{};
    std::vector<uint8_t> right_line_type{};
    for(int link_index = 0; link_index<smooth_split_info.lane_curvpoints.size()&&link_index<smooth_split_info.left_line.size()&&link_index<smooth_split_info.right_line.size();link_index++){
        if(left_line_type.size()<=link_index){
            left_line_type.push_back(smooth_split_info.left_line[link_index]);
        }
        if(right_line_type.size()<=link_index){
            right_line_type.push_back(smooth_split_info.right_line[link_index]);
        }
        auto lane_curvpoints = smooth_split_info.lane_curvpoints[link_index];
        //计算一段link的平均曲率
        if(lane_curvpoints.size()<=0){
            return;
        }else{
            for(auto curv:lane_curvpoints){
                accum_curve += curv.second;
                curv_points_num++;
                if(curv.first - split_start_s > 100){
                    reach_length_f = true;
                    break;
                }
            }
        }
        if(reach_length_f){
            break;
        }
    }  
    double averge_curve = 0;
    if(curv_points_num>0){
        averge_curve = accum_curve/curv_points_num;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "accum_curve: "<<accum_curve<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "curv_points_num: "<<curv_points_num<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "averge_curve: "<<averge_curve<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "p_curve_take_line_thred: "<<p_curve_take_line_thred<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "-p_curve_take_line_thred: "<<(-p_curve_take_line_thred)<<std::endl;
        if(averge_curve> p_curve_take_line_thred){
            //大于一正值，考虑右线，如果右线有不可跨的线，那么就是硬约束            
            for(auto type: right_line_type){
                if(type == 2 || type == 7){
                    split_front_s = 100;
                    return;
                }
            }
        }else if(averge_curve< (-p_curve_take_line_thred)){
            //小于于一负值，考虑左线，如果左线有不可跨的线，那么就是硬约束
            for(auto type: left_line_type){
                if(type == 2 || type == 6){
                    split_front_s = 100;
                    return;
                }
            }
        }
    }else{
        return; //error
    }   
}

void ReferenceLineProvider::FixMergeSplitSectionBoundsByLineType(const std::vector<double> &left_boundary, const std::vector<double> &right_boundary,const std::vector<uint8_t>& left_bounds_type, const std::vector<uint8_t>& right_bounds_type,const std::vector<BoundSection_s> &section_index_vec,
                                                                 const RefLineSmoothConfig &config,std::vector<double>& upper_bounds, std::vector<double>& lower_bounds){
    //如果左线不可跨越，那么上边界小，如果右线不可跨越，那么下边界小
    //对已经拓宽的边界进行修正
    if(left_bounds_type.size() != upper_bounds.size()){
        return;
    }
    if(right_bounds_type.size() != lower_bounds.size()){
        return;
    }
    for(int i = 0;i<left_bounds_type.size() && i<upper_bounds.size(); i++){
        if(left_bounds_type[i] == 2 || left_bounds_type[i]==6 ){
            upper_bounds[i]= config.bounds_val;
        }
    }
    for(int i = 0;i<right_bounds_type.size() && i<lower_bounds.size(); i++){
        if(right_bounds_type[i] == 2 || right_bounds_type[i]==7 ){
            lower_bounds[i]= config.bounds_val;
        }
    }

    //merge的拓宽边界不考虑线型，将merge的地方还原
    for(auto section:section_index_vec){
        if(section.scene == 18 || section.scene == 8){
            //merge
            for(int i = section.start_index; i <= section.end_index && i < upper_bounds.size(); i++){
                double val = config.p_split_merge_bounds_val_factor * config.bounds_val;
                if(upper_bounds[i]<val){
                    upper_bounds[i] = val;
                }
            }
            for(int i = section.start_index; i <= section.end_index && i < lower_bounds.size(); i++){
                double val = config.p_split_merge_bounds_val_factor * config.bounds_val;
                if(lower_bounds[i]<val){
                    lower_bounds[i] = val;
                }
            }
            for(int i = section.start_index; i <= section.end_index && i < upper_bounds.size()&& i<left_boundary.size()&&i<left_bounds_type.size(); i++){
                double val = config.p_split_merge_bounds_val_factor * config.bounds_val;
                if(left_bounds_type[1]==2){
                    upper_bounds[i]= BoundLatch(left_boundary[i],config.bounds_val,config.p_merge_gap_to_lane_marker);
                }
            }  
            for(int i = section.start_index; i <= section.end_index && i < lower_bounds.size()&&i<right_boundary.size()&&i<right_bounds_type.size(); i++){
                double val = config.p_split_merge_bounds_val_factor * config.bounds_val;
                if(right_bounds_type[i]==2){
                    lower_bounds[i] = BoundLatch(right_boundary[i],config.bounds_val,config.p_merge_gap_to_lane_marker);
                }
            }          
        }

        if(section.scene == 18 && upper_bounds.size()>0 && lower_bounds.size()>0){//防止merge跳
            upper_bounds.back()=config.bounds_val;
            lower_bounds.back()=config.bounds_val;
        }

    }

}


void ReferenceLineProvider::FixMergeSplitSectionBounds(std::vector<double> &bounds, const RefLineSmoothConfig &config,
                                                       const std::vector<BoundSection_s> index_v) {
    for (auto &iter : index_v) {
        for (int i = iter.start_index; i <= iter.end_index && i < bounds.size(); i++) {
            if(iter.scene == 1){
                if(bounds[i]<config.p_continuous_split_bounds_val_factor * config.bounds_val){
                    bounds[i] = config.p_continuous_split_bounds_val_factor * config.bounds_val;
                }
            }else if(iter.scene == 4){
                //大曲率不可跨越的线的，缩小
                bounds[i] = config.bounds_val;
            }else if(iter.scene == 3){
                //道路分歧，缩小
                bounds[i] = config.bounds_val;
            }else if(iter.scene == 13){
                 bounds[i] = config.p_short_continuous_split_bounds_val_factor * config.bounds_val;
            }else if(iter.scene == 99){
                 bounds[i] = config.p_straight_split_bounds_val_factor * config.bounds_val;
            }else{
                if(bounds[i]<config.p_split_merge_bounds_val_factor * config.bounds_val){
                    bounds[i] = config.p_split_merge_bounds_val_factor * config.bounds_val;
                }                
            }           
        }
    }
}

void ReferenceLineProvider::CalStartEndIndex(int ego_index, double tar_s, double front_s, double back_s,
                                             int &start_index, int &end_index) {
    double back_center_line_sample_dist = std::max(0.1, p_back_center_line_sample_dist);
    double front_center_line_sample_dist = std::max(0.1, p_front_center_line_sample_dist);
    if ((tar_s - back_s) < 0) {
        start_index = ego_index - static_cast<int>(std::abs(tar_s - back_s) / back_center_line_sample_dist);
    } else {
        start_index = ego_index + static_cast<int>(std::abs(tar_s - back_s) / front_center_line_sample_dist);
    }

    if ((tar_s + front_s) < 0) {
        end_index = ego_index - static_cast<int>(std::abs(tar_s + front_s) / back_center_line_sample_dist);
    } else {
        end_index = ego_index + static_cast<int>(std::abs(tar_s + front_s) / front_center_line_sample_dist);
    }

    return;
}

void ReferenceLineProvider::CutStartEndIndex(std::vector<BoundSection_s>& section_cut) {
    //从最远的开始，如果有道路硬约束，那么，近处的end不能超过远的硬约束
    for(int i = section_cut.size()-1;i>=0 && i<section_cut.size();i--){
        if(section_cut[i].scene == 3){
            for(int j = i-1;j>=0 && j<section_cut.size();j--){
                if(section_cut[j].end_index> section_cut[i].end_index){
                    section_cut[j].end_index = section_cut[i].end_index;
                }
            }
        }
    }

    return;
}

void ReferenceLineProvider::RemoveDeviatePoints(
    const double limit_extend_ratio, const std::vector<double>& reference_list,
    std::vector<double>& list) {
  if (list.size() < 2 || list.size() != reference_list.size()) {
    // std::cout << "[RemoveDeviatePoints] size error: " << list.size();
    return;
  }

  
  double max_delta_val = 0.0;
  for (size_t i = 1; i < reference_list.size(); ++i) {
    double delta_val = reference_list[i] - reference_list[i - 1];
    if (max_delta_val < std::fabs(delta_val)) {
      max_delta_val = std::fabs(delta_val);
    }
  }

  
  max_delta_val *= std::max(limit_extend_ratio, 1.0);
  for (size_t i = 1; i < list.size(); ++i) {
    double delta_val = reference_list[i] - list[i];
    if (std::fabs(delta_val) > max_delta_val) {
      
      list[i] = list[i - 1];
    }
  }
}

bool ReferenceLineProvider::JudgeSplitLaneDir(const SmoothSplitInfo& smooth_split_info, uint8_t& dir){
    dir = 0;
    if(smooth_split_info.lane_id.size()>0 && smooth_split_info.side_lane_id.size()>0){
        if(smooth_split_info.lane_id[0] < smooth_split_info.side_lane_id[0]){
            dir = 2; //right
        }else if(smooth_split_info.lane_id[0] > smooth_split_info.side_lane_id[0]){
            dir = 1; //left
        }else{
            //error
        }
    }else{
        return false;
    }
    return true;
}

void ReferenceLineProvider::FixBoundsByLaneDir(const std::vector<double> &left_boundary, const std::vector<double> &right_boundary,const std::vector<uint8_t> &left_bounds_type,const std::vector<uint8_t> &right_bounds_type, 
                                                const std::vector<BoundSection_s> &index_v,const RefLineSmoothConfig &config,
                                                const std::vector<uint8_t> &lane_dir,std::vector<double>& upper_bounds, std::vector<double>& lower_bounds){
    for (int split_index = 0;split_index<index_v.size() && split_index<lane_dir.size();split_index++) {
        BoundSection_s iter = index_v[split_index];
        uint8_t dir = lane_dir[split_index];
        uint8_t scene = index_v[split_index].scene;
        if(dir == 1 && scene !=11 && scene !=12){
            for (int i = iter.start_index; i <= iter.end_index && i < upper_bounds.size() && i<lower_bounds.size() && i<left_bounds_type.size()&& i<left_boundary.size(); i++) {
                 if(left_bounds_type[i]!=8){
                     upper_bounds[i]= BoundLatch(left_boundary[i],config.bounds_val,1.8);  
                 }  
            }            
        }else if(dir == 2 && scene !=11 && scene !=12){
            for (int i = iter.start_index; i <= iter.end_index && i < upper_bounds.size() && i<lower_bounds.size()&& i<right_bounds_type.size()&& i<right_boundary.size(); i++) {
                 if(right_bounds_type[i]!=8){
                     lower_bounds[i]= BoundLatch(right_boundary[i],config.bounds_val,1.8);  
                 }    
            }            
        }else{
            //error
        }

    }
}
double ReferenceLineProvider::BoundLatch(double boundary_l, double config_bound, double to_bound_gap){
    double bound_out = config_bound;
    if(boundary_l-to_bound_gap < config_bound){
        bound_out = config_bound;
    }else{
        bound_out = (boundary_l-to_bound_gap);
    }
    return bound_out;
}

bool ReferenceLineProvider::StraightSplitBigSmooth(int ego_index, double split_s, const SmoothSplitInfo& smooth_split_info,int point_num,BoundSection_s& big_smooth_section){
    double cuve_sum = 0;
    int curve_num = 0;
    //限速大于68才加大
    if(smooth_split_info.before_lane_type.size()>0 && smooth_split_info.before_speed_limit.size()>0){
        if(smooth_split_info.before_lane_type[0] == 4 || smooth_split_info.before_lane_type[0] == 5 || smooth_split_info.before_lane_type[0] == 6){
            uint8_t curlane_spd = (smooth_split_info.before_speed_limit[0] & 0xF) * 10;
            if(curlane_spd<68){
                return false;
            }
        }else{
            if(smooth_split_info.before_speed_limit[0]<68){
                return false;
            }
        }
    }

    //100m范围内的第二段link之后的，每个点都小于一定值
    for(int link_index = 1; link_index <smooth_split_info.lane_curvpoints.size();link_index++){
        for(int curv_index = 0;curv_index<smooth_split_info.lane_curvpoints[link_index].size();curv_index++){
            double offset = smooth_split_info.lane_curvpoints[link_index].at(curv_index).first - split_s;
            double curv_val = smooth_split_info.lane_curvpoints[link_index].at(curv_index).second;
            if(offset < 100){
                if(curv_val>0.0002){
                    return false;
                }
            }
            curve_num++;
        }

    }
    if(curve_num<3){
        return false;
    }
    int start_index = 0;
    int end_index = 0;
    CalStartEndIndex(ego_index, split_s, 95, 0, start_index, end_index);  
    // std::cout << __FILE__ << "," << __LINE__ << "," << "start_index: " << start_index << std::endl; 
    // std::cout << __FILE__ << "," << __LINE__ << "," << "end_index: " << end_index << std::endl; 
    start_index = std::max(0, start_index);
    start_index = std::min(point_num - 1, start_index);
    end_index = std::max(0, end_index);
    end_index = std::min(point_num - 1, end_index);
    // std::cout << __FILE__ << "," << __LINE__ << "," << "point_num: " << point_num << std::endl; 
    
    BoundSection_s section_tmp(start_index, end_index,99);
    big_smooth_section = section_tmp;
    return true;

}
}  // namespace framework
}  // namespace shell
}  // namespace earth
