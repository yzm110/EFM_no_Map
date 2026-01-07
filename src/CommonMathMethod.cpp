#include "CommonMathMethod.h"

namespace CommonTool {

DiscretePointsMath* DiscretePointsMath::GetInstance() {
    static DiscretePointsMath instance;
    return &instance;
}

bool DiscretePointsMath::ComputePathProfile(const EFMRefLinePoints& xy_points, std::vector<double>* headings,
                                            std::vector<double>* accumulated_s, std::vector<double>* kappas,
                                            std::vector<double>* dkappas) {
    headings->clear();
    kappas->clear();
    dkappas->clear();

    if (xy_points.size() < 2) {
        return false;
    }
    std::vector<double> dxs;
    std::vector<double> dys;
    std::vector<double> y_over_s_first_derivatives;
    std::vector<double> x_over_s_first_derivatives;
    std::vector<double> y_over_s_second_derivatives;
    std::vector<double> x_over_s_second_derivatives;

    // Get finite difference approximated dx and dy for heading and kappa
    // calculation
    std::size_t points_size = xy_points.size();
    for (std::size_t i = 0; i < points_size; ++i) {
        double x_delta = 0.0;
        double y_delta = 0.0;
        if (i == 0) {
            x_delta = (xy_points[i + 1].x - xy_points[i].x);
            y_delta = (xy_points[i + 1].y - xy_points[i].y);
        } else if (i == points_size - 1) {
            x_delta = (xy_points[i].x - xy_points[i - 1].x);
            y_delta = (xy_points[i].y - xy_points[i - 1].y);
        } else {
            x_delta = 0.5 * (xy_points[i + 1].x - xy_points[i - 1].x);
            y_delta = 0.5 * (xy_points[i + 1].y - xy_points[i - 1].y);
        }
        dxs.push_back(x_delta);
        dys.push_back(y_delta);
    }

    // Heading calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        headings->push_back(std::atan2(dys[i], dxs[i]));
    }

    // Get linear interpolated s for dkappa calculation
    double distance = 0.0;
    accumulated_s->push_back(distance);
    double fx = xy_points[0].x;
    double fy = xy_points[0].y;
    double nx = 0.0;
    double ny = 0.0;
    for (std::size_t i = 1; i < points_size; ++i) {
        nx = xy_points[i].x;
        ny = xy_points[i].y;
        double end_segment_s = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
        accumulated_s->push_back(end_segment_s + distance);
        distance += end_segment_s;
        fx = nx;
        fy = ny;
    }

    // Get finite difference approximated first derivative of y and x respective
    // to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = 0.0;
        double yds = 0.0;
        if (i == 0) {
            xds = (xy_points[i + 1].x - xy_points[i].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
            yds = (xy_points[i + 1].y - xy_points[i].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } else if (i == points_size - 1) {
            xds = (xy_points[i].x - xy_points[i - 1].x) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
            yds = (xy_points[i].y - xy_points[i - 1].y) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } else {
            xds = (xy_points[i + 1].x - xy_points[i - 1].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            yds = (xy_points[i + 1].y - xy_points[i - 1].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        x_over_s_first_derivatives.push_back(xds);
        y_over_s_first_derivatives.push_back(yds);
    }

    // Get finite difference approximated second derivative of y and x respective
    // to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xdds = 0.0;
        double ydds = 0.0;
        if (i == 0) {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
                   (accumulated_s->at(i + 1) - accumulated_s->at(i));
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
                   (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } else if (i == points_size - 1) {
            xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
                   (accumulated_s->at(i) - accumulated_s->at(i - 1));
            ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
                   (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } else {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
                   (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
                   (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        x_over_s_second_derivatives.push_back(xdds);
        y_over_s_second_derivatives.push_back(ydds);
    }

    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = x_over_s_first_derivatives[i];
        double yds = y_over_s_first_derivatives[i];
        double xdds = x_over_s_second_derivatives[i];
        double ydds = y_over_s_second_derivatives[i];
        double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        // if(kappa > 0.1){
        //     std::cout << "i: " << i << std::endl;
        //     std::cout << __FILE__ << __LINE__ << "kappa: " << kappa << std::endl;
        //     std::cout << "points:{ ";
        //     for(int i = 0; i < xy_points.size(); i ++){
        //         std::cout << "(" << xy_points[i].x << "," << xy_points[i].y << ")" << std::endl;
        //     }
        //     std::cout << "}" << std::endl;
        // }
        kappas->push_back(kappa);
    }

    // Dkappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double dkappa = 0.0;
        // if (i == 0) {
        //     dkappa = (kappas->at(i + 1) - kappas->at(i)) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        // } else if (i == points_size - 1) {
        //     dkappa = (kappas->at(i) - kappas->at(i - 1)) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        // } else {
        //     dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        // }

        if (i == 0) {
            dkappa = (kappas->at(i + 1) - kappas->at(i));
        } else if (i == points_size - 1) {
            dkappa = (kappas->at(i) - kappas->at(i - 1));
        } else {
            dkappa = (kappas->at(i + 1) - kappas->at(i - 1));
        }
        dkappas->push_back(dkappa);
    }
    return true;
}

bool DiscretePointsMath::GetNearestPointOnline(double utm_x, double utm_y, const EFMRefLinePoints& ref_line_points,
                                               int& nearest_index) {
    double min_dist = 65535;
    nearest_index = -1;
    EFMPoint ref_point;
    for (int i = 0; i < ref_line_points.size(); i++) {
        ref_point = ref_line_points[i];
        // printf("cartesian_point->x: %f ref_point.x: %f\n", cartesian_point->x, ref_point.x);
        // printf("cartesian_point->y: %f ref_point.y: %f\n", cartesian_point->y, ref_point.y);
        double dist = pow(utm_x - ref_point.x, 2) + pow(utm_y - ref_point.y, 2);
        dist = sqrt(dist);
        // printf("dist: %f i: %d\n", dist, i);
        if (dist <= min_dist) {
            nearest_index = i;
            min_dist = dist;
        }
    }
    return true;
}

bool DiscretePointsMath::IsPointInsideBodyCoordinate(EFMPoint point1, EFMPoint point2, EFMPoint tar_point) {
    double x_min = std::min(point1.x, point2.x);
    double x_max = std::max(point1.x, point2.x);
    double y_min = std::min(point1.y, point2.y);
    double y_max = std::max(point1.y, point2.y);

    bool is_inside = false;

    if (tar_point.x >= x_min && tar_point.x <= x_max && tar_point.y >= y_min && tar_point.y <= y_max) {
        // printf("point in the poins \n" );
        is_inside = true;
    }

    return is_inside;
}

bool DiscretePointsMath::InPolygon(EFMPoint tar_point, EFMRefLinePoints polygon){
    auto dcmp =[](double x)->int{
        if(fabs(x)<eps) return 0;
        else
            return x<0?-1:1;
    };    

    //判断点Q是否在P1和P2的线段上
    auto OnSegment=[=](EFMPoint P1,EFMPoint P2,EFMPoint Q){
        //前一个判断点Q在P1P2直线上 后一个判断在P1P2范围上
        return dcmp((P1-Q).CrossProduct(P2-Q))==0&&dcmp((P1-Q).InnerProduct(P2-Q))<=0;
    };

    bool flag = false; //相当于计数
    EFMPoint P1,P2; //多边形一条边的两个顶点
    for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++)
    {
        //polygon[]是给出多边形的顶点
        P1 = polygon[i];
        P2 = polygon[j];
        if(OnSegment(P1,P2,tar_point)){
           return true; //点在多边形一条边上 
        } 
        //前一个判断min(P1.y,P2.y)<P.y<=max(P1.y,P2.y)
        //这个判断代码我觉得写的很精妙 我网上看的 应该是大神模版
        //后一个判断被测点 在 射线与边交点 的左边
        if( ((dcmp(P1.y-tar_point.y)>0) != (dcmp(P2.y-tar_point.y)>0)) && dcmp(tar_point.x - (tar_point.y-P1.y)*(P1.x-P2.x)/(P1.y-P2.y)-P1.x)<0){
            flag = !flag;
        }
    }
    return flag;
}

bool DiscretePointsMath::PointInPolygon(EFMPoint tar_point, EFMRefLinePoints polygon){
    size_t n = polygon.size();
    bool inside = false;
    for(size_t i =0, j= n-1; i<n; j = i++){
        const EFMPoint & p1 =polygon[i];
        const EFMPoint & p2 =polygon[j];

        //检查点是否在多边形顶点
        if((tar_point.x == p1.x && tar_point.y == p1.y) ||(tar_point.x == p2.x && tar_point.y == p2.y)){
            return true;
        }

        //检查射线是否与边相交
        if((p1.y > tar_point.y) !=(p2.y > tar_point.y)){
            double intersect = (tar_point.y - p1.y)*(p2.x-p1.x)/(p2.y - p1.y) + p1.x;

            if(tar_point.x<= intersect){
                inside = !inside;
            }
        }

    }
    return inside;

}

bool DiscretePointsMath::CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    EFMPoint proj_point;
    bool is_inside = false;
    int nearest_index = -1;
    int sign = 1;
    if (GetProjectPointBodyCoordinate(tar_point, ref_line_points, proj_point, is_inside, nearest_index, sign) == false) {
        return false;
    }
    if (nearest_index >= ref_line_points.size() || nearest_index < 0) {
        return false;
    }

    double accumulate_s = 0.0;
    double dist_temp = 0.0;
    for (int i = 1; i <= nearest_index && nearest_index < ref_line_points.size(); i++) {
        dist_temp = sqrt(pow(ref_line_points[i].x - ref_line_points[i - 1].x, 2) +
                         pow(ref_line_points[i].y - ref_line_points[i - 1].y, 2));
        accumulate_s += dist_temp;
    }

    double dist = sqrt(pow(ref_line_points[nearest_index].x - proj_point.x, 2) +
                       pow(ref_line_points[nearest_index].y - proj_point.y, 2));
    if (proj_point.x < ref_line_points[nearest_index].x) {
        accumulate_s = accumulate_s - dist;
    } else {
        accumulate_s = accumulate_s + dist;
    }

    sl.s = accumulate_s;
    sl.l = sqrt(pow(tar_point.x - proj_point.x, 2) + pow(tar_point.y - proj_point.y, 2));
    
    //dir 
    // Point2Dd dir_line_vec(ref_line_points[1].x -ref_line_points[0].x, ref_line_points[1].y -ref_line_points[0].y);
    // Point2Dd dir_point_vec(tar_point.x -ref_line_points[0].x, tar_point.y -ref_line_points[0].y);
    // double sign = 1;
    // double cross = dir_line_vec.x *dir_point_vec.y - dir_line_vec.y * dir_point_vec.x;
    if(sign<0){
        sl.l = -sl.l;
    }
    // if(LeftOfLine(tar_point, ref_line_points[0], ref_line_points[1])){
    //      sl.l = -sl.l;
    // }
    return true;
}

bool DiscretePointsMath::CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    EFMPoint proj_point;
    is_inside = false;
    int nearest_index = -1;
    int sign = 1;
    if (GetProjectPointBodyCoordinate(tar_point, ref_line_points, proj_point, is_inside, nearest_index, sign) == false) {
        return false;
    }
    if (nearest_index >= ref_line_points.size() || nearest_index < 0) {
        return false;
    }

    double accumulate_s = 0.0;
    double dist_temp = 0.0;
    for (int i = 1; i <= nearest_index && nearest_index < ref_line_points.size(); i++) {
        dist_temp = sqrt(pow(ref_line_points[i].x - ref_line_points[i - 1].x, 2) +
                         pow(ref_line_points[i].y - ref_line_points[i - 1].y, 2));
        accumulate_s += dist_temp;
    }

    double dist = sqrt(pow(ref_line_points[nearest_index].x - proj_point.x, 2) +
                       pow(ref_line_points[nearest_index].y - proj_point.y, 2));
    if (proj_point.x < ref_line_points[nearest_index].x) {
        accumulate_s = accumulate_s - dist;
    } else {
        accumulate_s = accumulate_s + dist;
    }

    sl.s = accumulate_s;
    sl.l = sqrt(pow(tar_point.x - proj_point.x, 2) + pow(tar_point.y - proj_point.y, 2));
    
    //dir 
    // Point2Dd dir_line_vec(ref_line_points[1].x -ref_line_points[0].x, ref_line_points[1].y -ref_line_points[0].y);
    // Point2Dd dir_point_vec(tar_point.x -ref_line_points[0].x, tar_point.y -ref_line_points[0].y);
    // double sign = 1;
    // double cross = dir_line_vec.x *dir_point_vec.y - dir_line_vec.y * dir_point_vec.x;
    if(sign<0){
        sl.l = -sl.l;
    }
    // if(LeftOfLine(tar_point, ref_line_points[0], ref_line_points[1])){
    //      sl.l = -sl.l;
    // }
    return true;
}

bool DiscretePointsMath::CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside, int& nearest_index) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    EFMPoint proj_point;
    is_inside = false;
    nearest_index = -1;
    int sign = 1;
    if (GetProjectPointBodyCoordinate(tar_point, ref_line_points, proj_point, is_inside, nearest_index, sign) == false) {
        return false;
    }
    if (nearest_index >= ref_line_points.size() || nearest_index < 0) {
        return false;
    }

    double accumulate_s = 0.0;
    double dist_temp = 0.0;
    for (int i = 1; i <= nearest_index && nearest_index < ref_line_points.size(); i++) {
        dist_temp = sqrt(pow(ref_line_points[i].x - ref_line_points[i - 1].x, 2) +
                         pow(ref_line_points[i].y - ref_line_points[i - 1].y, 2));
        accumulate_s += dist_temp;
    }

    double dist = sqrt(pow(ref_line_points[nearest_index].x - proj_point.x, 2) +
                       pow(ref_line_points[nearest_index].y - proj_point.y, 2));
    if (proj_point.x < ref_line_points[nearest_index].x) {
        accumulate_s = accumulate_s - dist;
    } else {
        accumulate_s = accumulate_s + dist;
    }

    sl.s = accumulate_s;
    sl.l = sqrt(pow(tar_point.x - proj_point.x, 2) + pow(tar_point.y - proj_point.y, 2));
    
    //dir 
    // Point2Dd dir_line_vec(ref_line_points[1].x -ref_line_points[0].x, ref_line_points[1].y -ref_line_points[0].y);
    // Point2Dd dir_point_vec(tar_point.x -ref_line_points[0].x, tar_point.y -ref_line_points[0].y);
    // double sign = 1;
    // double cross = dir_line_vec.x *dir_point_vec.y - dir_line_vec.y * dir_point_vec.x;
    if(sign<0){
        sl.l = -sl.l;
    }
    // if(LeftOfLine(tar_point, ref_line_points[0], ref_line_points[1])){
    //      sl.l = -sl.l;
    // }
    return true;
}

bool DiscretePointsMath::CalPointSLBodyCoordinate(EFMRefLinePoints ref_line_points, EFMPoint tar_point, PointSLd& sl, bool& is_inside, EFMPoint& point_front, EFMPoint& point_back, EFMPoint& proj_point_res) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    EFMPoint proj_point;
    is_inside = false;
    int nearest_index = -1;
    int sign = 1;
    if (GetProjectPointBodyCoordinate(tar_point, ref_line_points, proj_point, is_inside, nearest_index, sign) == false) {
        return false;
    }
    if (nearest_index >= ref_line_points.size() || nearest_index < 0) {
        return false;
    }

    double accumulate_s = 0.0;
    double dist_temp = 0.0;
    for (int i = 1; i <= nearest_index && nearest_index < ref_line_points.size(); i++) {
        dist_temp = sqrt(pow(ref_line_points[i].x - ref_line_points[i - 1].x, 2) +
                         pow(ref_line_points[i].y - ref_line_points[i - 1].y, 2));
        accumulate_s += dist_temp;
    }

    double dist = sqrt(pow(ref_line_points[nearest_index].x - proj_point.x, 2) +
                       pow(ref_line_points[nearest_index].y - proj_point.y, 2));
    if (proj_point.x < ref_line_points[nearest_index].x) {
        accumulate_s = accumulate_s - dist;
        point_front = proj_point;
        point_back = ref_line_points[nearest_index];
    } else {
        accumulate_s = accumulate_s + dist;
        point_front = ref_line_points[nearest_index];
        point_back = proj_point;        
    }
    proj_point_res = proj_point;

    sl.s = accumulate_s;
    sl.l = sqrt(pow(tar_point.x - proj_point.x, 2) + pow(tar_point.y - proj_point.y, 2));
    
    //dir 
    // Point2Dd dir_line_vec(ref_line_points[1].x -ref_line_points[0].x, ref_line_points[1].y -ref_line_points[0].y);
    // Point2Dd dir_point_vec(tar_point.x -ref_line_points[0].x, tar_point.y -ref_line_points[0].y);
    // double sign = 1;
    // double cross = dir_line_vec.x *dir_point_vec.y - dir_line_vec.y * dir_point_vec.x;
    if(sign<0){
        sl.l = -sl.l;
    }
    // if(LeftOfLine(tar_point, ref_line_points[0], ref_line_points[1])){
    //      sl.l = -sl.l;
    // }
    return true;
}

bool DiscretePointsMath::GetProjectPointBodyCoordinate(EFMPoint tar_point, EFMRefLinePoints ref_line_points,
                                                       EFMPoint& proj_point, bool& is_inside, int& nearest_index, int& sign) {
    if (ref_line_points.size() < 2) {
        return false;
    }
    nearest_index = -1;
    GetNearestPointOnline(tar_point.x, tar_point.y, ref_line_points, nearest_index);
    if (nearest_index >= ref_line_points.size()) {
        nearest_index = std::max(0, static_cast<int>(ref_line_points.size() - 1));
    } else if (nearest_index < 0) {
        nearest_index = 0;
    }
    EFMPoint match_point = ref_line_points[nearest_index];

    // judge project point use nearest_index+1 or nearest -1
    EFMPoint vector_d(tar_point.x - match_point.x, tar_point.y - match_point.y);
    double vector_d_norm = sqrt(pow(vector_d.x, 2) + pow(vector_d.y, 2));
    EFMPoint vector_s(0.0, 0.0);
    double vector_s_norm = 0.0;
    double cos_A = -1.0;

    if ((ref_line_points.size() > nearest_index + 1) &&
        ((nearest_index - 1) >= 0 && (nearest_index - 1) < ref_line_points.size())) {
        vector_s.x = ref_line_points[nearest_index + 1].x - ref_line_points[nearest_index].x;
        vector_s.y = ref_line_points[nearest_index + 1].y - ref_line_points[nearest_index].y;
        if(fabs(vector_s.x)<0.000001 && fabs(vector_s.y)<0.000001){
            if((nearest_index+2)<ref_line_points.size()){
                vector_s.x = ref_line_points[nearest_index + 2].x - ref_line_points[nearest_index].x;
                vector_s.y = ref_line_points[nearest_index + 2].y - ref_line_points[nearest_index].y;                
            }
        }
        vector_s_norm = sqrt(pow(vector_s.x, 2) + pow(vector_s.y, 2));
        cos_A = (vector_s.x * vector_d.x + vector_s.y * vector_d.y) / (vector_s_norm * vector_d_norm);
        if (cos_A < 0) {
            // tar_point must between neareat+1 and nearest or between neareat-1 and nearest
            vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index - 1].x;
            vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index - 1].y;
            if(fabs(vector_s.x)<0.000001 && fabs(vector_s.y)<0.000001){
                if((nearest_index-2)<ref_line_points.size() && (nearest_index-2)>=0){
                    vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index-2].x;
                    vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index-2].y;                
                }
            }
        }
    } else if (nearest_index == 0 && (nearest_index + 1) < ref_line_points.size()) {
        vector_s.x = ref_line_points[nearest_index + 1].x - ref_line_points[nearest_index].x;
        vector_s.y = ref_line_points[nearest_index + 1].y - ref_line_points[nearest_index].y;
        if(fabs(vector_s.x)<0.000001 && fabs(vector_s.y)<0.000001){
            if((nearest_index+2)<ref_line_points.size()){
                vector_s.x = ref_line_points[nearest_index + 2].x - ref_line_points[nearest_index].x;
                vector_s.y = ref_line_points[nearest_index + 2].y - ref_line_points[nearest_index].y;                
            }
        }
    } else if (nearest_index == (ref_line_points.size() - 1) && (nearest_index - 1) >= 0 &&
               (nearest_index - 1) < ref_line_points.size()) {
        vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index - 1].x;
        vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index - 1].y;
        if(fabs(vector_s.x)<0.000001 && fabs(vector_s.y)<0.000001){
            if((nearest_index-2)<ref_line_points.size() && (nearest_index-2)>=0){
                vector_s.x = ref_line_points[nearest_index].x - ref_line_points[nearest_index-2].x;
                vector_s.y = ref_line_points[nearest_index].y - ref_line_points[nearest_index-2].y;                
            }
        }
    } else {
        // abnormal
        return false;
    }

    // cal vector_d project to vector_s 's ds
    vector_s_norm = sqrt(pow(vector_s.x, 2) + pow(vector_s.y, 2));
    double project_s = 0;
    if (vector_d_norm < 0.0001 || vector_s_norm <0.0001) {
        project_s = 0;
        proj_point.x = match_point.x;
        proj_point.y = match_point.y;
    } else {
        project_s = (vector_s.x * vector_d.x + vector_s.y * vector_d.y) / vector_s_norm;
        proj_point.x = match_point.x + project_s * vector_s.x / vector_s_norm;
        proj_point.y = match_point.y + project_s * vector_s.y / vector_s_norm;
    }

    // judge is inside line or not
    is_inside = true;
    if (nearest_index == 0 && (nearest_index + 1) < ref_line_points.size()) {
        is_inside =
            IsPointInsideBodyCoordinate(ref_line_points[nearest_index], ref_line_points[nearest_index + 1], proj_point);
    } else if (nearest_index == (ref_line_points.size() - 1) && (nearest_index - 1) >= 0 &&
               (nearest_index - 1) < ref_line_points.size()) {
        is_inside =
            IsPointInsideBodyCoordinate(ref_line_points[nearest_index], ref_line_points[nearest_index - 1], proj_point);
    } else {
        // if nearest not first, and also not last, it must be in middle
    }

    //dir
    EFMPoint dir_point_vec(tar_point.x -proj_point.x, tar_point.y -proj_point.y);
    double cross = vector_s.x *dir_point_vec.y - vector_s.y * dir_point_vec.x;
    if(cross<0){
        sign = -1;
    }else{
        sign =1;
    }    

    return true;
}

bool DiscretePointsMath::SaperateLineIntoTwoPart(const EFMRefLinePoints& marker_raw, EFMPoint tar_point,EFMRefLinePointsSection& marker_section) {                                                     
    marker_section[0].clear();
    marker_section[1].clear();
    EFMPoint proj_point;
    bool is_inside = false;
    int nearest_index = 0;
    int sign = 0;
    EFMRefLinePoints line_points{};
    for (int i = 0; i < marker_raw.size(); i++) {
        line_points.push_back({marker_raw[i].x, marker_raw[i].y});
    }
    if (CommonTool::DiscretePointsMath::GetProjectPointBodyCoordinate(tar_point, line_points,proj_point,
                                                                            is_inside, nearest_index,sign) == false) {
        return false;
    }

    if (is_inside == false || nearest_index >= marker_raw.size() || nearest_index < 0) {
        // project point not inside line
        return false;
    }

    EFMPoint proj_mkr(proj_point.x, proj_point.y);
    if (nearest_index < marker_raw.size() && marker_raw.size() > 0) {
        if (proj_point.x > marker_raw[nearest_index].x) {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index + 1; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }

        } else {
            for (int i = 0; i < marker_raw.size() && i <= nearest_index - 1; i++) {
                marker_section[0].push_back(marker_raw[i]);
            }
            marker_section[0].push_back(proj_mkr);

            marker_section[1].push_back(proj_mkr);
            for (int i = nearest_index; i < marker_raw.size(); i++) {
                marker_section[1].push_back(marker_raw[i]);
            }
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "nearest_index: " << nearest_index << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "is_inside: " << is_inside << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "proj_point.x: " << proj_point.x << " ,proj_point.y: " << proj_point.y << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " SaperateLineIntoTwoPart end"<<std::endl; 
    return true;
}

double DiscretePointsMath::LineLength(const EFMRefLinePoints& line_raw){
    double length = 0;
    for(int i =1;i<line_raw.size();i++){
        double dist = sqrt(pow(line_raw[i].x-line_raw[i-1].x,2)+ pow(line_raw[i].y-line_raw[i-1].y,2));
        length+= dist;
    }
    return length;
}

bool DiscretePointsMath::FixPathDensity(const EFMRefLinePoints& raw_reference_line, double sample_distance,
                                    EFMRefLinePoints& fixed_reference_line) {
    if (raw_reference_line.size() <= 1 || sample_distance < 0.1) {
        // std::cout << "raw point num to less" << std::endl;
        return false;
    }

    // 路径密度优化
    fixed_reference_line.clear();
    //
    double accumu_s = 0;
    EFMPoint point0 = raw_reference_line[0];
    fixed_reference_line.emplace_back(point0);
    //
    const double accumu_s_threshold = 5000;
    // std::cout << "FixPathDensity, raw_reference_line.size(): " << raw_reference_line.size() << std::endl;
    for (int i = 1; i < raw_reference_line.size() && accumu_s < accumu_s_threshold; i++) {
        const EFMPoint& point1 = raw_reference_line[i];
        double distance = sqrt(pow(point1.x - point0.x, 2) + pow(point1.y - point0.y, 2));
        if (distance <= sample_distance) {
            // std::cout << "   , continue;" << std::endl;
            continue;
        }
        while (distance > sample_distance && accumu_s < accumu_s_threshold) {
            double coeff = sample_distance / distance;
            double x = coeff * (point1.x - point0.x) + point0.x;
            double y = coeff * (point1.y - point0.y) + point0.y;
            point0.x = x;
            point0.y = y;
            fixed_reference_line.emplace_back(point0);
            accumu_s += sample_distance;
            distance -= sample_distance;
            // std::cout << " accumu_s: " << accumu_s << " distance: " << distance << std::endl;
        }
    }

    return true;
}

}  // namespace CommonMathMethod
