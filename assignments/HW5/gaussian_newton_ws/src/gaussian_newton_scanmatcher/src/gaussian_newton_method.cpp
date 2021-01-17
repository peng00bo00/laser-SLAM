#include <map.h>
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  <<   cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

Eigen::Vector3d GN_T2V(Eigen::Matrix3d T)
{
    Eigen::Vector3d vec;
    vec << T(0, 2), T(1, 2), atan2(T(1, 0), T(0, 0));

    return vec;
}

Eigen::Vector3d updatePoseVec(Eigen::Vector3d p, Eigen::Vector3d & dp) {
    Eigen::Matrix3d P = GN_V2T(dp) * GN_V2T(p);
    Eigen::Vector3d p_new = GN_T2V(P);

    // Eigen::Vector3d p_new = p + dp;

    return p_new;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    //TODO
    double x = (coords(0) - map->origin_x) / map->resolution + map->size_x / 2;
    double y = (coords(1) - map->origin_y) / map->resolution + map->size_y / 2;

    int x0 = floor(x);
    int y0 = floor(y);

    double u = x - x0;
    double v = y - y0;

    // scores for nearest 4 points
    double Z1 = map->cells[MAP_INDEX(map, x0, y0)].score;
    double Z2 = map->cells[MAP_INDEX(map, x0+1, y0)].score;
    double Z3 = map->cells[MAP_INDEX(map, x0+1, y0+1)].score;
    double Z4 = map->cells[MAP_INDEX(map, x0, y0+1)].score;

    // bilinear interpolation
    double l1 = (1-u) * (1-v);
    double l2 = u * (1-v);
    double l3 = u * v;
    double l4 = (1-u) * v;

    // score
    ans(0) = Z1*l1 + Z2*l2 + Z3*l3 + Z4*l4;

    // gradient
    ans(1) = (v * (Z3-Z4) + (1-v) * (Z2-Z1)) / map->resolution;
    ans(2) = (u * (Z3-Z2) + (1-u) * (Z4-Z1)) / map->resolution;

    //END OF TODO

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO    
    double x = now_pose(0);
    double y = now_pose(1);
    double theta = now_pose(2);

    Eigen::Matrix3d T = GN_V2T(now_pose);

    for (Eigen::Vector2d const&pt: laser_pts) {

        // transform to current pose
        Eigen::Vector2d ST = GN_TransPoint(pt, T);

        // dS
        Eigen::MatrixXd dS(2, 3);

        dS << 1, 0, -sin(theta) * pt(0) - cos(theta) * pt(1),
              0, 1,  cos(theta) * pt(0) - sin(theta) * pt(1);
        
        // M(S(T))
        Eigen::Vector3d MST = InterpMapValueWithDerivatives(map, ST);

        double score = MST(0);
        Eigen::Vector2d dM = MST.tail(2);

        // Jacobian
        Eigen::MatrixXd J = dM.transpose() * dS;

        H += J.transpose() * J;
        b += J.transpose() * (1 - score);
    }

    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 30;
    Eigen::Vector3d now_pose = init_pose;

    Eigen::Matrix3d H;
    Eigen::Vector3d b;

    Eigen::Vector3d dp;

    for(int i = 0; i < maxIteration; i++)
    {
        //TODO
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);

        dp = H.colPivHouseholderQr().solve(b);
        std::cout << "iteration: " << i << ", dp = " << dp.transpose() << std::endl;

        // now_pose += dp;
        now_pose = updatePoseVec(now_pose, dp);
        
        if (dp.norm() < 1e-3) break;
        //END OF TODO
    }
    std::cout << "Stop iteration with residual:" << dp.norm() << std::endl;
    init_pose = now_pose;

}
