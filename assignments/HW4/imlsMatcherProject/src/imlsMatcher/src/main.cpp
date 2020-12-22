#include "imls_icp.h"
#include <csm/csm_all.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改
// std::string bagfile = "/home/shenlan/imlsMatcherProject/src/bag/imls_icp.bag";
std::string bagfile = "/home/pengbo/laser-SLAM/assignments/HW4/imlsMatcherProject/src/bag/imls_icp.bag";

class imlsDebug
{
public:
    imlsDebug()
    {
        m_imlsPathPub = m_node.advertise<nav_msgs::Path>("imls_path_pub_",1,true);
        m_imlsPath.header.stamp = ros::Time::now();
        m_imlsPath.header.frame_id = "odom";
        m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
        m_odomPath.header.stamp = ros::Time::now();
        m_odomPath.header.frame_id = "odom";

        // PL-ICP or IMLS-ICP
        m_isFirstFrame = true;
        if (ros::param::get("use_imls", use_imls)) {
            std::cout << "Successfully retrieve 'use_imls'" << std::endl;
        } else {
            std::cout << "Could not retrieve 'use_imls', set default to 'true'" << std::endl;
            use_imls = true;
        }

        if (~use_imls) SetPLICPParams();

        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("/sick_scan"));
        topics.push_back(std::string("/odom"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //按顺序读取bag内激光的消息和里程计的消息
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            champion_nav_msgs::ChampionNavLaserScanConstPtr scan = m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
            if(scan != NULL)
                championLaserScanCallback(scan);

            nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if(odom != NULL)
                odomCallback(odom);

            ros::spinOnce();
            if(!ros::ok())
                break;
        }
        // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size(); ++i)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly) ||
               std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        if(m_isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            m_isFirstFrame = false;
            m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            ConvertChampionLaserScanToEigenPointCloud(msg, m_prevPointCloud);
            return ;
        }

        if (use_imls) {
            std::vector<Eigen::Vector2d> nowPts;
            ConvertChampionLaserScanToEigenPointCloud(msg, nowPts);

            //调用imls进行icp匹配，并输出结果．
            m_imlsMatcher.setSourcePointCloud(nowPts);
            m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

            Eigen::Matrix3d rPose,rCovariance;
            if(m_imlsMatcher.Match(rPose, rCovariance))
            {
                std::cout <<"IMLS Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
                Eigen::Matrix3d lastPose;
                lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                            sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                            0, 0, 1;
                Eigen::Matrix3d nowPose = lastPose * rPose;
                m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
                pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            }
            else
            {
                std::cout <<"IMLS Match Failed!!!!"<<std::endl;
            }

            m_prevPointCloud = nowPts;
        }
        else {
            LDP currentLDP;
            ConvertChampionLaserScanToLDP(msg, currentLDP);

            Eigen::Vector3d d_point_scan = PLICPBetweenTwoFrames(currentLDP, Eigen::Vector3d::Zero());
            Eigen::Matrix3d lastPose, rPose;

            lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                        sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                        0,  0,  1;

            rPose << cos(d_point_scan(2)), -sin(d_point_scan(2)), d_point_scan(0),
                    sin(d_point_scan(2)),  cos(d_point_scan(2)), d_point_scan(1),
                    0,  0,  1;

            Eigen::Matrix3d nowPose = lastPose * rPose;
            m_prevLaserPose << nowPose(0, 2) , nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0));
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
        }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        if(m_isFirstFrame == true)
            return;

        pubPath(msg, m_odomPath, m_odomPathPub);
    }

    //发布路径消息
    void pubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
        this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

        this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
        this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
        this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
        this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    //设置PL-ICP的参数
    void SetPLICPParams()
    {
        //设置激光的范围
        PLICPParams.min_reading = 0.1;
        PLICPParams.max_reading = 20;

        //设置位姿最大的变化范围
        PLICPParams.max_angular_correction_deg = 20.0;
        PLICPParams.max_linear_correction = 1;

        //设置迭代停止的条件
        PLICPParams.max_iterations = 50;
        PLICPParams.epsilon_xy = 0.000001;
        PLICPParams.epsilon_theta = 0.0000001;

        //设置correspondence相关参数
        PLICPParams.max_correspondence_dist = 1;
        PLICPParams.sigma = 0.01;
        PLICPParams.use_corr_tricks = 1;

        //设置restart过程，因为不需要restart所以可以不管
        PLICPParams.restart = 0;
        PLICPParams.restart_threshold_mean_error = 0.01;
        PLICPParams.restart_dt = 1.0;
        PLICPParams.restart_dtheta = 0.1;

        //设置聚类参数
        PLICPParams.clustering_threshold = 0.2;

        //用最近的10个点来估计方向
        PLICPParams.orientation_neighbourhood = 10;

        //设置使用PI-ICP
        PLICPParams.use_point_to_line_distance = 1;

        //不进行alpha_test
        PLICPParams.do_alpha_test = 0;
        PLICPParams.do_alpha_test_thresholdDeg = 5;

        //设置trimmed参数 用来进行outlier remove
        PLICPParams.outliers_maxPerc = 0.9;
        PLICPParams.outliers_adaptive_order = 0.7;
        PLICPParams.outliers_adaptive_mult = 2.0;

        //进行visibility_test 和 remove double
        PLICPParams.do_visibility_test = 1;
        PLICPParams.outliers_remove_doubles = 1;
        PLICPParams.do_compute_covariance = 0;
        PLICPParams.debug_verify_tricks = 0;
        PLICPParams.use_ml_weights = 0;
        PLICPParams.use_sigma_weights = 0;
    }

    //把激光雷达数据 转换为PL-ICP需要的数据
    void ConvertChampionLaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg, LDP& ldp)
    {
        int nPts = msg->ranges.size();
        // int nPts = pScan->intensities.size();
        ldp = ld_alloc_new(nPts);

        for (int i = 0; i < nPts; i++) {
            double dist = msg->ranges[i];
            if (dist > msg->range_min && dist < msg->range_max) {
                ldp->valid[i] = 1;
                ldp->readings[i] = dist;
            } else {
                ldp->valid[i] = 0;
                ldp->readings[i] = -1;
            }
            ldp->theta[i] = msg->angles[i];
        }
        ldp->min_theta = msg->angle_min;
        ldp->max_theta = msg->angle_max;

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;
    }

    //求两帧之间的icp位姿匹配
    Eigen::Vector3d  PLICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose) {
        prevLDP->odometry[0] = 0.0;
        prevLDP->odometry[1] = 0.0;
        prevLDP->odometry[2] = 0.0;

        prevLDP->estimate[0] = 0.0;
        prevLDP->estimate[1] = 0.0;
        prevLDP->estimate[2] = 0.0;

        prevLDP->true_pose[0] = 0.0;
        prevLDP->true_pose[1] = 0.0;
        prevLDP->true_pose[2] = 0.0;

        //设置匹配的参数值
        PLICPParams.laser_ref = prevLDP;
        PLICPParams.laser_sens = currentLDPScan;

        PLICPParams.first_guess[0] = tmprPose(0);
        PLICPParams.first_guess[1] = tmprPose(1);
        PLICPParams.first_guess[2] = tmprPose(2);

        PLICPResult.cov_x_m = 0;
        PLICPResult.dx_dy1_m = 0;
        PLICPResult.dx_dy2_m = 0;

        sm_icp(&PLICPParams,&PLICPResult);

        //nowPose在lastPose中的坐标
        Eigen::Vector3d  rPose;
        if(PLICPResult.valid)
        {
            //得到两帧激光之间的相对位姿
            rPose(0)=(PLICPResult.x[0]);
            rPose(1)=(PLICPResult.x[1]);
            rPose(2)=(PLICPResult.x[2]);

//        std::cout <<"Iter:"<<PLICPResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<PLICPResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<PLICPResult.error<<std::endl;

//        std::cout <<"PI ICP GOOD"<<std::endl;
        }
        else
        {
            std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
            rPose = tmprPose;
        }

        //更新

        //ld_free(prevLDP);

        prevLDP = currentLDPScan;

        return rPose;
    }

    bool m_isFirstFrame;
    ros::NodeHandle m_nh;
    IMLSICPMatcher m_imlsMatcher;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;
    nav_msgs::Path m_imlsPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher m_imlsPathPub;
    ros::Publisher m_odomPathPub;

    // PL-ICP
    bool use_imls;

    LDP prevLDP;
    sm_params PLICPParams;
    sm_result PLICPResult;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}

