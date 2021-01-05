#include <gaussian_newton.h>
#include <readfile.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

#define test "test"
#define intel "intel"
#define killian "killian"

//for visual
void PublishGraphForVisulization(ros::Publisher* pub,
                                 std::vector<Eigen::Vector3d>& Vertexs,
                                 std::vector<Edge>& Edges,
                                 int color = 0)
{
    visualization_msgs::MarkerArray marray;

    //point--red
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "ls-slam";
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    if(color == 0)
    {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
    }
    else
    {
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
    }

    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);

    //linear--blue
    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "karto";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;

    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    else
    {
        edge.color.r = 1.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    edge.color.a = 1.0;

    m.action = visualization_msgs::Marker::ADD;
    uint id = 0;

    //加入节点
    for (uint i=0; i<Vertexs.size(); i++)
    {
        m.id = id;
        m.pose.position.x = Vertexs[i](0);
        m.pose.position.y = Vertexs[i](1);
        marray.markers.push_back(visualization_msgs::Marker(m));
        id++;
    }

    //加入边
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        edge.points.clear();

        geometry_msgs::Point p;
        p.x = Vertexs[tmpEdge.xi](0);
        p.y = Vertexs[tmpEdge.xi](1);
        edge.points.push_back(p);

        p.x = Vertexs[tmpEdge.xj](0);
        p.y = Vertexs[tmpEdge.xj](1);
        edge.points.push_back(p);
        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    pub->publish(marray);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "ls_slam");

    ros::NodeHandle nodeHandle;

    // beforeGraph
    ros::Publisher beforeGraphPub,afterGraphPub;
    beforeGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("beforePoseGraph",1,true);
    afterGraphPub  = nodeHandle.advertise<visualization_msgs::MarkerArray>("afterPoseGraph",1,true);

    std::string data;
    if (ros::param::get("data", data)) {
        std::cout << "Successfully retrieve data src: " << data << std::endl;
    } else {
        std::cout << "Could not retrieve data, set default to 'test'" << std::endl;
        data = test;
    }

    std::string VertexPath, EdgePath;

    if (data.compare(intel) == 0) {
        std::cout << "Using Intel data set" << std::endl;
        VertexPath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/intel-v.dat";
        EdgePath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/intel-e.dat";
    }
    else if (data.compare(killian) == 0) {
        std::cout << "Using killian data set" << std::endl;
        VertexPath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/killian-v.dat";
        EdgePath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/killian-e.dat";
    }
    else {
        std::cout << "Using test data set" << std::endl;
        VertexPath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/test_quadrat-v.dat";
        EdgePath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam_gtsam/data/test_quadrat-e.dat";
    }

    // std::string VertexPath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam/data/test_quadrat-v.dat";
    // std::string EdgePath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam/data/test_quadrat-e.dat";

    // std::string VertexPath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam/data/intel-v.dat";
    // std::string EdgePath = "/home/pengbo/laser-SLAM/assignments/HW6/LSSLAMProject/src/ls_slam/data/intel-e.dat";

    std::vector<Eigen::Vector3d> Vertexs;
    std::vector<Edge> Edges;

    ReadVertexInformation(VertexPath,Vertexs);
    ReadEdgesInformation(EdgePath,Edges);

    PublishGraphForVisulization(&beforeGraphPub,
                                Vertexs,
                                Edges);

    double initError = ComputeError(Vertexs,Edges);
    std::cout <<"initError:"<<initError<<std::endl;

    int maxIteration = 100;
    double epsilon = 1e-4;

    // for(int i = 0; i < maxIteration;i++)
    // {
    //     std::cout <<"Iterations:"<<i<<std::endl;
    //     Eigen::VectorXd dx = LinearizeAndSolve(Vertexs,Edges);

    //     //进行更新
    //     //TODO--Start
    //     for (int j = 0; j < Vertexs.size(); j++) {
    //         Vertexs[j] += dx.block(3 * j, 0, 3, 1);
    //         normalAngle(Vertexs[j](2));
    //     }
    //     //TODO--End

    //     double maxError = -1;
    //     for(int k = 0; k < 3 * Vertexs.size();k++)
    //     {
    //         if(maxError < std::fabs(dx(k)))
    //         {
    //             maxError = std::fabs(dx(k));
    //         }
    //     }

    //     if(maxError < epsilon)
    //         break;
    // }

    // Create an empty nonlinear factor graph
    NonlinearFactorGraph graph;

    // Add a Gaussian prior on pose x_1
    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(0, priorMean, priorNoise));

    // // Add two odometry factors
    // Pose2 odometry(2.0, 0.0, 0.0);
    // noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    // graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
    // graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

    // // create (deliberately inaccurate) initial estimate
    // Values initial;
    // initial.insert(1, Pose2(0.5, 0.0, 0.2));
    // initial.insert(2, Pose2(2.3, 0.1, -0.2));
    // initial.insert(3, Pose2(4.1, 0.1, 0.1));

    // // optimize using Levenberg-Marquardt optimization
    // Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    // Marginals marginals(graph, result);
    // std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
    // std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
    // std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;

    std::cout << "Adding edges..." << std::endl;
    for (auto & edge: Edges) {
        Eigen::Vector3d z = edge.measurement;
        double x = z(0);
        double y = z(1);
        double theta = z(2);

        // Eigen::Matrix3d info = edge.infoMatrix;
        // double info_xx = info(0, 0);
        // double info_xy = info(0, 1);
        // double info_xt = info(0, 2);
        // double info_yy = info(1, 1);
        // double info_yt = info(1, 2);
        // double info_tt = info(2, 2);
        Matrix3 info = edge.infoMatrix;
        // noiseModel::Gaussian::shared_ptr;
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
        
        Pose2 measurement(x, y, theta);
        // std::cout << "Adding edge between " << edge.xi << " and " << edge.xj << std::endl;
        graph.add(BetweenFactor<Pose2>(edge.xi, edge.xj, measurement, odometryNoise));
    }
    graph.print("\nFactor Graph:\n");

    std::cout << "Adding vertices..." << std::endl;
    Values initial;
    for (size_t i = 0; i < Vertexs.size(); i++)
    {
        Eigen::Vector3d pose = Vertexs[i];
        initial.insert(i, Pose2(pose(0), pose(1), pose(2)));
    }
    initial.print("\nInitial Estimate:\n");

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    Marginals marginals(graph, result);

    result.print("Final Result:\n");


    double finalError  = ComputeError(Vertexs,Edges);

    std::cout <<"FinalError:"<<finalError<<std::endl;

    PublishGraphForVisulization(&afterGraphPub,
                                Vertexs,
                                Edges,1);

    ros::spin();



    return 0;
}




