#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <eigen3/Eigen/SparseCholesky>

#include <iostream>
#include <ctime>


//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}


//转换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}


//计算整个pose-graph的误差
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
 */
void CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                          Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bi)
{
    //TODO--Start
    Eigen::Vector2d ti = xi.head(2);
    Eigen::Vector2d tj = xj.head(2);
    Eigen::Vector2d tij = z.head(2);

    double ri = xi(2);
    double rj = xj(2);
    double rij= z(2);

    Eigen::Matrix2d Ri, Rij;

    Ri << cos(ri),-sin(ri),
          sin(ri), cos(ri);
    
    Rij<< cos(rij),-sin(rij),
          sin(rij), cos(rij);

    // ei
    ei.head(2) = Rij.transpose() * (Ri.transpose() * (tj - ti) - tij);
    ei(2) = rj - ri - rij;
    normalAngle(ei(2));

    // Ai
    Ai.setZero();
    Ai.block(0, 0, 2, 2) = -Rij.transpose() * Ri.transpose();

    Eigen::Matrix2d dRiT;
    dRiT << -sin(ri), cos(ri), 
            -cos(ri),-sin(ri);
    Ai.block(0, 2, 2, 1) = Rij.transpose() * dRiT * (tj - ti);

    Ai(2, 2) = -1;

    // Bi
    Bi.setIdentity();
    Bi.block(0, 0, 2, 2) = Rij.transpose() * Ri.transpose();
    //TODO--end
}

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    //构造H矩阵　＆ b向量
    double start, end;
    start = clock();
    for(int i = 0; i < Edges.size();i++)
    {
        //提取信息
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bi;
        CalcJacobianAndError(xi,xj,z,ei,Ai,Bi);

        //TODO--Start
        // H
        H.block(3*tmpEdge.xi, 3*tmpEdge.xi, 3, 3) += Ai.transpose() * infoMatrix * Ai;
        H.block(3*tmpEdge.xi, 3*tmpEdge.xj, 3, 3) += Ai.transpose() * infoMatrix * Bi;
        H.block(3*tmpEdge.xj, 3*tmpEdge.xi, 3, 3) += Bi.transpose() * infoMatrix * Ai;
        H.block(3*tmpEdge.xj, 3*tmpEdge.xj, 3, 3) += Bi.transpose() * infoMatrix * Bi;

        // b
        b.block(3*tmpEdge.xi, 0, 3, 1) += Ai.transpose() * infoMatrix * ei;
        b.block(3*tmpEdge.xj, 0, 3, 1) += Bi.transpose() * infoMatrix * ei;
        //TODO--End
    }
    end = clock();
    std::cout << "Time cost for finding Jacobian: " << 1000.0 * (end - start) / CLOCKS_PER_SEC << " ms" << std::endl;

    //求解
    Eigen::VectorXd dx;

    //TODO--Start
    // sparse H
    start = clock();
    Eigen::SparseMatrix<double> Hsp = H.sparseView();

    // solver
    Eigen::SimplicialLLT< Eigen::SparseMatrix<double> > solver;
    solver.compute(Hsp);

    dx = solver.solve(-b);

    end = clock();
    std::cout << "Time cost for solving equation: " << 1000.0 * (end - start) / CLOCKS_PER_SEC << " ms" << std::endl;

    //TODO-End

    return dx;
}

