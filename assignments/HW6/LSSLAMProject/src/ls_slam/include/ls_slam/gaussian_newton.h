#ifndef GAUSSIAN_NEWTON_H
#define GAUSSIAN_NEWTON_H

#include <vector>
#include <eigen3/Eigen/Core>

typedef struct edge
{
  int xi,xj;
  Eigen::Vector3d measurement;
  Eigen::Matrix3d infoMatrix;
}Edge;


Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges);

double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges);

double normalAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;

    while (angle <-M_PI) angle += 2 * M_PI;

    return angle;
}







#endif
