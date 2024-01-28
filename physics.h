#pragma once
#ifndef UNTITLED2_PHYSICS_H
#define UNTITLED2_PHYSICS_H

#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>


class Physics {
public:
    struct params {
        float g = 9.8f;
        float R = 8.31f;
        float T = 273.0f;
        float P = 1e5f;
        float k = 1.0;
        float d = 0.1;
        float mass = 0.1;
    };
    struct model {
        open3d::geometry::TriangleMesh mesh;
        Eigen::VectorXd
    };


private:



};


#endif //UNTITLED2_PHYSICS_H
