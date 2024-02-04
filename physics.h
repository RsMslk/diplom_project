#pragma once
#ifndef UNTITLED2_PHYSICS_H
#define UNTITLED2_PHYSICS_H

#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>


class Physics {
public:
    struct Params {
        double g = 9.8f;
        double R = 8.31f;
        double T = 273.0f;
        double P = 1e5f;
        double k = 1.0;
        double d = 0.1;
        double mass = 0.1;
    };
    struct Model {
        open3d::geometry::TriangleMesh mesh;
        Eigen::VectorXd vels;
    };

    Physics(Model &model);


    struct PVS {
        double P;
        double V;
        Eigen::VectorXd S;
    };

private:
    PVS countPVS(const Model &model);

    Eigen::MatrixXd getSpringForce(const Model &model);



    Model& _model;


};


#endif //UNTITLED2_PHYSICS_H
