#pragma once
#ifndef UNTITLED2_PHYSICS_H
#define UNTITLED2_PHYSICS_H

#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include <utility>
#include "types.h"

class Physics {
public:


    Physics(Model &model);

    static void countPVS(Model &model);

private:


    static Eigen::MatrixXd
    getSpringForce(const Model &model,
                   const Eigen::MatrixXd &coords);


    Model &_model;
    State _state;

    State
    rightSide(const State &state);

    State getInitState(const Model &model);

    State integrateIter(const State &state);

public:
    void solve(size_t iter_num);

    Eigen::MatrixXd getGravForce(const Model &model,
                                 const Eigen::MatrixXd &coords);

    Eigen::MatrixXd
    getSpringDamping(const Model &model,
                     const State &state);
};


#endif //UNTITLED2_PHYSICS_H
