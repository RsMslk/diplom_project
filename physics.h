#pragma once
#ifndef UNTITLED2_PHYSICS_H
#define UNTITLED2_PHYSICS_H

#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include <utility>


class Physics {
public:
    struct Params {
        double g = 9.8;
        double R = 8.31;
        double T = 273.0;
        double P = 1e5;
        double k = 100.0;
        double d = 0.1;
        double mass = 0.1;
        double dt = 0.01;
    };

    struct PVS {
        double P;
        double V;
        Eigen::VectorXd S;
    };

    struct Model {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr;
        PVS pvs;

        explicit Model(
                std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr_in = nullptr,
                PVS pvs_in = PVS(
                        {.P = 0.0, .V = 0.0, .S = Eigen::Vector3d::Zero()}))
                : mesh_ptr(std::move(mesh_ptr_in)),
                  pvs(pvs_in) {
        };
    };

    struct State {
        Eigen::MatrixXd coords;
        Eigen::MatrixXd vels;

        explicit State(
                Eigen::MatrixXd coords_in = Eigen::Matrix<double, 1, 1>::Zero(),
                Eigen::MatrixXd vels_in = Eigen::Matrix<double, 1, 1>::Zero())
                : coords(std::move(coords_in)),
                  vels(std::move(vels_in)) {};

        inline State operator+(const State &b) const {
            return State(b.coords + coords, b.vels + vels);
        }

        inline State operator-(const State &b) const {
            return State(coords - b.coords, vels - b.vels);
        }

        inline State operator*(const double &b) const {
            return State(b * coords, b * vels);
        }

        inline State operator/(const double &b) const {
            return State(coords / b, vels / b);
        }

    };

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
};


#endif //UNTITLED2_PHYSICS_H
