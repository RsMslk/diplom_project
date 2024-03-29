
#include "physics.h"

Physics::Physics(Model &model) : _model(model) {
    _state = getInitState(model);
    countPVS(model);
}

State Physics::getInitState(const Model &model) {
    const auto &mesh = model.mesh_ptr;
    const auto &vertices = mesh->vertices_;
    Eigen::MatrixXd coords;
    coords.resize(3, vertices.size());
    coords.setZero();
    Eigen::MatrixXd vels;
    vels.resize(3, vertices.size());
    vels.setZero();

    for (std::size_t j = 0; j < vertices.size(); ++j) {
        coords.col(j) = vertices.at(j);
    }
    return State(coords, vels);
}

void Physics::countPVS(Model &model) {
    Params params;
    auto mesh = model.mesh_ptr;
    double V = mesh->GetVolume();
    if (V < 1e-3) {
        throw;
    }
    double P = params.R * params.T / V;
    Eigen::VectorXd S;
    const auto &vertices = mesh->vertices_;
    const auto &triangles = mesh->triangles_;

    S.resize(triangles.size());
    for (size_t j = 0; j < triangles.size(); ++j) {

        double surf = ((vertices.at(triangles.at(j)(0)) -
                        vertices.at(
                                triangles.at(j)(1))).cross(
                vertices.at(triangles.at(j)(1)) -
                vertices.at(triangles.at(j)(2))) /
                       2).norm();
        S(j) = surf;
    }
    model.pvs.P = P;
    model.pvs.V = V;
    model.pvs.S = S;
}

Eigen::MatrixXd
Physics::getSpringDamping(const Model &model,
                          const State &state) {
    const Eigen::MatrixXd coords = state.coords;
    const Eigen::MatrixXd vels = state.vels;
    Params params;
    auto mesh = model.mesh_ptr;
    const auto &triangles = mesh->triangles_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> F;
    F.resize(3, coords.cols());
    F.setZero();
    // TODO CODE STYLE
    for (const auto &triangle: triangles) {
        Eigen::Vector3d edge_0 = coords.col(triangle(0)) -
                                 coords.col(triangle(1));
        Eigen::Vector3d edge_1 = coords.col(triangle(1)) -
                                 coords.col(triangle(2));
        Eigen::Vector3d edge_2 = coords.col(triangle(2)) -
                                 coords.col(triangle(0));
        Eigen::Vector3d vel_0 = vels.col(triangle(0)) -
                                vels.col(triangle(1));
        Eigen::Vector3d vel_1 = vels.col(triangle(1)) -
                                vels.col(triangle(2));
        Eigen::Vector3d vel_2 = vels.col(triangle(2)) -
                                vels.col(triangle(0));
        Eigen::Vector3d vel_0_pr;

        if (vel_0.norm() > 1e-3)
            vel_0_pr = edge_0.dot(vel_0) / (vel_0).norm() *
                       edge_0.normalized();
        else
            vel_0_pr = Eigen::Vector3d::Zero();

        Eigen::Vector3d vel_1_pr;
        if (vel_1.norm() > 1e-3)
            vel_1_pr = edge_1.dot(vel_1) / (vel_1).norm() *
                       edge_1.normalized();
        else
            vel_1_pr = Eigen::Vector3d::Zero();

        Eigen::Vector3d vel_2_pr;
        if (vel_2.norm() > 1e-3)
            vel_2_pr =
                    edge_2.dot(vel_2) / (vel_2).norm() *
                    edge_2.normalized();
        else
            vel_2_pr = Eigen::Vector3d::Zero();


        F.col(triangle(0)) +=
                params.d * (vel_2_pr - vel_0_pr);
        F.col(triangle(1)) +=
                params.d * (vel_0_pr - vel_1_pr);
        F.col(triangle(2)) +=
                params.d * (vel_1_pr - vel_2_pr);
    }
    F = F / 2;
    return F;

}


Eigen::MatrixXd
Physics::getSpringForce(const Model &model,
                        const Eigen::MatrixXd &coords) {
    Params params;
    auto mesh = model.mesh_ptr;
    const auto &triangles = mesh->triangles_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> F;
    F.resize(3, coords.cols());
    F.setZero();
    for (const auto &triangle: triangles) {
        Eigen::Vector3d edge_0 = coords.col(triangle(0)) -
                                 coords.col(triangle(1));
        edge_0 = edge_0.normalized() * (edge_0.norm() - params.edge_rest);
        Eigen::Vector3d edge_1 = coords.col(triangle(1)) -
                                 coords.col(triangle(2));
        edge_1 = edge_1.normalized() * (edge_1.norm() - params.edge_rest);
        Eigen::Vector3d edge_2 = coords.col(triangle(2)) -
                                 coords.col(triangle(0));
        edge_2 = edge_2.normalized() * (edge_2.norm() - params.edge_rest);
        F.col(triangle(0)) += params.k * (edge_2 - edge_0);
        F.col(triangle(1)) += params.k * (edge_0 - edge_1);
        F.col(triangle(2)) += params.k * (edge_1 - edge_2);
    }
    F = F / 2;
    // F = 3 X 12
    return F;
}

Eigen::MatrixXd Physics::getGravForce(const Model &model,
                                      const Eigen::MatrixXd &coords) {
    Params params;
    auto mesh = model.mesh_ptr;
    const auto &triangles = mesh->triangles_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> F;
    F.resize(3, coords.cols());
    F.setZero();
    for (size_t j = 0; j < F.cols(); ++j) {
        F.col(j) = Eigen::Vector3d(0.0, 0.0, -params.g);
    }
    return F;
}


//coords, vels
State
Physics::rightSide(const State &state) {
    Eigen::MatrixXd vels = state.vels;
    Eigen::MatrixXd coords = state.coords;
    Eigen::MatrixXd r_dot = vels;
    Eigen::MatrixXd v_dot = getSpringForce(_model, coords) +
                            getSpringDamping(_model,
                                             state) +
                            getGravForce(_model, coords);
    return State(r_dot, v_dot);
}

State
Physics::integrateIter(const State &state) {
    Params params;
    const State k_1 = rightSide(state);
    const State k_2 = rightSide(
            state + k_1 * params.dt / 2.0);
    const State k_3 = rightSide(
            state + k_2 * params.dt / 2.0);
    const State k_4 = rightSide(state + k_1 * params.dt);

    return state +
           (k_1 + k_2 * 2.0 + k_3 * 2.0 + k_4) * params.dt /
           6;
}

void checkFloorCollision(State &state) {

    for (size_t j = 0; j < state.coords.cols(); ++j) {
        if (state.coords(2, j) < -10.0) {
            std::cout << " COLLISION\n";
            state.vels(2, j) = -state.vels(2, j);
        }
    }
}


void Physics::solve(size_t iter_num) {
    State current_state = _state;
    for (std::size_t i = 0; i < iter_num; ++i) {
        current_state = integrateIter(current_state);
        checkFloorCollision(current_state);
        _state = current_state;
        auto &vertices = _model.mesh_ptr->vertices_;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            _model.mesh_ptr->vertices_.at(
                    i) = current_state.coords.col(i);
        }
//        std::cout << _model.mesh_ptr->vertices_.at(0).transpose() << '\n';
    }

}

