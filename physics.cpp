
#include "physics.h"

Physics::Physics(Model &model) : _model(model) {
    _state = getInitState(model);
    countPVS(model);
}

Physics::State Physics::getInitState(const Model &model) {
    const auto &mesh = model.mesh;
    const auto &vertices = mesh.vertices_;
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
    auto mesh = model.mesh;
    double V = mesh.GetVolume();
    if (V < 1e-3) {
        throw;
    }
    double P = params.R * params.T / V;
    Eigen::VectorXd S;
    const auto &vertices = mesh.vertices_;
    const auto &triangles = mesh.triangles_;

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
Physics::getSpringForce(const Model &model,
                        const Eigen::MatrixXd &coords) {
    Params params;
    auto mesh = model.mesh;
    const auto &triangles = mesh.triangles_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> F;
    F.resize(3, coords.cols());
    F.setZero();
    for (const auto &triangle: triangles) {
        Eigen::Vector3d edge_0 = coords.col(triangle(0)) -
                                 coords.col(triangle(1));
        Eigen::Vector3d edge_1 = coords.col(triangle(1)) -
                                 coords.col(triangle(2));
        Eigen::Vector3d edge_2 = coords.col(triangle(2)) -
                                 coords.col(triangle(0));
        F.col(triangle(0)) += params.k * (edge_2 - edge_0);
        F.col(triangle(1)) += params.k * (edge_0 - edge_1);
        F.col(triangle(2)) += params.k * (edge_1 - edge_2);
    }
    F = F / 2;
    std::cout << F.rows() << " X " << F.cols() << '\n';
    // F = 3 X 12
    return F;
}

//coords, vels
Physics::State
Physics::rightSide(const State &state) {
    Eigen::MatrixXd vels = state.vels;
    Eigen::MatrixXd coords = state.coords;
    Eigen::MatrixXd r_dot = vels;
    Eigen::MatrixXd v_dot = getSpringForce(_model, coords);
    return State(r_dot, v_dot);
}

Physics::State
Physics::integrateIter(const Physics::State &state) {
    Params params;
    const State k_1 = rightSide(state);
    const State k_2 = rightSide(state + k_1 * params.dt / 2.0);
    const State k_3 = rightSide(state + k_2 * params.dt / 2.0);
    const State k_4 = rightSide(state + k_1 * params.dt);

    return state + (k_1 + k_2 * 2.0 + k_3 * 2.0 + k_4) * params.dt / 6;
}

void Physics::solve(size_t iter_num) {
    State current_state = _state;
    for ( std::size_t i = 0; i < iter_num; ++ i){
        current_state = integrateIter(current_state);
    }
    _state = current_state;
    auto & vertices = _model.mesh.vertices_;
    for (std::size_t i = 0; i < vertices.size(); ++i){
        vertices.at(i) = current_state.coords.col(i);
    }

}

