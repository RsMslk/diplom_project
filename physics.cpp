
#include "physics.h"

Physics::Physics(Model &model) : _model(model) {};

Physics::PVS Physics::countPVS(const Model &model) {
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
    return {.P = P,
            .V = V,
            .S = S};
};

Eigen::MatrixXd
Physics::getSpringForce(const Model &model) {
    Params params;
    auto mesh = model.mesh;
    const auto &vertices = mesh.vertices_;
    const auto &triangles = mesh.triangles_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> F;
    F.resize(3, vertices.size());
    F.setZero();
    for (const auto &triangle: triangles) {
        Eigen::Vector3d edge_0 = vertices.at(triangle(0)) -
                                 vertices.at(triangle(1));
        Eigen::Vector3d edge_1 = vertices.at(triangle(1)) -
                                 vertices.at(triangle(2));
        Eigen::Vector3d edge_2 = vertices.at(triangle(2)) -
                                 vertices.at(triangle(0));
        F.col(triangle(0)) += params.k * (edge_2 - edge_0);
        F.col(triangle(1)) += params.k * (edge_0 - edge_1);
        F.col(triangle(2)) += params.k * (edge_1 - edge_2);
    }
    F = F / 2;
    std::cout << F.rows() <<" X " <<  F.cols() << '\n';
    return F;
}

