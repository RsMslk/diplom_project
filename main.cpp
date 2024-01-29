#include <iostream>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include "physics.h"
#include "Visualizer.h"

int main() {
    std::shared_ptr <open3d::geometry::TriangleMesh> mesh = open3d::geometry::TriangleMesh::CreateIcosahedron(
            1.0);
    Eigen::VectorXf vels;
    vels.resize(mesh->vertices_.size());
    Physics::Model _model = {.mesh = *mesh,
            .vels = vels};


    Physics core(_model);
    Visualizer _visualizer(_model);
    std::cout << " Hello World\n";
    return 0;
}
