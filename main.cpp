#include <iostream>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include "physics.h"
#include "Visualizer.h"

int main() {
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::geometry::TriangleMesh::CreateIcosahedron(
            1.0);
    mesh->ComputeVertexNormals();
    Physics::Model _model = {.mesh = *mesh};
    Physics::countPVS(_model);

    Physics core(_model);
    core.solve(1000);
    Visualizer _visualizer(_model,
                           mesh);
    std::cout << " Hello World\n";
    return 0;
}
