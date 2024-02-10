#include <iostream>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include "physics.h"
#include "Visualizer.h"
#include "thread"

int main() {
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::geometry::TriangleMesh::CreateIcosahedron(
            0.5);
    mesh->ComputeVertexNormals();
    std::shared_ptr<Physics::Model> _model = std::make_shared<Physics::Model>(mesh);

    Physics core(*_model);

    Visualizer _visualizer(*_model);

    std::function<bool(open3d::visualization::Visualizer* vis)> need_to_update = [&](open3d::visualization::Visualizer* vis){
        return true;
    };

//    _visualizer.registerCallbackFunction(need_to_update);
    for (int j = 0; j < 100000000; ++ j) {
        core.solve(1);
        _visualizer.update(mesh);
    }
    std::cout << " Hello World\n";
    return 0;
}
