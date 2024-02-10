#include <iostream>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include "physics.h"
#include "Visualizer.h"

int main() {
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = open3d::geometry::TriangleMesh::CreateIcosahedron(
            1.0);
    mesh->ComputeVertexNormals();
    std::shared_ptr<Model> _model = std::make_shared<Model>(mesh);

    Physics::countPVS(*_model);
    std::cout << " P " <<_model->pvs.P << " V " << _model->pvs.V << " S " << _model->pvs.S << '\n';
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
