#include "Visualizer.h"

Visualizer::Visualizer(Physics::Model &model)
        :
        _model(model),
        _visualizer(
                open3d::visualization::VisualizerWithCustomAnimation()) {
    init(_model.mesh_ptr);
}

void
Visualizer::init(
        const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {

    _visualizer.CreateVisualizerWindow();
    auto test_mesh = open3d::geometry::TriangleMesh::CreateBox(10.0, 1.0, 0.001);
    for ( auto & vert: test_mesh->vertices_){
        vert -= Eigen::Vector3d::UnitZ() * 50.0;
    }
    _visualizer.AddGeometry(test_mesh);
    _visualizer.AddGeometry(mesh);
    _visualizer.GetRenderOption().ToggleMeshShowWireframe();
    _visualizer.GetRenderOption().ToggleLightOn();
    _visualizer.PollEvents();
    _visualizer.UpdateRender();
}

void
Visualizer::registerCallbackFunction(std::function<bool(
        open3d::visualization::Visualizer *)> &callback) {
    _visualizer.RegisterAnimationCallback(callback);

}

void Visualizer::update(
        std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {
    _visualizer.UpdateGeometry(mesh);
//    std::cout << " " << _model.mesh_ptr->GetVolume() << '\n';
    _visualizer.PollEvents();
    _visualizer.UpdateRender();
}
