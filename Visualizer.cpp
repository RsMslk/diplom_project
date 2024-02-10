#include "Visualizer.h"

Visualizer::Visualizer(Model &model)
        :
        _model(model),
        _visualizer(
                open3d::visualization::VisualizerWithCustomAnimation()) {
    init(_model.mesh_ptr);
}

void
Visualizer::init(
        const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh) {

    _visualizer.CreateVisualizerWindow();
    auto test_mesh = open3d::geometry::TriangleMesh::CreateBox(
            2.0, 10.0, 0.001);
    for (auto &vert: test_mesh->vertices_) {
        vert += Eigen::Vector3d (-1.0, -5.0, -10.0);
    }

    _visualizer.GetViewControl().ChangeFieldOfView(90.0);
    _visualizer.PrintVisualizerHelp();
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
    std::cout << mesh->GetVolume() << '\n';
    _visualizer.UpdateGeometry(mesh);
    _visualizer.PollEvents();
    _visualizer.UpdateRender();
}
