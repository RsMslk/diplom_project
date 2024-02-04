#include "Visualizer.h"

Visualizer::Visualizer(Physics::Model &model,
                       const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh)
        :
        _model(model),
        _visualizer(
                open3d::visualization::VisualizerWithKeyCallback()) {
    init(mesh);
}

void
Visualizer::init(
        const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {

    _visualizer.CreateVisualizerWindow();
    _visualizer.AddGeometry(mesh);
    _visualizer.UpdateGeometry(mesh);
    _visualizer.PollEvents();
    _visualizer.Run();
}
