#include "Visualizer.h"

Visualizer::Visualizer(const Physics::Model &model) :
        _model(model),
        _visualizer(
                open3d::visualization::VisualizerWithKeyCallback()) {
    init();
}

void Visualizer::init() {
//    _visualizer.;
    _visualizer.CreateVisualizerWindow();

}
