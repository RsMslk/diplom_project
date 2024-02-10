#ifndef MAIN_VISUALIZER_H
#define MAIN_VISUALIZER_H

#include "physics.h"

class Visualizer {
public:
    Visualizer(Physics::Model &model);

    void init(
            const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);

    void registerCallbackFunction(std::function<bool(
            open3d::visualization::Visualizer *)> &callback);

    void update(
            std::shared_ptr<open3d::geometry::TriangleMesh> &mesh);

private:
    const Physics::Model &_model;
    open3d::visualization::VisualizerWithCustomAnimation _visualizer;


};


#endif //MAIN_VISUALIZER_H
