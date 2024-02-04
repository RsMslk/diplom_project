#ifndef MAIN_VISUALIZER_H
#define MAIN_VISUALIZER_H

#include "physics.h"

class Visualizer {
public:
    Visualizer(Physics::Model &model,
               const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);
    void init(
            const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);

private:
    const Physics::Model &_model;
    open3d::visualization::VisualizerWithKeyCallback _visualizer;

};


#endif //MAIN_VISUALIZER_H
