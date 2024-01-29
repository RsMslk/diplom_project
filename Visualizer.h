#ifndef MAIN_VISUALIZER_H
#define MAIN_VISUALIZER_H

#include "physics.h"

class Visualizer {
public:
    Visualizer(const Physics::Model &model);
    void init();

private:
    const Physics::Model &_model;
    open3d::visualization::VisualizerWithKeyCallback _visualizer;

};


#endif //MAIN_VISUALIZER_H
