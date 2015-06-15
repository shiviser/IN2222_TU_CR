#pragma once

// tf is always at the top
#include <tf/transform_broadcaster.h>

// standard libraries
#include <iostream>

// third parties
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// custom includes
#include "shapes_detector.h"
#include "bot_controller.h"


struct Component {
    static const int REMOVE = -1;

    // attributes
    int shape;
    int color;

    // slot in the workbench = {0, 1, 2, 3, 4}
    int slot;

    // -1 means remove
    int move_to;

    // constructor
    Component(int _shape, int _color, int _slot, int _move_to=Component::NO_MOVE):
            shape(_shape), color(_color), slot(_slot), move_to(_move_to) {
        if (move_to == Component::NO_MOVE) move_to = slot;
    }

private:
    static const int NO_MOVE = -2;
};


struct Slot {
    int slot;
    cv::Point3f coord;
};


class ProdMaster {
private:
    BotController* bot;
    ShapesDetector* shapes_detector;

    std::vector<Component> blueprint;

    std::vector<Slot> workbench_slots;

    // TODO: parsed variables / UI variables

protected:
    //
    bool get_workbench_state(const std::vector<Component>& blueprint, std::vector<Component>& workbench_state, std::vector<Component>& missing_components);

    //
    void rearrange_workbench(std::vector<Component>& workbench_state);

    //
    void order_components(std::vector<Component>& missing_components);

    //
    void receive_components(const std::vector<Component>& blueprint, std::vector<Component>& workbench_state);

    //
    void broadcast_loc(const cv::Point3d position3D, std::string ref, std::string target_name);


public:
    // constructor
    ProdMaster(ros::NodeHandle *n);

    // produce
    // return codes:
    //      0 - success
    int produce();


    // destructor
    ~ProdMaster();

};