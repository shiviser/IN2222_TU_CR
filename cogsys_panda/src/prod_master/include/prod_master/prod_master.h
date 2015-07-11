#pragma once

// tf is always at the top
#include <tf/transform_broadcaster.h>

// standard libraries
#include <iostream>

// third parties
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <QSettings>
#include <QFileInfo>
#include <std_msgs/String.h>
#include <prod_master_srvs/ComputedTransformation.h>

// custom includes
#include "shapes_detector.h"
#include "bot_controller.h"
#include "robotino_controller.h"


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
    ros::NodeHandle *ros_node;

    BotController* bot;
    RobotinoController* robotino;
    ShapesDetector* shapes_detector;

    std::vector<Component> blueprint;

    std::vector<Slot> workbench_slots_cambot;
    std::vector<Slot> workbench_slots_gripperbot;
    Slot collector_slot;

    cv::Point3f cambot_wait_pos;
    cv::Point3f gripperbot_wait_pos;

    bool ADVERSARIAL_ENV;

    ros::Publisher transformation_pub;
    ros::ServiceServer transformation_server;

    // parse_config_file helper
    void _parse_config_slots(QSettings &ini_file, std::string bot_name, std::vector<Slot>& slots_collection);

    // parse_config_file helper
    void _parse_config_robots_pos(QSettings &ini_file, std::string bot_name, cv::Point3f& bot_wait_pos);

protected:
    //
    bool initiate_production_state(std::vector<Component>& workbench_state, std::vector<Component>& missing_components);

    //
    void rearrange_workbench(std::vector<Component>& workbench_state);

    //
    bool order_components(std::vector<Component>& missing_components);

    //
    void receive_components(std::vector<Component>& workbench_state, std::vector<Component>& missing_components);

    //
    cv::Point3f get_position_wrt_gripper(int img_point_x, int img_point_y);

    //
    void parse_config_file(std::string filepath);

    // parse required objects
    void parse_blueprint_file(std::string filepath);

    //
    void execute_move_components(std::vector<int>& to_move, std::vector<Component>& workbench_state);

    //
    bool update_missing(const std::vector<Component>& workbench_state, std::vector<Component>& missing_components);

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
