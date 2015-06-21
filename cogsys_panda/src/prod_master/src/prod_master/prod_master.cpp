// all other includes go in the header
#include "prod_master.h"


ProdMaster::ProdMaster(ros::NodeHandle *n) {
    ros_node = n;

    // config file for positions etc.
    parse_config_file("/config_data/config.ini");

    // TODO: parse file for production
    // load_production("/config_data/production.ini");

    bot = new BotController(n);
    shapes_detector = new ShapesDetector(n);

    transformation_pub = n->advertise<std_msgs::String>("/prod_master/start_trans_listen", 1);
};


void ProdMaster::_parse_config_slots(QSettings &ini_file, std::string bot_name, std::vector<Slot>& slots_collection) {
    std::cout << "Parsing SLOTS for " << bot_name << " ... " << std::endl;

    ini_file.beginGroup("SLOTS");

    int N_SLOTS = ini_file.value("N_SLOTS", 0).toInt();
    std::cout << "N_SLOTS: " << N_SLOTS << std::endl;

    std::stringstream target_name;
    target_name.str("");
    target_name << bot_name << "_SLOTS_Z";
    float slots_height = ini_file.value(target_name.str().c_str(), 0.0f).toString().toFloat();
    std::cout << target_name.str().c_str() << ": " << slots_height << std::endl;

    for(unsigned int n = 0; n < N_SLOTS; ++n) {
        Slot cur_slot;
        cur_slot.slot = n;

        cv::Vec3f cur_slot__coord;
        // Z is same for all slots
        cur_slot__coord[2] = slots_height;

        // using ASCII value of X and Y
        for (char axis = 88; axis <= 89; ++axis) {
            target_name.str("");
            target_name << bot_name << "_SLOT_" << n << "_" << axis;

            cur_slot__coord[axis - 88] = ini_file.value(target_name.str().c_str(), 0.0f).toString().toFloat();
        }

        cur_slot.coord = cur_slot__coord;
        slots_collection.push_back(cur_slot);

        std::cout << bot_name << "_SLOT_" << n << ": slot = " << cur_slot.slot << ", coord = " << cur_slot.coord << std::endl;
    }

    ini_file.endGroup();
};


void ProdMaster::_parse_config_robots_pos(QSettings &ini_file, std::string bot_name, cv::Point3f& bot_wait_pos) {
    std::cout << "Parsing ROBOTS_POS for " << bot_name << " ... " << std::endl;
    ini_file.beginGroup("ROBOTS_POS");

    std::stringstream target_name;
    cv::Vec3f wait_pos;

    // using ASCII value of X, Y and Z get the cambot pos
    for (char axis = 88; axis <= 90; ++axis) {
        target_name.str("");
        target_name << bot_name << "_WAITS_ROBOTINO_" << axis;

        wait_pos[axis - 88] = ini_file.value(target_name.str().c_str(), 0.0f).toString().toFloat();
    }
    bot_wait_pos = wait_pos;
    std::cout << "CAMBOT_WAITS_ROBOTINO: " << bot_wait_pos << std::endl;

    ini_file.endGroup();
};


void ProdMaster::parse_config_file(std::string filepath) {
    std::string path = ros::package::getPath("prod_master");
    std::stringstream ss_path;
    ss_path << path << filepath;
    QString filename(ss_path.str().c_str());
    QFileInfo config(filename);

    if(!config.exists()) {
        std::cout <<"Error reading " << filepath << " file!" <<std::endl;
        throw;
    }

    QSettings ini_file(filename, QSettings::IniFormat);

    // START WORKBENCH SLOTS FOR CAMBOT
    _parse_config_slots(ini_file, "CAMBOT", workbench_slots_cambot);
    // END WORKBENCH SLOTS FOR CAMBOT

    // START WORKBENCH SLOTS FOR GRIPPERBOT
    _parse_config_slots(ini_file, "GRIPPERBOT", workbench_slots_gripperbot);
    // END WORKBENCH SLOTS FOR GRIPPERBOT

    // START COLLECTOR POSITION
    ini_file.beginGroup("SLOTS");

    std::stringstream target_name;
    // using ASCII value of X, Y and Z get the collector slot
    cv::Vec3f collector_slot__coord;
    for (char axis = 88; axis <= 90; ++axis) {
        target_name.str("");
        target_name << "SLOT_COLLECTOR_" << axis;

        collector_slot__coord[axis - 88] = ini_file.value(target_name.str().c_str(), 0.0f).toString().toFloat();
    }

    // saving to class variable
    collector_slot.slot = -1;
    collector_slot.coord = collector_slot__coord;
    std::cout << "SLOT_COLLECTOR: slot = " << collector_slot.slot << ", coord = " << collector_slot.coord << std::endl;

    ini_file.endGroup();
    // END COLLECTOR POSITION

    // START DIFFERENT ROBOTS POSITIONS
    _parse_config_robots_pos(ini_file, "CAMBOT", cambot_wait_pos);

    _parse_config_robots_pos(ini_file, "GRIPPERBOT", gripperbot_wait_pos);
    // END DIFFERENT ROBOTS POSITIONS
};


void ProdMaster::broadcast_loc(const cv::Point3d& position3D, std::string ref, std::string target_name) {

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position3D.x/1000.0,
                                    position3D.y/1000.0,
                                    position3D.z/1000.0));
    tf::Quaternion q;
    q.setX(0.0);
    q.setY(0.0);
    q.setZ(0.0);
    q.setW(1.0);
    transform.setRotation(q);

    // TODO: remove later
    std::cout << "broadcasted" << std::endl;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref, target_name));

};


bool ProdMaster::get_workbench_state(const std::vector<Component>& blueprint, std::vector<Component>& workbench_state, std::vector<Component>& missing_components) {
    bool is_done = true;

    // important: move gripper bot to wait position
    bot->movegripperbot(gripperbot_wait_pos.x, gripperbot_wait_pos.y, gripperbot_wait_pos.z);

    // slots occupancy by the available objects on the workbench
    int TOTAL_SLOTS = workbench_slots_cambot.size();
    bool *occupied = new bool[TOTAL_SLOTS]();

    // getting present components on the workbench
    for (auto i_slot_pos = workbench_slots_cambot.begin(); i_slot_pos != workbench_slots_cambot.end(); ++i_slot_pos) {
        bot->movecambot(i_slot_pos->coord.x, i_slot_pos->coord.y, i_slot_pos->coord.z);

        // TODO: returns the shape and color or None ?
        int shape, color;
        // std::vector<int> found_object = shapes_detector.get_object(false); //if {-1;-1}, there is no object
	// shape = found_object[0];
	// color = found_object[1];
	//
        // if (shape == -1 && colour == -1) {
        //     is_done = false;
        //     missing_components.push_back(Component(blueprint[i_slot_pos->slot]));
        //     continue;
        // }

        // initially every component detected is assumed to have to be removed
        Component cur_comp(shape, color, i_slot_pos->slot, Component::REMOVE);

        // find if component has to be retained by using the blueprint and which available slot number
        int move_to;
        for (auto i_blueprint_comp = blueprint.begin(); i_blueprint_comp != blueprint.end(); ++i_blueprint_comp) {
            if (i_blueprint_comp->shape == cur_comp.shape && i_blueprint_comp->color == cur_comp.color) {
                move_to = i_blueprint_comp->slot;
                if (!occupied[move_to]) {
                    // this slot is now occupied
                    occupied[move_to] = true;

                    // changed move to location to this slot
                    cur_comp.move_to = move_to;
                    break;
                }
            }
        }

        // production not done if component has to be moved
        if (cur_comp.slot != cur_comp.move_to) is_done = false;

        // update state
        workbench_state.push_back(cur_comp);
    }

    // cleanup and return
    delete [] occupied;
    return is_done;
};


void ProdMaster::execute_remove_components(std::vector<int>& to_move, std::vector<Component>& workbench_state) {
    // TODO if time permits, try to get movements in a smooth path: 3 or 4 points trajectory

    for (int i_comp = 0; i_comp < to_move.size(); ++i_comp) {
        // go to start position
        bot->movetohome(false, true);

        // initate move
        bot->opengripper();

        // move to piece
        Slot piece_slot = workbench_slots_gripperbot[workbench_state[i_comp].slot];
        bot->movegripperbot(piece_slot.coord.x, piece_slot.coord.y, piece_slot.coord.z);

        // hold the objecct
        bot->closegripper();

        // remove
        if (workbench_state[i_comp].move_to == Component::REMOVE) {
            // move to collector
            bot->movegripperbot(collector_slot.coord.x, collector_slot.coord.y, collector_slot.coord.z);
        }

        // to move to different location
        else {
            Slot move_to = workbench_slots_gripperbot[workbench_state[i_comp].move_to];
            bot->movegripperbot(move_to.coord.x, move_to.coord.y, move_to.coord.z);
        }

        // drop
        bot->opengripper();
    }
};


void ProdMaster::rearrange_workbench(std::vector<Component>& workbench_state) {
    // important: move cam bot to safe position
    bot->movecambot(cambot_wait_pos.x, cambot_wait_pos.y, cambot_wait_pos.z);

    // stores indices of the workbench_state that corresponds to components to be moved/shuffled
    // moved: consists of to be removed or moved to another position
    // for shuffling: pairs are stored as adjacent elements
    std::vector<int> to_shuffle;
    std::vector<int> to_move;

    for (int i_comp = 0; i_comp < workbench_state.size(); ++i_comp) {
        // it is in the position where it has to be
        if (workbench_state[i_comp].slot == workbench_state[i_comp].move_to) continue;

        // component to be removed
        if (workbench_state[i_comp].move_to == Component::REMOVE) {
            to_move.push_back(i_comp);
            continue;
        }

        // TODO: we're assuming no shuffling at the moment
        else {
            to_move.push_back(i_comp);
        }
        // movements can be thought of as a graph - a node represent a slot number and edge between two  connecting
        // nodes represent movement from first edge to second,
        // cycles in this graph means shuffles, solve them first
        // then for the remaining (acyclic, directed) paths start executing movements (edges) from tail to head of the path

    }

    // remove and move
    execute_remove_components(to_move, workbench_state);

    // TODO: shuffle
};


void ProdMaster::receive_components(const std::vector<Component>& blueprint, std::vector<Component>& workbench_state) {
    // TODO: detect
    bool detected = true;
    // std::vector<int> found_object = shapes_detector.get_object(false); //if {-1;-1}, there is no object

    if (detected) {

    }

};


// TODO: maybe make it a class function
//bool store_trans_result(prod_master_srvs::ComputedTransformation::Request  &req,
//         prod_master_srvs::ComputedTransformation::Response &res) {
//    // storing received data
////    result.x = req.x;
////    result.y = req.y;
////    result.z = req.z;
//
//    // received successfully
//    res.status = true;
//    return true;
//}

cv::Point3d ProdMaster::get_point_wrt_gripper(const cv::Point3d& pos3D_wrt_cambot) {
//    transformation_pub.publish("start");

    // TODO: http://answers.ros.org/question/12045/how-to-deliver-arguments-to-a-callback-function/
    cv::Point3d result;
//    transformation_server = ros_node->advertiseService("/prod_master_srvs/computed_transformation", store_trans_result);

    while (true) {
        broadcast_loc(pos3D_wrt_cambot, "cambot_wrist", "TARGET_BOX_0");
    }

    // TODO: remove
    return cv::Point3d(1.0f, 1.0f, 10.f);
};


void ProdMaster::order_components(std::vector<Component>& missing_components) {
    // talk to Robotino
    // move cambot and gripper bot to perpendicularly sidewise-outward position
};


int ProdMaster::produce() {
    bool is_done = false;
    std::vector<Component> workbench_state;
    std::vector<Component> missing_components;

    // until production done
    while(true) {
        // get state of the workbench
        is_done = get_workbench_state(blueprint, workbench_state, missing_components);
        if (is_done) return 0;

        // rearranging involves moving components to their correct positions
        rearrange_workbench(workbench_state);

        // nothing to get so we are done
        if (missing_components.size() == 0) return 0;

        // order components - uses a (sync) service to robotino in the robotino_controller
        order_components(missing_components);

        // fetch stuff from robotino
        receive_components(blueprint, workbench_state);
    }
};


ProdMaster::~ProdMaster() {
    delete bot;
    delete shapes_detector;
};
