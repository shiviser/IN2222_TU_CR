// all other includes go in the header
#include "prod_master.h"


ProdMaster::ProdMaster(ros::NodeHandle *n) {
    bot = new BotController(n);
    shapes_detector = new ShapesDetector(n);

    // config file for positions etc.
    parse_config_file("/config_data/config.ini");

    // TODO: parse file for production
    // load_production("/config_data/production.ini");
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


void ProdMaster::broadcast_loc(const cv::Point3d position3D, std::string ref, std::string target_name) {

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

    // important: move gripper bot to safe position
    bot->movetohome(false, true);

    // slots occupancy by the available objects on the workbench
    int TOTAL_SLOTS = workbench_slots_cambot.size();
    bool *occupied = new bool[TOTAL_SLOTS]();

    // getting present components on the workbench
    for (auto i_slot_pos = workbench_slots_cambot.begin(); i_slot_pos != workbench_slots_cambot.end(); ++i_slot_pos) {
        bot->movecambot(i_slot_pos->coord.x, i_slot_pos->coord.y, i_slot_pos->coord.z);

        // TODO: returns the shape and color or None ?
        int shape, color;
        // shapes_detector.julia();
        // if (no shape) {
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


void ProdMaster::rearrange_workbench(std::vector<Component>& workbench_state) {
    // important: move cam bot to safe position
    bot->movetohome(true, true);

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

        // TODO: do we want to store missing compoments in the state ??
        // makes element access by slot number easy but then we have to check if it is an object or missing object
        // movements can be thought of as a graph - a node represent a slot number and edge between two  connecting
        // nodes represent movement from first edge to second,
        // cycles in this graph means shuffles, solve them first
        // then for the remaining (acyclic, directed) paths start executing movements (edges) from tail to head of the path

    }

    // we might want to have two helper functions - for shuffling and for moving & removing

    // if time permits, try to get movement in a smooth path: 3/4 points trajectory
};


void ProdMaster::receive_components(const std::vector<Component>& blueprint, std::vector<Component>& workbench_state) {

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

        // order components
        order_components(missing_components);

        // wait until robotino is back by:
        // return a code (let's say 1)
        // and the main keeps listening to the robotino
        // and reacts based on if the produce returned 1 otherwise tells robotino that his communication is unexpected

        // fetch stuff from robotino
        receive_components(blueprint, workbench_state);
    }
}


ProdMaster::~ProdMaster() {
    delete bot;
    delete shapes_detector;
};
