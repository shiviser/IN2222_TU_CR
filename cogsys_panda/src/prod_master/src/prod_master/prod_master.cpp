// all other includes go in the header
#include "prod_master.h"


ProdMaster::ProdMaster(ros::NodeHandle *n) {
    ros_node = n;

    // config file for positions etc.
    parse_config_file("/config_data/config.ini");

    // TODO: parse file for production
    // load_production("/config_data/production.ini");
    Component first(0, 0, 0);
    blueprint.push_back(first);
    Component second(1, 1, 1);
    blueprint.push_back(second);
    Component third(2, 2, 2);
    blueprint.push_back(third);

    bot = new BotController(n);
    robotino = new RobotinoController(n);
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

    // START MISCELLANEOUS
    ini_file.beginGroup("MISC");

    // flag that says if the environment is adversarial;
    // if true (non zero) we use camera everything to update workbench state
    target_name.str("");
    target_name << "ADVERSARIAL_ENV";
    ADVERSARIAL_ENV = (bool)ini_file.value(target_name.str().c_str(), 0).toString().toInt();
    std::cout << "ADVERSARIAL_ENV: "<< ADVERSARIAL_ENV << std::endl;

    ini_file.endGroup();
    // END MISCELLANEOUS
};


bool ProdMaster::initiate_production_state(std::vector<Component>& workbench_state, std::vector<Component>& missing_components) {
    // clear previous data
    workbench_state.clear();
    missing_components.clear();

    bool is_done = true;

    // important: move gripper bot to wait position
    bot->movegripperbot(gripperbot_wait_pos.x, gripperbot_wait_pos.y, gripperbot_wait_pos.z);

    // total slots used in the blueprint; sometimes we might leave the last slot for shuffling purposes
    int TOTAL_SLOTS_USED = blueprint.size();
    bool *occupied = new bool[TOTAL_SLOTS_USED]();

    // getting present components on the workbench
    for (auto i_slot_pos = workbench_slots_cambot.begin(); i_slot_pos != workbench_slots_cambot.end(); ++i_slot_pos) {
        bot->movecambot(i_slot_pos->coord.x, i_slot_pos->coord.y, i_slot_pos->coord.z);

        // because the camera service is very slow
        sleep(1);

        // scan using shape detector
        shape_detect_srvs::shape found_object;
        bool found = shapes_detector->get_center_shape(found_object);

        // slot found empty
         if (!found) {
             // verbose print
             std::cout << "Slot " << i_slot_pos->slot << " is empty!" << std::endl;

             is_done = false;
             missing_components.push_back(Component(blueprint[i_slot_pos->slot]));
             continue;
         }

        // initially every component detected is assumed to have to be removed
        Component cur_comp(found_object.shape, found_object.colour, i_slot_pos->slot, Component::REMOVE);

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

    // verbose printing workbench
    std::cout << "The initiated workbench is: " << std::endl;
    for (auto i_workbench = workbench_state.begin(); i_workbench != workbench_state.end(); ++i_workbench) {
        std::cout << "color: " << i_workbench->color << ", shape: " << i_workbench->shape << ", slot: " << i_workbench->slot << ", move_to: " << i_workbench->move_to << std::endl;
    }

    // cleanup and return
    delete [] occupied;
    return is_done;
};


bool ProdMaster::update_missing(const std::vector<Component>& workbench_state, std::vector<Component>& missing_components) {
    // clear previous data
    missing_components.clear();

    // total slots used in the blueprint; sometimes we might leave the last slot for shuffling purposes
    int TOTAL_SLOTS_USED = blueprint.size();
    bool *missing = new bool[TOTAL_SLOTS_USED]();
    // assume everything missing by default
    std::fill_n(missing, TOTAL_SLOTS_USED, true);

    // find what's not missing
    for (auto i_workbench_slot = workbench_state.begin(); i_workbench_slot != workbench_state.end(); ++i_workbench_slot) {
        // current workbench piece is not meant to be removed (this check is redundant given at this point
        // there is nothing to be removed and all the available pieces are in their right places but oh well!)
        if (i_workbench_slot->move_to != Component::REMOVE) {
            // exclude from missing; note we use the move_to not slot assuming that the piece can still need moving
            missing[i_workbench_slot->move_to] = false;
        }
    }

    // create list of the actually missing components
    for (int i_slot = 0; i_slot < TOTAL_SLOTS_USED; ++i_slot) {
        if (missing[i_slot]) {
            missing_components.push_back(blueprint[i_slot]);
        }
    }

    return (missing_components.size() == 0);
};


void ProdMaster::execute_move_components(std::vector<int>& to_move, std::vector<Component>& workbench_state) {
    // TODO if time permits, try to get movements in a smooth path: 3 or 4 points trajectory

    for (int i_comp = 0; i_comp < to_move.size(); ++i_comp) {
        // go to start position
        bot->movetohome(false, true);

        // initate move
        bot->opengripper();

        // current piece
        auto cur_piece = workbench_state.begin() + to_move[i_comp];

        // move to piece
        Slot piece_slot = workbench_slots_gripperbot[cur_piece->slot];
        bot->movegripperbot(piece_slot.coord.x, piece_slot.coord.y, piece_slot.coord.z);

        // hold the object
        bot->closegripper();

        // remove
        if (cur_piece->move_to == Component::REMOVE) {
            // move to collector
            bot->movegripperbot(collector_slot.coord.x, collector_slot.coord.y, collector_slot.coord.z);

            // remove piece from workbench state
            workbench_state.erase(cur_piece);
        }

        // to move to different location
        else {
            Slot move_to = workbench_slots_gripperbot[cur_piece->move_to];
            bot->movegripperbot(move_to.coord.x, move_to.coord.y, move_to.coord.z);

            // update workbench piece slot
            cur_piece->slot = cur_piece->move_to;
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
        if (workbench_state[i_comp].slot == workbench_state[i_comp].move_to) {
            std::cout << "Object in its place, will not be touched." << std::endl;

            continue;
        }

        // component to be removed
        else if (workbench_state[i_comp].move_to == Component::REMOVE) {
            std::cout << "Object to be removed." << std::endl;

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
    execute_move_components(to_move, workbench_state);

    // TODO: shuffle
};


void ProdMaster::receive_components(std::vector<Component>& workbench_state, std::vector<Component>& missing_components) {
    bool robotino_scan_complete = false;

    while(!robotino_scan_complete) {
        std::vector<shape_detect_srvs::shape> found_objects = shapes_detector->get_shapes();

        if (!found_objects.empty()) {
            // because we expect only one shape to be detected
            shape_detect_srvs::shape cur_item = found_objects[0];

            for (auto i_missing = missing_components.begin(); i_missing != missing_components.end(); ++i_missing) {
                // scanned object matches a missing (required) object
                if (i_missing->color == cur_item.colour && i_missing->shape == cur_item.shape) {
                    // convert to 3D location, get position of object in image to gripperbot base
                    cv::Point3f pick_up_loc = get_position_wrt_gripper(cur_item.xPos, cur_item.yPos);

                    // ask robotino to move to gripper
                    robotino->move_to_gripperbot();

                    // ready to hold
                    bot->opengripper();

                    // move close to object on robotino
                    bot->movegripperbot(pick_up_loc.x, pick_up_loc.y, pick_up_loc.z);

                    // hold the object
                    bot->closegripper();

                    // move to destination on workbench
                    Slot move_to = workbench_slots_gripperbot[i_missing->slot];
                    bot->movegripperbot(move_to.coord.x, move_to.coord.y, move_to.coord.z);

                    // release the object
                    bot->opengripper();

                    // update the workbench
                    Component added_comp(i_missing->shape, i_missing->color, i_missing->slot, i_missing->slot);
                    workbench_state.push_back(added_comp);

                    robotino->move_to_cambot();
                    robotino_scan_complete = robotino->rotate();

                    // remove from missing and stop
                    missing_components.erase(i_missing);
                    break;
                }
            }
        }
    }
};


cv::Point3f ProdMaster::get_position_wrt_gripper(int img_point_x, int img_point_y) {
    float k = 0.395/560.0;

	float xo=0.0; float yo=0.37;
		
		
	float x = (img_point_x-240)*k*-1.f + xo;
	float y = (img_point_y-320)*k + yo;
	float z = 0.23;

    return cv::Point3f(x, y, z);
};


bool ProdMaster::order_components(std::vector<Component>& missing_components) {
    // move cambot and gripper bot to wait position for the robotino
    bot->movecambot(cambot_wait_pos.x, cambot_wait_pos.y, cambot_wait_pos.z);
    bot->movegripperbot(gripperbot_wait_pos.x, gripperbot_wait_pos.y, gripperbot_wait_pos.z);

    std::vector<robotino_controller::shape> shopping_list;
    for (auto i_missing = missing_components.begin(); i_missing != missing_components.end(); ++i_missing) {
        robotino_controller::shape cur_item;

        switch (i_missing->color) {
            // red
            case 0:
                cur_item.color = "red";
                break;
                // blue
            case 1:
                cur_item.color = "blue";
                break;
                // green
            case 2:
                cur_item.color = "green";
                break;
                // yellow
            case 3:
                cur_item.color = "yellow";
                break;
            default:
                std::cout << "Missing component has unknown color: " << i_missing->color << ". Will be skipped." << std::endl;
                continue;
        }

        switch (i_missing->shape) {
            // square
            case 0:
                cur_item.shape = "quad";
                break;
                // circle
            case 1:
                cur_item.shape = "circle";
                break;
                // triangle
            case 2:
                cur_item.shape = "triangle";
                break;
            default:
                std::cout << "Missing component has unknown shape: " << i_missing->shape << ". Will be skipped." << std::endl;
                continue;
        }

        shopping_list.push_back(cur_item);
    }

    return robotino->fetch(shopping_list);
};


int ProdMaster::produce() {
    bool is_done = false;
    std::vector<Component> workbench_state;
    std::vector<Component> missing_components;

    // printing blueprint
    std::cout << "The blueprint is: " << std::endl;
    for (auto i_blueprint = blueprint.begin(); i_blueprint != blueprint.end(); ++i_blueprint) {
        std::cout << "color: " << i_blueprint->color << ", shape: " << i_blueprint->shape << ", slot: " << i_blueprint->slot << std::endl;
    }

    // get state of the workbench by camera detection
    is_done = initiate_production_state(workbench_state, missing_components);

    // rearranging involves moving components to their correct positions
    rearrange_workbench(workbench_state);

    // until production done
    while(!is_done) {
        // nothing to get so we are done
        if (missing_components.size() == 0) break;

        // order components - uses a (sync) service to robotino in the robotino_controller
        bool order_success = false;
        while (!order_success) {
            order_success = order_components(missing_components);
        }

        // fetch stuff from robotino
        receive_components(workbench_state, missing_components);

        if (!ADVERSARIAL_ENV) {
            // assuming the state of the workbench is only changed by the gripperbot and that it works perfectly
            is_done = update_missing(workbench_state, missing_components);
        }
        else {
            // here we use detection to update state again
            is_done = initiate_production_state(workbench_state, missing_components);

            // rearranging involves moving components to their correct positions
            rearrange_workbench(workbench_state);
        }
    }

    // done
    return 0;
};


ProdMaster::~ProdMaster() {
    delete bot;
    delete shapes_detector;
};
