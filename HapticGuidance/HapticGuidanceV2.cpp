#include "HapticGuidanceV2.h"
#include "Input.h"
#include <random>

using namespace mel;
using mel::util::print;
using mel::util::stringify;

//-----------------------------------------------------------------------------
// CONSTRUCTOR
//-----------------------------------------------------------------------------

HapticGuidanceV2::HapticGuidanceV2(util::Clock& clock, core::Daq* ow_daq, exo::OpenWrist& open_wrist, Cuff& cuff,
    int subject_number, int condition, std::string start_trial) :
    StateMachine(8),
    clock_(clock),
    ow_daq_(ow_daq),
    ow_(open_wrist),
    //meii_daq_(meii_daq),
    //meii_(meii),
    cuff_(cuff),
    subject_number_(subject_number),
    condition_(condition)
{
    // create subject folder
    if (subject_number < 10)
        directory_ = "S0" + stringify(subject_number) + "_C" + stringify(condition);
    else
        directory_ = "S" + stringify(subject_number) + "_C" + stringify(condition);

    // seed random number generator with subject num
    srand(subject_number);

    // build the experiment order
    build_experiment();

    // set the current trial index
    for (int i = 0; i < trials_tag_names_.size(); ++i) {
        if (start_trial == trials_tag_names_[i])
            current_trial_index_ = i - 1;
    }

    // Add columns to logger
    log_.add_col("Time [s]").add_col("Amplitude [px]").add_col("Sin Freq. [Hz]").add_col("Cos Freq. [Hz]")
        .add_col("Exp_x [px]").add_col("Exp_y [px]").add_col("Angular Error [rad]").add_col("OW PS Position [rad]").add_col("OW PS Velocity [rad/s]").add_col("OW PS Total Torque [Nm]")
        .add_col("OW PS Compensation Torque [Nm]").add_col("OW PS Task Torque [Nm]")
        .add_col("CUFF Motor Position 1").add_col("CUFF Motor Position 2");

    // show/hide Unity elements
    update_visible(
        true,  // backround
        true,  // pendulum
        false, // trajectory region
        false, // trajectory center
        false,  // expert
        true,  // radius
        false, // stars
        false   // UI
    );
}

//-----------------------------------------------------------------------------
// START STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_start(const util::NoEventData*) {

    // launch game
    game.launch();

    // enable OpenWrist DAQ
    if (condition_ > 0) {
        util::Input::acknowledge("\nPress Enter to enable OpenWrist DAQ <" + ow_daq_->name_ + ">.", util::Input::Key::Enter);
        ow_daq_->enable();
        if (!ow_daq_->is_enabled()) {
            event(ST_STOP);
            return;
        }
    }

    // enable and pretension CUFF
    if (condition_ == 1 || condition_ == 2) {
        util::Input::acknowledge("\nPress Enter to enable and pretension CUFF" + ow_daq_->name_ + ">.", util::Input::Key::Enter);
        cuff_.enable();
        if (!cuff_.is_enabled()) {
            event(ST_STOP);
            return;
        }
        cuff_.pretension(cuff_normal_force_, offset, scaling_factor);
        release_cuff();
    }

    // enable MahiExo-II DAQ
    if (condition_ == 3) {
        /*
        util::print("\nPress Enter to enable MahiExo-II Daq <" + ow_daq_->name_ + ">.");
        util::Input::wait_for_key_press(util::Input::Key::Return);
        meii_daq_->enable();
        if (!meii_daq_->is_enabled()) {
        event(ST_STOP);
        return;
        }

        // check DAQ behavior for safety
        meii_daq_->read_all();
        meii_.update_kinematics();
        if (meii_.check_all_joint_limits()) {
        event(ST_STOP);
        return;
        }
        */
    }

    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// FAMILIARIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_familiarization(const util::NoEventData*) {

    // show/hide Unity elements
    update_visible(
        true,  // backround
        true,  // pendulum
        false, // trajectory region
        false, // trajectory center
        true,  // expert
        true,  // radius
        false, // stars
        true   // UI
    );

    // reset move to flag
    move_to_started_ = false;

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[FAMILIARIZATION] && !stop_) {
        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[FAMILIARIZATION] };
        timer_.write(timer_data_);
        // step system
        step_system();
        // log data
        log_step();
        // check for stop input
        stop_ = check_stop();
        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// TRAINING STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_training(const util::NoEventData*) {

    // show/hide Unity elements
    update_visible(
        true,  // backround
        true,  // pendulum
        false, // trajectory region
        false, // trajectory center
        false,  // expert
        true,  // radius
        false, // stars
        true   // UI
    );

    // reset move to flag
    move_to_started_ = false;

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[TRAINING] && !stop_) {

        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[TRAINING] };
        timer_.write(timer_data_);

        // step OpenWrist and Pendulum
        step_system();

        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2) {
            step_cuff();
        }

        // step ME-II guidance
        if (condition_ == 3) {
            /*
            meii_daq_->reload_watchdog();
            meii_daq_->read_all();

            meii_.update_kinematics();
            meii_.check_all_joint_limits();

            double_vec meii_torques(5, 0.0);
            //double meii_torque = meii_.robot_joint_pd_controllers_[1].calculate(open_wrist_.joints_[0]->get_position(), meii_.get_anatomical_joint_position(1), 0, meii_.get_anatomical_joint_velocity(1));
            //meii_.joints_[1]->set_torque(meii_torque);

            for (auto i = 0; i < 5; ++i) {
            if (i != 1) {
            if (!move_started_meii) {
            meii_torques[i] = meii_.robot_joint_pd_controllers_[i].move_to_hold(neutral_pos_meii_[i], meii_.joints_[i]->get_position(), speed_meii_[i], meii_.joints_[i]->get_velocity(), clock_.delta_time_, pos_tol_meii_[i], true);
            move_started_meii = true;
            }
            else {
            meii_torques[i] = meii_.robot_joint_pd_controllers_[i].move_to_hold(neutral_pos_meii_[i], meii_.joints_[i]->get_position(), speed_meii_[i], meii_.joints_[i]->get_velocity(), clock_.delta_time_, pos_tol_meii_[i], false);
            }
            }
            else {
            meii_torques[i] = 7.0 * traj_error_;
            }
            }
            meii_daq_->write_all();
            */
        }        

        // log data
        log_step();

        // check for stop input
        stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // release CUFF
    if (condition_ == 2 || condition_ == 3)
        release_cuff();


    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// BREAK STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_break(const util::NoEventData*) {

    // show/hide Unity elements
    update_visible(
        true,  // backround
        true,  // pendulum
        false, // trajectory region
        false, // trajectory center
        false,  // expert
        true,  // radius
        false, // stars
        true   // UI
    );

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[BREAK] && !stop_) {
        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[BREAK] };
        timer_.write(timer_data_);
        // check for stop input
        stop_ = check_stop();
        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// GENERALIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_generalization(const util::NoEventData*) {

    // show/hide Unity elements
    update_visible(
        true,  // backround
        true,  // pendulum
        false, // trajectory region
        false, // trajectory center
        false,  // expert
        true,  // radius
        false, // stars
        true   // UI
    );

    // reset move to flag
    move_to_started_ = false;

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[GENERALIZATION] && !stop_) {
        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[GENERALIZATION] };
        timer_.write(timer_data_);
        // step system and devices
        if (condition_ >= 0) {
            step_system();
        }
        // log data
        log_step();
        // check for stop input
        stop_ = check_stop();
        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to the next state
    event(ST_TRANSITION);

}

//-----------------------------------------------------------------------------
// TRANSITION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_transition(const util::NoEventData*) {

    // suspend hardware
    if (condition_ > 0) {
        ow_.disable();
        ow_daq_->stop_watchdog();
    }

    if (condition_ == 3) {
        //meii_.disable();
        //meii_daq_->stop_watchdog();
    }

    // save the data log from the last trial
    if (trials_started_) {
        log_.save_and_clear_data(trials_tag_names_[current_trial_index_], directory_ + "\\_" + trials_block_names_[current_trial_index_], true);
    }

    // show/hide Unity elements
    update_visible(true, true, false, false, false, false, true, true);

    // start a new tiral if there is one or stop hasn't been requested
    if (current_trial_index_ < num_trials_total_ - 1 && !stop_) {

        // increment the trial;
        current_trial_index_ += 1;

        // delay or ask for input
        //if (TRIALS_TAG_NAMES_[current_trial_index_] == "F1-1" ||
        //    TRIALS_TAG_NAMES_[current_trial_index_] == "T1-1" ||
        //    TRIALS_TAG_NAMES_[current_trial_index_] == "G1-1")
        //{
            trial_.write_message(trials_tag_names_[current_trial_index_]);
            print("\nNEXT TRIAL: <" + trials_tag_names_[current_trial_index_] + ">. Press SPACE to begin.");
            while (!util::Input::is_key_pressed(util::Input::Space)) {
                stop_ = check_stop();
                if (stop_) {
                    event(ST_STOP);
                    return;
                }
            }
        //}
        //else {
        //    util::print("\nNEXT TRIAL: <" + TRIALS_TAG_NAMES_[current_trial_index_] + ">.");
        //}

        //// write wait
        //trial_.write_message("HOLD");
        //clock_.start();
        //while (clock_.time() < 3.0) {
        //    std::array<double, 2> timer = { clock_.time(), 3.0 };
        //    timer_.write(timer);
        //    stop_ = check_stop();
        //    if (stop_) {
        //        event(ST_STOP);
        //        return;
        //    }
        //    clock_.hybrid_wait();
        //}


        // write the trial string out
        trial_.write_message(trials_tag_names_[current_trial_index_]);

        std::array<double, 2> timer = { 0, length_trials_[trials_block_types_[current_trial_index_]] };
        timer_.write(timer);

        // reset the pendlum
        pendulum_.reset();

        // set the trajectory parameters
        traj_param_ = traj_params_[current_trial_index_];

        // print message
        print("STARTING TRIAL: <" + trials_tag_names_[current_trial_index_] + ">." +
            " A = " + stringify(traj_param_.amp_) + " S = " + stringify(traj_param_.sin_) + " C = " + stringify(traj_param_.cos_));
        print("Press ESC or CTRL+C to terminate the experiment.");

        trials_started_ = true;

        // resume hardware
        if (condition_ > 0) {
            ow_.enable();
            ow_daq_->start_watchdog(0.1);
        }

        if (condition_ == 3) {
            //meii_.enable();
            //meii_daq_->start_watchdog(0.1);
        }

        // transition to the next state
        if (trials_block_types_[current_trial_index_] == FAMILIARIZATION)
            event(ST_FAMILIARIZATION);
        else if (trials_block_types_[current_trial_index_] == TRAINING)
            event(ST_TRAINING);
        else if (trials_block_types_[current_trial_index_] == BREAK)
            event(ST_BREAK);
        else if (trials_block_types_[current_trial_index_] == GENERALIZATION)
            event(ST_GENERALIZATION);
        return;
    }
    else {
        event(ST_STOP);
        return;
    }
}

//-----------------------------------------------------------------------------
// STOP STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_stop(const util::NoEventData*) {
    if (current_trial_index_ < 0)
        print("\nExperiment terminated during startup. Disabling hardware.");
    else if (current_trial_index_ < num_trials_total_ - 1)
        print("\nExperiment terminated during trial " + util::namify(trials_tag_names_[current_trial_index_]) + ". Disabling hardware.");
    else
        print("\nExperiment completed. Disabling hardware.");

    if (condition_ >= 0)
        ow_daq_->disable();

    if (condition_ == 2 || condition_ == 3)
        cuff_.disable();
}

//-----------------------------------------------------------------------------
// UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::log_step() {
    std::vector<double> row;
    row.push_back(clock_.time());
    row.push_back(traj_param_.amp_);
    row.push_back(traj_param_.sin_);
    row.push_back(traj_param_.cos_);
    row.push_back(expert_position_px_[0]);
    row.push_back(expert_position_px_[1]);
    row.push_back(error_);
    row.push_back(ow_.joints_[0]->get_position());
    row.push_back(ow_.joints_[0]->get_velocity());
    row.push_back(ps_total_torque_);
    row.push_back(ps_comp_torque_);
    row.push_back(-pendulum_.Tau[0]);
    row.push_back((double)cuff_pos_1_);
    row.push_back((double)cuff_pos_2_);
    log_.add_row(row);
}

void HapticGuidanceV2::wait_for_input() {
    util::Input::wait_for_key(util::Input::Key::Space);
}

bool HapticGuidanceV2::check_stop() {
    return util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::C));
}

void HapticGuidanceV2::build_experiment() {

    
    std::vector<TrajParam> traj_params_training_full;
    traj_params_training_full.reserve(12);
    traj_params_training_full.insert(traj_params_training_full.end(), traj_params_training_.begin(), traj_params_training_.end());
    traj_params_training_full.insert(traj_params_training_full.end(), traj_params_training_.begin(), traj_params_training_.end());
    traj_params_training_full.insert(traj_params_training_full.end(), traj_params_training_.begin(), traj_params_training_.end());
    traj_params_training_full.insert(traj_params_training_full.end(), traj_params_training_.begin(), traj_params_training_.end());

    // for every block
    for (auto it = block_order_.begin(); it != block_order_.end(); ++it) {
        // increment the number of blocks of this type of block
        num_blocks_[*it] += 1;

        // generate a set of temporary traj params equal to num trails in the block
        std::vector<TrajParam> traj_params_temp;
        if (*it == FAMILIARIZATION)
            traj_params_temp.push_back(traj_param_familiarization_);
        else if (*it == BREAK)
            traj_params_temp.push_back(TrajParam());
        else if (*it == TRAINING)
            traj_params_temp = traj_params_training_full;
        else if (*it == GENERALIZATION)
            traj_params_temp = traj_params_generalization_;

        // shuffle the temp traj params
        std::random_shuffle(traj_params_temp.begin(), traj_params_temp.end());

        // for each trial in this block type
        for (int i = 0; i < num_trials_[*it]; i++) {
            trials_block_types_.push_back(*it);
            trials_block_names_.push_back(block_names_[*it]);
            trials_tag_names_.push_back(block_tags_[*it] + std::to_string(num_blocks_[*it]) + "-" + std::to_string(i + 1));
            num_trials_total_ += 1;
            traj_params_.push_back(traj_params_temp[i]);
        }
    }
}

//-----------------------------------------------------------------------------
// TRAJECTORY UTILS
//-----------------------------------------------------------------------------

double HapticGuidanceV2::trajectory(double time) {
    return traj_param_.amp_ * (std::sin(2.0 * math::PI * traj_param_.sin_ * time) * std::cos(2.0 * math::PI * traj_param_.cos_ * time));
}

void HapticGuidanceV2::update_trajectory(double time) {
    // compute trajectory
    for (int i = 0; i < num_traj_points_; i++) {
        trajectory_y_px_[i] = static_cast<float>(i* spacing_px_ - (num_traj_points_ - 1) * spacing_px_ * 0.5);
        trajectory_x_px_[i] = static_cast<float>(trajectory(trajectory_y_px_[i] * screen_time_ / screen_height_ + time));
    }
    // send trajectory to Unity
    trajectory_x_.write(trajectory_x_px_);
    trajectory_y_.write(trajectory_y_px_);
}

void HapticGuidanceV2::update_expert(double time) {
    expert_angle_ = trajectory(time);
    expert_position_px_[0] = static_cast<float>(length_px_ * cos(expert_angle_ - 90.0* math::DEG2RAD));
    expert_position_px_[1] = static_cast<float>(length_px_ + length_px_ * sin(expert_angle_ - 90.0* math::DEG2RAD));
    exp_pos.write(expert_position_px_);
}

void HapticGuidanceV2::release_cuff() {
    cuff_.set_motor_positions(offset[0] + 1000, offset[1] - 1000, true);
}

//-----------------------------------------------------------------------------
// VISUALIZATION UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::update_visible(bool background, bool pendulum, bool trajectory_region, bool trajectory_center, bool expert, bool radius, bool stars, bool ui) {
    visible_data_ = { 0,0,0,0,0,0,0 };
    if (background)
        visible_data_[0] = 1;
    if (pendulum)
        visible_data_[1] = 1;
    if (trajectory_region)
        visible_data_[2] = 1;
    if (trajectory_center)
        visible_data_[3] = 1;
    if (expert)
        visible_data_[4] = 1;
    if (radius)
        visible_data_[5] = 1;
    if (stars)
        visible_data_[6] = 1;
    if (ui)
        visible_data_[7] = 1;
    unity_.write(visible_data_);
}

//-----------------------------------------------------------------------------
// CONTROL LOOP UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::step_cuff() {

    // set motor positions to offsets
    cuff_pos_1_ = offset[0];
    cuff_pos_2_ = offset[1];

    // feedforward mechanism
    cuff_pos_1_ += (short int)((expert_angle_)* cuff_ff_gain_);
    cuff_pos_2_ += (short int)((expert_angle_)* cuff_ff_gain_);

    // feedback mechanism
    //cuff_pos_1_ -= (short int)((std::abs(error_))* cuff_fb_gain_); // squeeze right side
    //cuff_pos_2_ += (short int)((std::abs(error_))* cuff_fb_gain_); // squeeze left side

    // set motor positions
    cuff_.set_motor_positions(cuff_pos_1_, cuff_pos_2_, true);
}

void HapticGuidanceV2::step_system(double external_torque) {

    // read and reload DAQ
    if (condition_ > 0) {
        ow_daq_->reload_watchdog();
        ow_daq_->read_all();
    }

    // update trajectory
    //update_trajectory(clock_.time());

    // update expert position
    update_expert(clock_.time());

    // step the pendulum simuation
    pendulum_.step_simulation(clock_.time(), ow_.joints_[0]->get_position(), ow_.joints_[0]->get_velocity());

    // compute anglular error / write out angles
    //expert_angle_ = asin((double)expert_position_px_[0] / length_px_);
    error_ = (ow_.joints_[0]->get_position() - expert_angle_);
    angles_data_ = { ow_.joints_[0]->get_position() , expert_angle_ , error_ };
    angles_.write(angles_data_);

    if (condition_ > 0) {
        // compute OpenWrist PS torque
        ow_.joints_[0]->set_torque(external_torque);
        ow_.joints_[0]->add_torque(ow_.compute_gravity_compensation(0));
        ow_.joints_[0]->add_torque(0.75 * ow_.compute_friction_compensation(0));
        ow_.joints_[0]->add_torque(-pendulum_.Tau[0]);

        // compute OpenWrist FE and RU torque
        if (!move_to_started_) {
            ow_.joints_[1]->set_torque(pd1_.move_to_hold(0, ow_.joints_[1]->get_position(),
                move_to_speed_ * math::DEG2RAD, ow_.joints_[1]->get_velocity(),
                clock_.delta_time_, math::DEG2RAD, true));

            ow_.joints_[2]->set_torque(pd2_.move_to_hold(math::DEG2RAD * 0, ow_.joints_[2]->get_position(),
                move_to_speed_ * math::DEG2RAD, ow_.joints_[2]->get_velocity(),
                clock_.delta_time_, math::DEG2RAD, true));

            move_to_started_ = true;
        }
        else {
            ow_.joints_[1]->set_torque(pd1_.move_to_hold(0, ow_.joints_[1]->get_position(),
                move_to_speed_ * math::DEG2RAD, ow_.joints_[1]->get_velocity(),
                clock_.delta_time_, math::DEG2RAD, false));

            ow_.joints_[2]->set_torque(pd2_.move_to_hold(math::DEG2RAD * 0, ow_.joints_[2]->get_position(),
                move_to_speed_ * math::DEG2RAD, ow_.joints_[2]->get_velocity(),
                clock_.delta_time_, math::DEG2RAD, false));
        }
    }

    // check joint limits
    if (ow_.check_all_joint_limits()) {
        stop_ = true;
    }

    // update OpenWrist state
    ow_.update_state_map();

    // write the DAQ
    if (condition_ > 0)
        ow_daq_->write_all();
}