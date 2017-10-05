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

    // build the experiment
    build_experiment();

    // set the current trial index
    for (int i = 0; i < all_trial_tags_.size(); ++i) {
        if (start_trial == all_trial_tags_[i])
            current_trial_index_ = i - 1;
    }

    // Add columns to logger
    init_logs();

    // launch game
    //game.launch();
}

//-----------------------------------------------------------------------------
// START STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_start(const util::NoEventData*) {

    move_to_started_ = false;

    // enable OpenWrist DAQ
    util::print("");
    if (condition_ > 0) {
        ow_daq_->enable();
        if (!ow_daq_->is_enabled()) {
            event(ST_STOP);
            return;
        }
    }

    // update Unity UI
    if (current_trial_index_ == -1) {
        menu_msg.write_message("begin");
    }
    else {
        menu_msg.write_message("resume");
    }

    // wait for UI response 
    util::print("\nWaiting for subject to enable haptic device.");
    while (menu_msg.read_message() != "enable_go" && !auto_stop_ && !manual_stop_) {
        if (condition_ > 0)
            step_system_ui();
        manual_stop_ = check_stop();
        if (auto_stop_ || manual_stop_) {
            event(ST_STOP);
            return;
        }
    }
    
    // enable haptic device
    if (condition_ == 1 || condition_ == 2) {
        // enable and pretension CUFF
        cuff_.enable();
        if (!cuff_.is_enabled()) {
            event(ST_STOP);
            return;
        }
        cuff_.pretension(cuff_normal_force_, offset, scaling_factor);
        cinch_cuff();
    }    
    else if (condition_ == 3) {
        // enable MahiExo-II DAQ
    }

    menu_msg.write_message("enable_done");

    // wait for UI response
    util::print("\nWaiting for subject to start experiment");
    while (menu_msg.read_message() != "start" && !auto_stop_ && !manual_stop_) {
        if (condition_ > 0)
            step_system_ui();
        manual_stop_ = check_stop();
        if (auto_stop_ || manual_stop_) {
            event(ST_STOP);
            return;
        }
    }

    // start the simulation and clock
    pendulum_.reset();
    clock_.start();
    if (condition_ > 0) {
        ow_.enable();
        ow_daq_->start_watchdog(0.1);
    }

    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// FAMILIARIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_familiarization(const util::NoEventData*) {

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[FAMILIARIZATION] && !manual_stop_ && !auto_stop_) {
        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[FAMILIARIZATION] };
        timer_.write(timer_data_);
        // step system
        step_system_play();
        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2) {
            step_cuff();
        }
        // log data
        log_step();
        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // show the UI
    menu_msg.write_message("fam_done");

    // disable openwrist
    if (condition_ > 0) {
        ow_.disable();
        ow_daq_->stop_watchdog();
    }

    // wait for UI response
    util::print("\nWaiting for subject to start training.");
    while (menu_msg.read_message() != "train" && !auto_stop_ && !manual_stop_) {
        if (condition_ > 0)
            step_system_ui();
        manual_stop_ = check_stop();
    }

    // renable openwrist
    if (condition_ > 0) {
        ow_.enable();
        ow_daq_->start_watchdog(0.1);
    }
    move_to_started_ = false;

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// TRAINING STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_training(const util::NoEventData*) {

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[TRAINING] && !manual_stop_ && !auto_stop_) {

        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[TRAINING] };
        timer_.write(timer_data_);

        // step OpenWrist and Pendulum
        step_system_play();

        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2) {
            step_cuff();
        }

        // step ME-II guidance
        if (condition_ == 3) {

        }        

        // log data
        log_step();

        // check for stop input
        manual_stop_ = check_stop();

        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// BREAK STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_break(const util::NoEventData*) {

    // disable hardware
    if (condition_ > 0) {
        ow_.disable();
        ow_daq_->stop_watchdog();
    }

    if (condition_ == 1 || condition_ == 2)
        cuff_.disable();

    if (condition_ == 3) {
        // disable ME-II
    }

    // display UI
    menu_msg.write_message("break_begin");

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[BREAK] && !manual_stop_ && !auto_stop_) {
        if (condition_ > 0) {
            step_system_ui();
        }
        // update countdown timer
        timer_data_ = { clock_.time(), length_trials_[BREAK] };
        timer_.write(timer_data_);
        // check for stop input
        manual_stop_ = check_stop();
        // check for subject break end
        if (menu_msg.read_message() == "break_end")
            break;
        // wait for the next clock cycle
        clock_.hybrid_wait();
    }

    menu_msg.write_message("break_end");

    // transition to the next state
    if (manual_stop_ || auto_stop_)
        event(ST_STOP);
    else
        event(ST_START);
}

//-----------------------------------------------------------------------------
// GENERALIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidanceV2::sf_generalization(const util::NoEventData*) {

    // start the control loop
    clock_.start();
    while (clock_.time() < length_trials_[GENERALIZATION] && !manual_stop_ && !auto_stop_) {
        // update countdown timer

        timer_data_ = { clock_.time(), length_trials_[GENERALIZATION] };
        timer_.write(timer_data_);
        
        // step OpenWrist and Pendulum
        step_system_play();

        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2) {
            step_cuff();
        }

        // log data
        log_step();
        // check for stop input
        manual_stop_ = check_stop();
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

    // save the data log from the last trial
    if (trials_started_) {
        log_trial();
    }

    // start a new tiral if there is one or stop hasn't been requested
    if (current_trial_index_ < num_trials_total_ - 1 && !manual_stop_ && !auto_stop_) {

        // set flag
        trials_started_ = true;

        // increment the trial;
        current_trial_index_ += 1;

        // reset scores
        player_score_ = 0;
        expert_score_ = 0;

        // reset expert
        expert_angle_ = 0.0;

        // set the current trajectory
        traj_ = all_trajs_[current_trial_index_];

        // special case for G1-1 (gross code, fix later)
        if (all_trial_tags_[current_trial_index_] == "G1-1") {
            if (condition_ > 0) {
                ow_.disable();
                ow_daq_->stop_watchdog();
            }

            // display UI
            menu_msg.write_message("gen_begin");

            util::print("\nWaiting for subject to start generalization");
            while (menu_msg.read_message() != "gen" && !auto_stop_ && !manual_stop_) {
                if (condition_ > 0)
                    step_system_ui();
                manual_stop_ = check_stop();
            }

            // renable openwrist
            if (condition_ > 0) {
                ow_.enable();
                ow_daq_->start_watchdog(0.1);
            }
            move_to_started_ = false;
        }

        // write out UI information
        trial_.write_message(all_trial_names_[current_trial_index_]);
        traj_name_.write_message(traj_.name_);
        std::array<double, 2> timer = { 0, length_trials_[trials_block_types_[current_trial_index_]] };
        timer_.write(timer);



        
        // wait for user to confirm the trial while rendering pendulum
        ui_msg.write_message("show_reset");
        reset_timer_.write_message("reset");
        if (all_trial_tags_[current_trial_index_] != "B1-1") {
            print("\nNEXT TRIAL: <" + all_trial_tags_[current_trial_index_] + ">. TRAJECTORY: " + traj_.name_ + ". Waiting for subject to confirm trial.");
            while (!confirmed_ && !manual_stop_ && !auto_stop_) {

                // step system 
                step_system_idle();

                // check  if resetters triggered
                if (((std::abs(player_angle_) > reset_angle_window_) && !reset_triggered_) || util::Input::is_key_pressed(util::Input::R, false)) {
                    reset_triggered_ = true;
                    ui_msg.write_message("show_confirm");
                    // reset scores
                    max_score_ = length_trials_[trials_block_types_[current_trial_index_]] * clock_.frequency_ * error_window_;
                    player_score_ = 0;
                    expert_score_ = 0;
                    high_score_ = high_score_records_[traj_.name_];
                    scores_data_ = { player_score_, expert_score_, high_score_, max_score_ };
                    scores_.write(scores_data_);
                    score_msg.write_message("reset_score");
                }

                // if reset triggered, check if player is confirming
                double confirm_speed = 1.0 / confirm_length_;
                if (reset_triggered_ && (std::abs(player_angle_) < confirm_angle_window_)) {
                    confirmation_percent_ += confirm_speed * clock_.delta_time_;
                }
                else if (reset_triggered_) {
                    confirmation_percent_ -= confirm_speed * clock_.delta_time_;
                }
                confirmation_percent_ = math::saturate(confirmation_percent_, 1.0, 0.0);
                error_angle_ = (1.0 - confirmation_percent_) * error_window_; // charging effect
                confirmer.write(confirmation_percent_);

                if (confirmation_percent_ == 1.0) {
                    reset_triggered_ = false;
                    confirmation_percent_ = 0.0;
                    confirmed_ = true;
                }

                // update angles
                angles_data_ = { player_angle_ , expert_angle_ , error_angle_ };
                angles_.write(angles_data_);

                // wait for the next clock cycle
                clock_.hybrid_wait();
            }
            if (manual_stop_ || auto_stop_) {
                event(ST_STOP);
                return;
            }
        }

        // print console message for conductor
        print("STARTING TRIAL: <" + all_trial_tags_[current_trial_index_] + ">. TRAJECTORY: " + traj_.name_);
        print("Press ESC or CTRL+SPACE to terminate the experiment.");

        // reset user confirmatin triggers
        reset_triggered_ = false;
        confirmed_ = false;

        // reset the pendlum
        pendulum_.reset();

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
    
    if (current_trial_index_ < 0) {
        print("\nExperiment terminated during startup. Disabling hardware.");
        menu_msg.write_message("abort");
    }
    else if (current_trial_index_ < num_trials_total_ - 1) {
        print("\nExperiment aborted during trial " + util::namify(all_trial_tags_[current_trial_index_]) + ". Disabling hardware.");
        menu_msg.write_message("abort");
    }
    else {
        print("\nExperiment completed. Disabling hardware and saving data log.");
        menu_msg.write_message("complete");
    }


    if (condition_ > 0)
    {
        ow_.disable();
        ow_daq_->stop_watchdog();
        ow_daq_->disable();
    }

    if (condition_ == 1 || condition_ == 2) {
        cuff_.disable();
    }

    if (condition_ == 3) {

    }

    main_log_.save_data(directory_, directory_, true);
    main_log_.wait_for_save();
}

//-----------------------------------------------------------------------------
// UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::init_logs() {
    main_log_
        .add_col("Trial No.")
        .add_col("Block Type")
        .add_col("Amplitude [deg]")
        .add_col("A")
        .add_col("B")
        .add_col("C")
        .add_col("a [Hz]")
        .add_col("b [Hz]")
        .add_col("c [Hz]")
        .add_col("Player Score")
        .add_col("Expert Score")
        .add_col("Abs. Error Mean")
        .add_col("Abs. Error Std. Dev.");

    trial_log_
        .add_col("Time [s]")
        .add_col("Player Angle [deg]")
        .add_col("Expert Angle [deg]")
        .add_col("Error [deg]")
        .add_col("OW PS Position [rad]")
        .add_col("OW PS Velocity [rad/s]")
        .add_col("OW PS Total Torque [Nm]")
        .add_col("OW PS Compensation Torque [Nm]")
        .add_col("OW PS Task Torque [Nm]")
        .add_col("CUFF Ref. Motor Position 1")
        .add_col("CUFF Ref. Motor Position 2")
        .add_col("CUFF Act. Motor Position 1")
        .add_col("CUFF Act. Motor Position 2")
        .add_col("CUFF Act. Motor Current 1")
        .add_col("CUFF ACt. Motor Current 2");
}

void HapticGuidanceV2::log_trial() {
    std::vector<double> row;
    row.reserve(main_log_.get_col_count());

    row.push_back(static_cast<double>(current_trial_index_));
    row.push_back(static_cast<double>(trials_block_types_[current_trial_index_]));
    row.push_back(traj_.amp_);
    row.push_back(traj_.A_);
    row.push_back(traj_.B_);
    row.push_back(traj_.C_);
    row.push_back(traj_.a_);
    row.push_back(traj_.b_);
    row.push_back(traj_.c_);
    row.push_back(player_score_);
    row.push_back(expert_score_);

    std::vector<double> abs_error = math::abs_vec(trial_log_.get_col("Error [deg]"));

    row.push_back(math::mean(abs_error));
    row.push_back(math::stddev_s(abs_error));

    main_log_.add_row(row);

    std::string filename = stringify(current_trial_index_) + "_" + all_trial_tags_[current_trial_index_] + "_" + traj_.name_;
    std::string directory = directory_ + "\\_" + all_trial_blocks_[current_trial_index_];

    trial_log_.save_and_clear_data(filename, directory, true);
}

void HapticGuidanceV2::log_step() {
    std::vector<double> row;
    row.reserve(trial_log_.get_col_count());

    row.push_back(clock_.time());
    row.push_back(player_angle_);
    row.push_back(expert_angle_);
    row.push_back(error_angle_);
    row.push_back(ow_.joints_[0]->get_position());
    row.push_back(ow_.joints_[0]->get_velocity());
    row.push_back(ps_total_torque_);
    row.push_back(ps_comp_torque_);
    row.push_back(-pendulum_.Tau[0]);
    row.push_back(static_cast<double>(cuff_ref_pos_1_));
    row.push_back(static_cast<double>(cuff_ref_pos_2_));
    row.push_back(static_cast<double>(cuff_act_pos_1_));
    row.push_back(static_cast<double>(cuff_act_pos_2_));
    row.push_back(static_cast<double>(cuff_act_current_1_));
    row.push_back(static_cast<double>(cuff_act_current_2_));

    trial_log_.add_row(row);
}

void HapticGuidanceV2::wait_for_input() {
    util::Input::wait_for_key(util::Input::Key::Space, false);
}

bool HapticGuidanceV2::check_stop() {
    return ( util::Input::is_key_pressed(util::Input::Escape) || (util::Input::is_key_pressed(util::Input::LControl) && util::Input::is_key_pressed(util::Input::Space)));
}

void HapticGuidanceV2::build_experiment() {
    
    std::vector<Trajectory> trajs_training_full;
    trajs_training_full.reserve(12);
    trajs_training_full.insert(trajs_training_full.end(), trajs_training_.begin(), trajs_training_.end());
    trajs_training_full.insert(trajs_training_full.end(), trajs_training_.begin(), trajs_training_.end());
    trajs_training_full.insert(trajs_training_full.end(), trajs_training_.begin(), trajs_training_.end());
    trajs_training_full.insert(trajs_training_full.end(), trajs_training_.begin(), trajs_training_.end());

    std::vector<Trajectory> trajs_generalization_full;
    trajs_generalization_full.reserve(12);
    trajs_generalization_full.insert(trajs_generalization_full.end(), trajs_generalization_.begin(), trajs_generalization_.end());
    trajs_generalization_full.insert(trajs_generalization_full.end(), trajs_generalization_.begin(), trajs_generalization_.end());
    trajs_generalization_full.insert(trajs_generalization_full.end(), trajs_generalization_.begin(), trajs_generalization_.end());
    trajs_generalization_full.insert(trajs_generalization_full.end(), trajs_generalization_.begin(), trajs_generalization_.end());

    // for every block
    for (auto it = block_order_.begin(); it != block_order_.end(); ++it) {
        // increment the number of blocks of this type of block
        num_blocks_[*it] += 1;

        // generate a set of temporary traj params equal to num trails in the block
        std::vector<Trajectory> trajs_temp;
        if (*it == FAMILIARIZATION)
            trajs_temp.push_back(traj_familiarization_);
        else if (*it == BREAK)
            trajs_temp.push_back(traj_break_);
        else if (*it == TRAINING)
            trajs_temp = trajs_training_full;
        else if (*it == GENERALIZATION)
            trajs_temp = trajs_generalization_full;

        // shuffle the temp traj params
        std::random_shuffle(trajs_temp.begin(), trajs_temp.end());

        // for each trial in this block type
        for (int i = 0; i < num_trials_[*it]; i++) {
            trials_block_types_.push_back(*it);
            all_trial_blocks_.push_back(block_names_[*it]);
            all_trial_tags_.push_back(block_tags_[*it] + std::to_string(num_blocks_[*it]) + "-" + std::to_string(i + 1));
            all_trial_names_.push_back(block_names_[*it] + " " + std::to_string(num_blocks_[*it]) + "-" + std::to_string(i + 1));
            num_trials_total_ += 1;
            all_trajs_.push_back(trajs_temp[i]);
            high_score_records_[trajs_temp[i].name_] = 0.0; // init high scores
        }       

    }

    // print results of build
    //for (int i = 0; i < num_trials_total_; ++i) {
    //    util::print(all_trial_tags_[i] + " " + all_trial_blocks_[i] + " " + all_trajs_[i].name_);
    //}

}

//-----------------------------------------------------------------------------
// TRAJECTORY UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::cinch_cuff() {
    cuff_.set_motor_positions(offset[0], offset[1], true);
}

void HapticGuidanceV2::release_cuff() {
    cuff_.set_motor_positions(offset[0] + 1000, offset[1] - 1000, true);
}


void HapticGuidanceV2::step_cuff() {

    // set motor positions to offsets
    cuff_ref_pos_1_ = offset[0];
    cuff_ref_pos_2_ = offset[1];

    // feedforward mechanism
    cuff_ref_pos_1_ += (short int)((expert_angle_)* cuff_ff_gain_);
    cuff_ref_pos_2_ += (short int)((expert_angle_)* cuff_ff_gain_);

    // feedback mechanism
    //cuff_pos_1_ -= (short int)((std::abs(error_))* cuff_fb_gain_); // squeeze right side
    //cuff_pos_2_ += (short int)((std::abs(error_))* cuff_fb_gain_); // squeeze left side

    // set motor positions
    cuff_.set_motor_positions(cuff_ref_pos_1_, cuff_ref_pos_2_, true);

    // get current CUFF state
    cuff_.get_motor_positions(cuff_act_pos_1_, cuff_act_pos_2_, true);
    cuff_.get_motor_currents(cuff_act_current_1_, cuff_act_current_2_, true);
}


void HapticGuidanceV2::update_scores() {

    // calculate max score
    max_score_ = length_trials_[trials_block_types_[current_trial_index_]] * clock_.frequency_ * error_window_;

    // update current scores
    player_score_ += math::saturate(error_window_ - std::abs(error_angle_), error_window_, 0.0);
    expert_score_ += (error_window_);

    // update high scores
    if (player_score_ > high_score_records_[traj_.name_])
        high_score_records_[traj_.name_] = player_score_;

    high_score_ = high_score_records_[traj_.name_];

    scores_data_ = { player_score_, expert_score_, high_score_, max_score_ };
    scores_.write(scores_data_);
}

//-----------------------------------------------------------------------------
// CONTROL LOOP UTILS
//-----------------------------------------------------------------------------

void HapticGuidanceV2::step_system_ui() {

    ow_daq_->read_all();
    ow_.get_joint_positions();
    ow_.update_state_map();

    if (ow_.check_all_joint_limits()) {
        auto_stop_ = true;
    }
}

void HapticGuidanceV2::step_system_play() {

    // read and reload DAQ
    if (condition_ > 0) {
        ow_daq_->reload_watchdog();
        ow_daq_->read_all();
    }

    // step the pendulum simuation
    pendulum_.step_simulation(clock_.time(), ow_.joints_[0]->get_position(), ow_.joints_[0]->get_velocity());

    // update player angle
    player_angle_ = ow_.joints_[0]->get_position() * math::RAD2DEG;

    // update expert angle
    expert_angle_ = traj_.eval(clock_.time());

    // compute anglular error / write out angles
    error_angle_ = player_angle_ - expert_angle_;
    angles_data_ = { player_angle_ , expert_angle_ , error_angle_ };
    angles_.write(angles_data_);

    // compute scores
    update_scores();

    if (condition_ > 0) {
        // compute OpenWrist PS torque
        ps_comp_torque_ = ow_.compute_gravity_compensation(0) + 0.75 * ow_.compute_friction_compensation(0);
        ps_total_torque_ = ps_comp_torque_ - pendulum_.Tau[0];        
        ow_.joints_[0]->set_torque(ps_total_torque_);

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
        auto_stop_ = true;
    }

    // update OpenWrist state
    ow_.update_state_map();

    // write the DAQ
    if (condition_ > 0)
        ow_daq_->write_all();

    // check for manual stop from conductor
    manual_stop_ = check_stop();
}

void HapticGuidanceV2::step_system_idle() {

    // read and reload DAQ
    if (condition_ > 0) {
        ow_daq_->reload_watchdog();
        ow_daq_->read_all();
    }

    // step the pendulum simuation
    pendulum_.step_simulation(clock_.time(), ow_.joints_[0]->get_position(), ow_.joints_[0]->get_velocity());

    // update player angle
    player_angle_ = ow_.joints_[0]->get_position() * math::RAD2DEG;

    if (condition_ > 0) {
        // compute OpenWrist PS torque
        ps_comp_torque_ = ow_.compute_gravity_compensation(0) + 0.75 * ow_.compute_friction_compensation(0);
        ps_total_torque_ = ps_comp_torque_ - pendulum_.Tau[0];
        ow_.joints_[0]->set_torque(ps_total_torque_);

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
        auto_stop_ = true;
    }

    // update OpenWrist state
    ow_.update_state_map();

    // write the DAQ
    if (condition_ > 0)
        ow_daq_->write_all();

    // check for manual stop from conductor
    manual_stop_ = check_stop();

}