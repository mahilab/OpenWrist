#include "HapticGuidance.hpp"
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <random>
#include <MEL/Utility/Console.hpp>


//-----------------------------------------------------------------------------
// CONSTRUCTOR
//-----------------------------------------------------------------------------

HapticGuidance::HapticGuidance(Q8Usb& q8_ow, OpenWrist& ow, Cuff& cuff, int subject_number, int condition, std::string start_trial) :
    StateMachine(8),
    timer_(hertz(1000)),
    q8_ow_(q8_ow),
    ow_(ow),
    // meii_daq_(meii_daq),
    // meii_(meii),
    cuff_(cuff),
    subject_number_(subject_number),
    condition_(condition),
    ms_timer_("timer"),
    ms_angles_("angles"),
    ms_scores_("scores"),
    ms_confirmer_("confirmer"),
    ms_reset_timer_("reset_timer"),
    ms_trial_("trial"),
    ms_traj_name_("traj_name"),
    ms_ui_msg_("ui_msg"),
    ms_score_msg_("score_msg"),
    ms_menu_msg_("menug_msg"),
    timer_data_(2, 0.0),
    angles_data_(3, 0.0),
    scores_data_(4, 0.0)
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

    // print start message to console
    print("\nSubject Number: " + std::to_string(subject_number_));
    print("Condition:      " + std::to_string(condition_));
    print("Start Trial:    " + all_trial_tags_[current_trial_index_+1]);
}

//-----------------------------------------------------------------------------
// START STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_start(const NoEventData*) {

    pd1_.reset_move_to_hold();
    pd2_.reset_move_to_hold();

    // meii_.robot_joint_pd_controllers_[0].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[1].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[2].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[3].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[4].reset_move_to_hold();

    // enable OpenWrist DAQ
    print("");
    if (condition_ > 0) {
        q8_ow_.enable();
        if (!q8_ow_.is_enabled()) {
            event(ST_STOP);
            return;
        }
    }

    // update Unity UI
    if (current_trial_index_ == -1) {
        ms_menu_msg_.write_message("begin");
    }
    else {
        ms_menu_msg_.write_message("resume");
    }

    // wait for UI response
    print("\nWaiting for subject to enable haptic device.");
    while (ms_menu_msg_.read_message() != "enable_go" && !auto_stop_ && !manual_stop_) {
        if (condition_ > 0)
            step_system_ui();
        manual_stop_ = check_stop();
        if (auto_stop_ || manual_stop_) {
            event(ST_STOP);
            return;
        }
    }

    // enable haptic device
    if (condition_ == 1 || condition_ == 2 || condition_ == 3) {
        // enable and pretension CUFF
        cuff_.enable();
        if (!cuff_.is_enabled()) {
            event(ST_STOP);
            return;
        }
        cuff_.pretension(cuff_normal_force_, offset, scaling_factor);
        cinch_cuff();
    }
    // else if (condition_ == 4) {
    //     meii_daq_->enable();
    //     if (!meii_daq_->is_enabled()) {
    //         event(ST_STOP);
    //         return;
    //     }
    // }

    ms_menu_msg_.write_message("enable_done");

    // wait for UI response
    print("\nWaiting for subject to start experiment");
    while (ms_menu_msg_.read_message() != "start" && !auto_stop_ && !manual_stop_) {
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
    timer_.restart();
    if (condition_ > 0) {
        ow_.enable();
        q8_ow_.watchdog.start();
    }
    // if (condition_ == 4) {
    //     meii_.enable();
    //     meii_daq_->start_watchdog(0.1);
    // }

    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// FAMILIARIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_familiarization(const NoEventData*) {

    // start the control loop
    timer_.restart();
    while (timer_.get_elapsed_time().as_seconds() < length_trials_[FAMILIARIZATION] && !manual_stop_ && !auto_stop_) {
        // update countdown timer
        timer_data_ = { timer_.get_elapsed_time().as_seconds(), length_trials_[FAMILIARIZATION] };
        ms_timer_.write_data(timer_data_);
        // step system
        step_system_play();
        // step guidance
        if (condition_ == 1 || condition_ == 2 || condition_ == 3) {
            step_cuff();
        }
        else if (condition_ == 4) {
            step_meii();
        }
        // log data
        log_step();
        // wait for the next clock cycle
        timer_.wait();
    }

    // show the UI
    ms_menu_msg_.write_message("fam_done");

    // disable openwrist
    if (condition_ > 0) {
        ow_.disable();
        q8_ow_.watchdog.stop();
    }

    // disable mahi exo ii
    // if (condition_ == 4) {
    //     meii_.disable();
    //     meii_daq_->stop_watchdog();
    // }

    // wait for UI response
    print("\nWaiting for subject to start training.");
    while (ms_menu_msg_.read_message() != "train" && !auto_stop_ && !manual_stop_) {
        if (condition_ > 0)
            step_system_ui();
        manual_stop_ = check_stop();
    }

    // renable openwrist
    if (condition_ > 0) {
        ow_.enable();
        q8_ow_.watchdog.start();
    }

    pd1_.reset_move_to_hold();
    pd2_.reset_move_to_hold();

    // meii_.robot_joint_pd_controllers_[0].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[1].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[2].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[3].reset_move_to_hold();
    // meii_.robot_joint_pd_controllers_[4].reset_move_to_hold();

    // renable mahi exo ii
    // if (condition_ == 4) {
    //     meii_.enable();
    //     meii_daq_->start_watchdog(0.1);
    // }

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// TRAINING STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_training(const NoEventData*) {

    // start the control loop
    timer_.restart();
    while (timer_.get_elapsed_time().as_seconds() < length_trials_[TRAINING] && !manual_stop_ && !auto_stop_) {

        // update countdown timer
        timer_data_ = { timer_.get_elapsed_time().as_seconds(), length_trials_[TRAINING] };
        ms_timer_.write_data(timer_data_);

        // step OpenWrist and Pendulum
        step_system_play();

        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2 || condition_ == 3) {
            step_cuff();
        }

        // step ME-II guidance
        // if (condition_ == 4) {
        //     step_meii();
        // }

        // log data
        log_step();

        // check for stop input
        manual_stop_ = check_stop();

        // wait for the next clock cycle
        timer_.wait();
    }

    // transition to the next state
    event(ST_TRANSITION);
}

//-----------------------------------------------------------------------------
// BREAK STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_break(const NoEventData*) {

    // disable hardware
    if (condition_ > 0) {
        ow_.disable();
        q8_ow_.watchdog.stop();
    }

    if (condition_ == 1 || condition_ == 2 || condition_ == 3)
        cuff_.disable();

    // if (condition_ == 4) {
    //     meii_.disable();
    //     meii_daq_->disable();
    // }

    // display UI
    ms_menu_msg_.write_message("break_begin");

    // start the control loop
    timer_.restart();
    while (timer_.get_elapsed_time().as_seconds() < length_trials_[BREAK] && !manual_stop_ && !auto_stop_) {
        if (condition_ > 0) {
            step_system_ui();
        }
        // update countdown timer
        timer_data_ = { timer_.get_elapsed_time().as_seconds(), length_trials_[TRAINING] };
        ms_timer_.write_data(timer_data_);
        // check for stop input
        manual_stop_ = check_stop();
        // check for subject break end
        if (ms_menu_msg_.read_message() == "break_end")
            break;
        // wait for the next clock cycle
        timer_.wait();
    }

    ms_menu_msg_.write_message("break_end");

    // transition to the next state
    if (manual_stop_ || auto_stop_)
        event(ST_STOP);
    else
        event(ST_START);
}

//-----------------------------------------------------------------------------
// GENERALIZATION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_generalization(const NoEventData*) {

    // start the control loop
    timer_.restart();
    while (timer_.get_elapsed_time().as_seconds() < length_trials_[GENERALIZATION] && !manual_stop_ && !auto_stop_) {
        // update countdown timer

        timer_data_ = { timer_.get_elapsed_time().as_seconds(), length_trials_[TRAINING] };
        ms_timer_.write_data(timer_data_);

        // step OpenWrist and Pendulum
        step_system_play();

        // step CUFF guidance
        if (condition_ == 1 || condition_ == 2 || condition_ == 3) {
            step_cuff();
        }
        else if (condition_ == 4) {
            step_meii();
        }

        // log data
        log_step();
        // check for stop input
        manual_stop_ = check_stop();
        // wait for the next clock cycle
        timer_.wait();
    }

    // transition to the next state
    event(ST_TRANSITION);

}

//-----------------------------------------------------------------------------
// TRANSITION STATE FUNCTION
//-----------------------------------------------------------------------------
void HapticGuidance::sf_transition(const NoEventData*) {

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
                q8_ow_.watchdog.stop();
            }
            // if (condition_ == 4) {
            //     meii_.disable();
            //     meii_daq_->stop_watchdog();
            // }

            // display UI
            ms_menu_msg_.write_message("gen_begin");

            print("\nWaiting for subject to start generalization");
            while (ms_menu_msg_.read_message() != "gen" && !auto_stop_ && !manual_stop_) {
                if (condition_ > 0)
                    step_system_ui();
                manual_stop_ = check_stop();
            }

            // renable openwrist
            if (condition_ > 0) {
                ow_.enable();
                q8_ow_.watchdog.start();
            }
            // if (condition_ == 4) {
            //     meii_.enable();
            //     meii_daq_->start_watchdog(0.1);
            // }

            pd1_.reset_move_to_hold();
            pd2_.reset_move_to_hold();

            // meii_.robot_joint_pd_controllers_[0].reset_move_to_hold();
            // meii_.robot_joint_pd_controllers_[1].reset_move_to_hold();
            // meii_.robot_joint_pd_controllers_[2].reset_move_to_hold();
            // meii_.robot_joint_pd_controllers_[3].reset_move_to_hold();
            // meii_.robot_joint_pd_controllers_[4].reset_move_to_hold();


        }

        // write out UI information
        ms_trial_.write_message(all_trial_names_[current_trial_index_]);
        ms_traj_name_.write_message(traj_.name_);
        std::vector<double> timer = { 0, length_trials_[trials_block_types_[current_trial_index_]] };
        ms_timer_.write_data(timer);

        // wait for user to confirm the trial while rendering pendulum
        ms_ui_msg_.write_message("show_reset");
        ms_reset_timer_.write_message("reset");
        if (all_trial_tags_[current_trial_index_] != "B1-1") {
            print("\nNEXT TRIAL: <" + all_trial_tags_[current_trial_index_] + ">. TRAJECTORY: " + traj_.name_ + ". Waiting for subject to confirm trial.");
            while (!confirmed_ && !manual_stop_ && !auto_stop_) {

                // step system
                step_system_idle();

                // step meii
                if (condition_ == 4)
                    step_meii();

                // check  if resetters triggered
                if (((std::abs(player_angle_) > reset_angle_window_) && !reset_triggered_) || Keyboard::is_key_pressed(Key::R, false)) {
                    reset_triggered_ = true;
                    ms_ui_msg_.write_message("show_confirm");
                    // reset scores
                    max_score_ = length_trials_[trials_block_types_[current_trial_index_]] * timer_.get_frequency().as_hertz() * error_window_;
                    player_score_ = 0;
                    expert_score_ = 0;
                    high_score_ = high_score_records_[traj_.name_];
                    scores_data_ = { player_score_, expert_score_, high_score_, max_score_ };
                    ms_scores_.write_data(scores_data_);
                    ms_score_msg_.write_message("reset_score");
                }

                // if reset triggered, check if player is confirming
                double confirm_speed = 1.0 / confirm_length_;
                if (reset_triggered_ && (std::abs(player_angle_) < confirm_angle_window_)) {
                    confirmation_percent_ += confirm_speed * timer_.get_period().as_seconds();
                }
                else if (reset_triggered_) {
                    confirmation_percent_ -= confirm_speed * timer_.get_period().as_seconds();
                }
                confirmation_percent_ = saturate(confirmation_percent_, 1.0, 0.0);
                error_angle_ = (1.0 - confirmation_percent_) * error_window_; // charging effect
                ms_confirmer_.write_data({confirmation_percent_});

                if (confirmation_percent_ == 1.0) {
                    reset_triggered_ = false;
                    confirmation_percent_ = 0.0;
                    confirmed_ = true;
                }

                // update angles
                angles_data_ = { player_angle_ , expert_angle_ , error_angle_ };
                ms_angles_.write_data(angles_data_);

                // wait for the next clock cycle
                timer_.wait();
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
void HapticGuidance::sf_stop(const NoEventData*) {

    if (current_trial_index_ < 0) {
        print("\nExperiment terminated during startup. Disabling hardware.");
        ms_menu_msg_.write_message("abort");
    }
    else if (current_trial_index_ < num_trials_total_ - 1) {
        print("\nExperiment aborted during trial <" + all_trial_tags_[current_trial_index_] + ">. Disabling hardware.");
        ms_menu_msg_.write_message("abort");
    }
    else {
        print("\nExperiment completed. Disabling hardware and saving data log.");
        ms_menu_msg_.write_message("complete");
    }


    if (condition_ > 0)
    {
        ow_.disable();
        q8_ow_.watchdog.stop();
        q8_ow_.disable();
    }

    if (condition_ == 1 || condition_ == 2 || condition_ == 3) {
        cuff_.disable();
    }

    // if (condition_ == 4) {
    //     meii_.disable();
    //     meii_daq_->stop_watchdog();
    //     meii_daq_->disable();
    // }

    main_log_.save_data(directory_, directory_, true);
    main_log_.wait_for_save();
}

//-----------------------------------------------------------------------------
// UTILS
//-----------------------------------------------------------------------------

void HapticGuidance::init_logs() {
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
        // .add_col("MEII PS Position [rad]")
        // .add_col("MEII PS Velocity [rad/s]")
        // .add_col("MEII PS Torque [Nm]");
        //.add_col("CUFF Ref. Motor Position 1")
        //.add_col("CUFF Ref. Motor Position 2")
        //.add_col("CUFF Act. Motor Position 1")
        //.add_col("CUFF Act. Motor Position 2")
        //.add_col("CUFF Act. Motor Current 1")
        //.add_col("CUFF ACt. Motor Current 2");
}

void HapticGuidance::log_trial() {
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

    std::vector<double> abs_error = abs_vec(trial_log_.get_col("Error [deg]"));

    row.push_back(mean(abs_error));
    row.push_back(stddev_s(abs_error));

    main_log_.add_row(row);

    std::string filename = stringify(current_trial_index_) + "_" + all_trial_tags_[current_trial_index_] + "_" + traj_.name_;
    std::string directory = directory_ + "\\_" + all_trial_blocks_[current_trial_index_];

    trial_log_.save_and_clear_data(filename, directory, true);
}

void HapticGuidance::log_step() {
    std::vector<double> row;
    row.reserve(trial_log_.get_col_count());

    row.push_back(timer_.get_elapsed_time().as_seconds());
    row.push_back(player_angle_);
    row.push_back(expert_angle_);
    row.push_back(error_angle_);
    row.push_back(ow_[0].get_position());
    row.push_back(ow_[0].get_velocity());
    row.push_back(ps_total_torque_);
    row.push_back(ps_comp_torque_);
    row.push_back(-pendulum_.Tau[0]);
    // row.push_back(meii_.joints_[1]->get_position());
    // row.push_back(meii_.joints_[1]->get_velocity());
    // row.push_back(meii_.joints_[1]->get_torque());
    //row.push_back(static_cast<double>(cuff_ref_pos_1_));
    //row.push_back(static_cast<double>(cuff_ref_pos_2_));
    //row.push_back(static_cast<double>(cuff_act_pos_1_));
    //row.push_back(static_cast<double>(cuff_act_pos_2_));
    //row.push_back(static_cast<double>(cuff_act_current_1_));
    //row.push_back(static_cast<double>(cuff_act_current_2_));

    trial_log_.add_row(row);
}

void HapticGuidance::wait_for_input() {
    Keyboard::wait_for_key(Key::Space, false);
}

bool HapticGuidance::check_stop() {
    return ( Keyboard::is_key_pressed(Key::Escape) || (Keyboard::is_key_pressed(Key::LControl) && Keyboard::is_key_pressed(Key::Space)));
}

void HapticGuidance::build_experiment() {

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
    //    print(all_trial_tags_[i] + " " + all_trial_blocks_[i] + " " + all_trajs_[i].name_);
    //}

}

//-----------------------------------------------------------------------------
// TRAJECTORY UTILS
//-----------------------------------------------------------------------------

void HapticGuidance::cinch_cuff() {
    cuff_.set_motor_positions(offset[0], offset[1], true);
}

void HapticGuidance::release_cuff() {
    cuff_.set_motor_positions(offset[0] + 1000, offset[1] - 1000, true);
}


void HapticGuidance::step_cuff() {

    // set motor positions to offsets
    cuff_ref_pos_1_ = offset[0];
    cuff_ref_pos_2_ = offset[1];

    // feedforward mechanism
    if (condition_ == 1 || condition_ == 3) {
        // feedforward mechanism
        cuff_ref_pos_1_ += (short int)((expert_angle_)* cuff_ff_gain_);
        cuff_ref_pos_2_ += (short int)((expert_angle_)* cuff_ff_gain_);
    }
    else if (condition_ == 2) {
        // feedforward mechanism (reversed b/c CUFF flipped)
        cuff_ref_pos_1_ -= (short int)((expert_angle_)* cuff_ff_gain_);
        cuff_ref_pos_2_ -= (short int)((expert_angle_)* cuff_ff_gain_);
    }
    else if (condition_ == 3) {
        // feedback mechanism
        cuff_ref_pos_1_ -= (short int)((error_angle_)* cuff_ff_gain_);
        cuff_ref_pos_2_ -= (short int)((error_angle_)* cuff_ff_gain_);
    }

    // set motor positions
    cuff_.set_motor_positions(cuff_ref_pos_1_, cuff_ref_pos_2_, true);

    // get current CUFF state
    cuff_.get_motor_positions(cuff_act_pos_1_, cuff_act_pos_2_, true);
    cuff_.get_motor_currents(cuff_act_current_1_, cuff_act_current_2_, true);
}

/*void HapticGuidance::step_meii() {
    meii_daq_->reload_watchdog();
    if (meii_daq_->is_watchdog_expired()) {
        print("ME-II Watchdog tripped!");
        auto_stop_ = true;
    }

    meii_daq_->read_all();

    meii_.joints_[0]->set_torque(meii_.robot_joint_pd_controllers_[0].move_to_hold(-10 * math::DEG2RAD, meii_.joints_[0]->get_position(), 0.25, meii_.joints_[0]->get_velocity(), clock_.delta_time_, math::DEG2RAD, 10 * math::DEG2RAD));
    meii_.joints_[1]->set_torque(pd1_meii_.move_to_hold(math::DEG2RAD * (meii_offset + expert_angle_), meii_.joints_[1]->get_position(), 0.25, meii_.joints_[1]->get_velocity(), clock_.delta_time_, math::DEG2RAD, 30 * math::DEG2RAD));

    meii_.joints_[2]->set_torque(0.0);
    meii_.joints_[3]->set_torque(0.0);
    meii_.joints_[4]->set_torque(0.0);


    if (meii_.check_all_joint_limits()) {
        auto_stop_ = true;
    }

    meii_daq_->write_all();
}*/


void HapticGuidance::update_scores() {

    // calculate max score
    max_score_ = length_trials_[trials_block_types_[current_trial_index_]] * clock_.frequency_ * error_window_;

    // update current scores
    player_score_ += saturate(error_window_ - std::abs(error_angle_), error_window_, 0.0);
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

void HapticGuidance::step_system_ui() {

    q8_ow_->read_all();
    ow_.get_joint_positions();
    ow_.update_state_map();

    if (ow_.check_all_joint_limits() || q8_ow_->is_watchdog_expired()) {
        auto_stop_ = true;
    }
}

void HapticGuidance::step_system_play() {

    // read and reload DAQ
    if (condition_ > 0) {
        q8_ow_->reload_watchdog();
        q8_ow_->read_all();
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

        ow_.joints_[1]->set_torque(pd1_.move_to_hold(0, ow_.joints_[1]->get_position(),
            move_to_speed_ * math::DEG2RAD, ow_.joints_[1]->get_velocity(),
            clock_.delta_time_, math::DEG2RAD, 10*math::DEG2RAD));

        ow_.joints_[2]->set_torque(pd2_.move_to_hold(math::DEG2RAD * 0, ow_.joints_[2]->get_position(),
            move_to_speed_ * math::DEG2RAD, ow_.joints_[2]->get_velocity(),
            clock_.delta_time_, math::DEG2RAD, 10 * math::DEG2RAD));

    }

    // check joint limits
    if (ow_.check_all_joint_limits() || q8_ow_->is_watchdog_expired()) {
        auto_stop_ = true;
    }

    // update OpenWrist state
    ow_.update_state_map();

    // write the DAQ
    if (condition_ > 0)
        q8_ow_->write_all();

    // check for manual stop from conductor
    manual_stop_ = check_stop();
}

void HapticGuidance::step_system_idle() {

    // read and reload DAQ
    if (condition_ > 0) {
        q8_ow_->reload_watchdog();
        q8_ow_->read_all();
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

        ow_.joints_[1]->set_torque(pd1_.move_to_hold(0, ow_.joints_[1]->get_position(),
            move_to_speed_ * math::DEG2RAD, ow_.joints_[1]->get_velocity(),
            clock_.delta_time_, math::DEG2RAD, 10* math::DEG2RAD));

        ow_.joints_[2]->set_torque(pd2_.move_to_hold(math::DEG2RAD * 0, ow_.joints_[2]->get_position(),
            move_to_speed_ * math::DEG2RAD, ow_.joints_[2]->get_velocity(),
            clock_.delta_time_, math::DEG2RAD, 10* math::DEG2RAD));

    }

    // check joint limits
    if (ow_.check_all_joint_limits() || q8_ow_->is_watchdog_expired()) {
        auto_stop_ = true;
    }

    // update OpenWrist state
    ow_.update_state_map();

    // write the DAQ
    if (condition_ > 0)
        q8_ow_->write_all();

    // check for manual stop from conductor
    manual_stop_ = check_stop();

}
