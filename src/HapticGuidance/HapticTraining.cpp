#include "HapticTraining.hpp"
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/Console.hpp>
#include <random>
#include <fstream>
#include <MEL/Utility/Windows/Keyboard.hpp>


bool read_csv(std::string filename, std::string directory, int row_offset, std::vector<std::vector<double>>& output) {
    output.clear();
    std::string full_filename = directory + "\\" + filename + ".csv";
    std::ifstream input(full_filename);
    input.precision(12);
    if (input.is_open()) {
        std::string csv_line;
        int row = 0;
        while (std::getline(input, csv_line)) {
            if (row++ >= row_offset) {
                std::istringstream csv_stream(csv_line);
                std::vector<double> row;
                std::string number;
                double data;
                while (std::getline(csv_stream, number, ',')) {
                    std::istringstream number_stream(number);
                    number_stream >> data;
                    row.push_back(data);
                }
                output.push_back(row);
            }
        }
        return true;
    }
    else {
        LOG(Warning) << "File not found for read_csv().";
        return false;
    }
}

std::vector<double> get_column(const std::vector<std::vector<double>>& data, int col) {
    std::vector<double> col_data(data.size());
    for (std::size_t i = 0; i < data.size(); ++i) {
        col_data[i] = data[i][col];
    }
    return col_data;
}

ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
    print("Ctrl+C Pressed");
    ctrlc = true;
    return true;
}

HapticTraining::HapticTraining(Q8Usb& q8,
                               OpenWrist& ow,
                               Cuff& cuff,
                               int subject_number,
                               int condition,
                               const std::string& start_trial)
    : StateMachine(9),
      q8_(q8),
      ow_(ow),
      cuff_(cuff),
      subject_number_(subject_number),
      condition_(static_cast<Condition>(condition)),
      current_trial_index_(0),
      pd0_(60.0, 1.0),
      pd1_(60.0, 1.0),
      pd2_(40.0, 0.5),
      timer_(hertz(1000)),
      ms_scores_("scores"),
      data_scores_(4, 0.0)
{
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // create subject folder
    if (subject_number < 10)
        directory_ =
            "S0" + stringify(subject_number) + "_C" + stringify(condition);
    else
        directory_ =
            "S" + stringify(subject_number) + "_C" + stringify(condition);

    // seed random number generator with subject num
    srand(subject_number);

    // build the experimemnt
    build_experiment();

    // set the current trial index
    for (int i = 0; i < all_trial_tags_.size(); ++i) {
        if (start_trial == all_trial_tags_[i])
            current_trial_index_ = i - 1;
    }

    LOG(Info) << "Starting Experiment";
    LOG(Info) << "Subject Number: " << subject_number;
    LOG(Info) << "Condition: " << condition_;
    LOG(Info) << "Start Trial: " << all_trial_tags_[current_trial_index_ + 1];
}

void HapticTraining::build_experiment() {
    // for every block
    for (auto it = block_order_.begin(); it != block_order_.end(); ++it) {
        // increment the number of blocks of this type of block
        num_blocks_[*it]++;

        // for each trial in this block type
        for (int i = 0; i < num_trials_[*it]; i++) {
            trials_block_types_.push_back(*it);
            all_trial_blocks_.push_back(block_names_[*it]);
            all_trial_tags_.push_back(block_tags_[*it] +
                                      std::to_string(num_blocks_[*it]) + "-" +
                                      std::to_string(i + 1));
            all_trial_names_.push_back(block_names_[*it] + " " +
                                       std::to_string(num_blocks_[*it]) + "-" +
                                       std::to_string(i + 1));
            num_trials_total_++;
        }
    }

    // // print results of build
    // for (int i = 0; i < num_trials_total_; ++i) {
    //     print(all_trial_tags_[i] + " " + all_trial_blocks_[i]);
    // }
}

void HapticTraining::sf_start(const NoEventData*) {

    pd0_.reset_move_to_hold();
    pd2_.reset_move_to_hold();

    // enable CUFF
    if (condition_ == OW_CUFF) {
        cuff_.enable();
        cuff_.pretension(cuff_normal_force_, offset_, scaling_factor_);
        cuff_.set_motor_positions(offset_[0], offset_[1], true);
    }

    // enable OpenWrist
    if (condition_ == OW_ONLY || condition_ == OW_CUFF) {
        q8_.enable();
        ow_.enable();
        q8_.watchdog.start();
    }

    // transition to next state
    event(ST_RESET);
}

void HapticTraining::sf_balance(const NoEventData*) {

    // reset pendulum
    fp_.reset(0, 0, 0, 0);

    bool balanced = true;
    Time unblanaced_time = seconds(2.0);
    Clock unbalanced_clock;

    Time curr_up_time = Time::Zero;
    Clock up_time_clock;

    timer_.restart();
    while(!ctrlc && balanced) {
        q8_.watchdog.kick();
        q8_.update_input();
        render_pendulum();
        render_walls();
        lock_joints();

        // logic
        if (!fp_.upright) {
            if (unbalanced_clock.get_elapsed_time() > unblanaced_time)
                balanced = false;
            up_time_clock.restart();
        }
        else {
            unbalanced_clock.restart();
            curr_up_time = up_time_clock.get_elapsed_time();
            if (curr_up_time > best_up_time) {
                best_up_time = curr_up_time;
            }
        }
        data_scores_[0] = 0.0;
        data_scores_[1] = data_scores_[1];
        data_scores_[2] = 0.05 * curr_up_time.as_seconds() / (1 + 0.05 * curr_up_time.as_seconds());
        data_scores_[3] = 0.05 * best_up_time.as_seconds() / (1 + 0.05 * best_up_time.as_seconds());
        ms_scores_.write_data(data_scores_);

        // check limits
        if (ow_.any_torque_limit_exceeded()) {
            event(ST_STOP);
            return;
        }

        // update output
        q8_.update_output();
        timer_.wait();
    }

    if (ctrlc)
        event(ST_STOP);
    else
        event(ST_RESET);
}

void HapticTraining::sf_invert(const NoEventData*) {

    // reset pendulum
    fp_.reset(0, PI, 0, 0);

    timer_.restart();
    while (!ctrlc && timer_.get_elapsed_time() < seconds(10)) {

        q8_.watchdog.kick();
        q8_.update_input();
        render_pendulum();
        render_walls();
        lock_joints();

        // check limits
        if (ow_.any_torque_limit_exceeded()) {
            event(ST_STOP);
            return;
        }

        data_scores_[0] = (1.0 - timer_.get_elapsed_time().as_seconds() / 10.0);
        data_scores_[1] = data_scores_[1];
        data_scores_[2] = 0.0;
        data_scores_[3] = data_scores_[3];
        ms_scores_.write_data(data_scores_);

        // update output
        q8_.update_output();
        timer_.wait();
    }

    if (ctrlc)
        event(ST_STOP);
    else
        event(ST_RESET);

}

void HapticTraining::sf_reset(const NoEventData*) {

    Time reset_time = seconds(1.0);

    pd1_.reset_move_to_hold();

    double q1 = fp_.q1;
    double q2 = fp_.q2;
    double score1 = data_scores_[0];
    double score2 = data_scores_[2];
    double t = 0.0;

    timer_.restart();
    while (!ctrlc && timer_.get_elapsed_time() < reset_time) {
        q8_.watchdog.kick();
        q8_.update_input();

        ow_[1].set_torque(pd1_.move_to_hold(0, ow_[1].get_position(),
            60 * mel::DEG2RAD, ow_[1].get_velocity(),
            0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

        t = timer_.get_elapsed_time().as_seconds() / reset_time.as_seconds();

        fp_.q1 = ow_[1].get_position();
        fp_.q2 = q2 + t * (0 - q2);
        fp_.write_state();

        data_scores_[0] = score1 - score1 * t;
        data_scores_[2] = score2 - score2 * t;
        ms_scores_.write_data(data_scores_);


        lock_joints();

        if (ow_.any_torque_limit_exceeded()) {
            event(ST_STOP);
            return;
        }

        q8_.update_output();
        timer_.wait();
    }

    if (ctrlc)
        event(ST_STOP);
    else
        event(ST_INVERT);

}

void HapticTraining::sf_stop(const NoEventData*) {
    LOG(Info) << "Stopping Experiment";
}

//==============================================================================
// FUNCTIONS
//==============================================================================

void HapticTraining::render_pendulum() {
    tau_ = K_player_ * (ow_[1].get_position() - fp_.q1) + B_player_ * (ow_[1].get_velocity() - fp_.q1d);
    fp_.update(timer_.get_elapsed_time(), tau_);
    ow_[1].set_torque(-fp_.tau1);
}

void HapticTraining::render_walls() {
    if (ow_[1].get_position() >= wall_) {
        if (ow_[1].get_velocity() > 0)
            ow_[1].add_torque(k_wall_ * (wall_ - ow_[1].get_position()) + b_wall_ * (0 - ow_[1].get_velocity()));
        else
            ow_[1].add_torque(k_wall_ * (wall_ - ow_[1].get_position()));
    }
    else if (ow_[1].get_position() <= -wall_) {
        if (ow_[1].get_velocity() < 0)
            ow_[1].add_torque(k_wall_ * (-wall_ - ow_[1].get_position()) + b_wall_ * (0 - ow_[1].get_velocity()) );
        else
            ow_[1].add_torque(k_wall_ * (-wall_ - ow_[1].get_position()));
    }
}

void HapticTraining::lock_joints() {
    ow_[0].set_torque(pd0_.move_to_hold(0, ow_[0].get_position(),
        60 * mel::DEG2RAD, ow_[0].get_velocity(),
        0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

    ow_[2].set_torque(pd2_.move_to_hold(mel::DEG2RAD * 0, ow_[2].get_position(),
        60 * mel::DEG2RAD, ow_[2].get_velocity(),
        0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
}

