#include "HapticTraining.hpp"
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/Console.hpp>
#include <random>

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
      current_trial_index_(0) {
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

    // print results of build
    for (int i = 0; i < num_trials_total_; ++i) {
        print(all_trial_tags_[i] + " " + all_trial_blocks_[i]);
    }
}

void HapticTraining::sf_start(const NoEventData*) {
    // enable hardware
}

void HapticTraining::sf_familiarization(const NoEventData*) {}

void HapticTraining::sf_evaluation(const NoEventData*) {}

void HapticTraining::sf_training(const NoEventData*) {}

void HapticTraining::sf_break(const NoEventData*) {}

void HapticTraining::sf_generalization(const NoEventData*) {}

void HapticTraining::sf_transition(const NoEventData*) {}

void HapticTraining::sf_stop(const NoEventData*) {}
