#ifndef MEL_HAPTICTRAINING_HPP
#define MEL_HAPTICTRAINING_HPP

#include <MEL/Core/PdController.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include "Cuff/Cuff.hpp"
#include "FurutaPendulum.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "OpenWrist.hpp"

// =============================================================================
// Haptic Training Experiments
// -----------------------------------------------------------------------------
// Evan Pezent (April 2018 - May 2018)
// -----------------------------------------------------------------------------
// Condition 0: No Devices (for debugging)
// Condition 1: OpenWrist
// Condition 2: OpenWrist + CUFF
// -----------------------------------------------------------------------------
// TRIAL           |
// -----------------------------------------------------------------------------
// Familiarization |
// Evaluation      |
// Training        |
// Evaluation      |
// Training        |
// Evaluation      |
// Training        |
// Evaluation      |
// Break           |
// Training        |
// Evaluation      |
// Training        |
// Evaluation      |
// Training        |
// Evaluation      |
// Generalization  |

using namespace mel;

class HapticTraining : public StateMachine {
public:
    /// Constructor
    HapticTraining(Q8Usb& q8,
                   OpenWrist& ow,
                   Cuff& cuff,
                   int subject_number,
                   int condition,
                   const std::string& start_trial);

    /// States
    enum States {
        ST_START,
        ST_FAMILIARIZATION,
        ST_EVALUATION,
        ST_TRAINING,
        ST_BREAK,
        ST_GENERALIZATION,
        ST_TRANSITION,
        ST_STOP,
        ST_NUM_STATES
    };

    /// STATE FUNCTIONS
    void sf_start(const NoEventData*);
    void sf_familiarization(const NoEventData*);
    void sf_evaluation(const NoEventData*);
    void sf_training(const NoEventData*);
    void sf_break(const NoEventData*);
    void sf_generalization(const NoEventData*);
    void sf_transition(const NoEventData*);
    void sf_stop(const NoEventData*);

    /// STATE ACTIONS
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_start>
        sa_start;
    StateAction<HapticTraining,
                NoEventData,
                &HapticTraining::sf_familiarization>
        sa_familiarization;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_evaluation>
        sa_evaluation;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_training>
        sa_training;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_break>
        sa_break;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_generalization>
        sa_generlization;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_transition>
        sa_transition;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_stop> sa_stop;

    /// STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_start, &sa_familiarization, &sa_evaluation, &sa_training,
            &sa_break, &sa_generlization,   &sa_transition, &sa_stop,
        };
        return &STATE_MAP[0];
    }

    /// SUBJECT
    const int subject_number_;

    /// CONDITION
    enum Condition { DEFAULT = 0, OW_ONLY = 1, OW_CUFF = 2 };

    const Condition condition_;

    /// BLOCK TYPES
    enum BlockType {
        FAMILIARIZATION = 0,
        EVALUATION      = 1,
        TRAINING        = 2,
        BREAK           = 3,
        GENERALIZATION  = 4
    };

    const std::array<std::string, 5> block_names_ = {
        "FAMILIARIZATION", "EVALUATION", "TRAINING", "BREAK", "GENERALIZATION"};
    const std::array<std::string, 5> block_tags_ = {"F", "E", "T", "B", "G"};

    // EXPERIMENT BLOCK ORDER (SET MANUALLY)
    const std::vector<BlockType> block_order_ = {
        FAMILIARIZATION, EVALUATION, TRAINING,   EVALUATION,
        TRAINING,        EVALUATION, TRAINING,   EVALUATION,
        BREAK,           TRAINING,   EVALUATION, TRAINING,
        EVALUATION,      TRAINING,   EVALUATION, GENERALIZATION};

    // NUMBER OF BLOCKS PER BLOCK TYPE (SET DURING CONSTRUCTION)
    // [ FAMILIARIZATION, TRAINING, BREAK, GENERALIZATION ]
    std::array<int, 5> num_blocks_ = {0, 0, 0, 0, 0};

    // NUMBER OF TRIALS PER BLOCK TYPE PER BLOCK NUMBER (SET MANUALLY)
    // [ FAMILIARIZATION, TRAINING, BREAK, GENERALIZATION ]
    const std::array<int, 5> num_trials_ = {1, 3, 12, 1, 12};

    // EXPERIMENT TRIAL ORDERING
    void build_experiment();
    int current_trial_index_ = 0;
    std::vector<BlockType> trials_block_types_;
    std::vector<std::string> all_trial_blocks_;
    std::vector<std::string> all_trial_tags_;
    std::vector<std::string> all_trial_names_;
    int num_trials_total_ = 0;
    bool trials_started_  = false;

    // SUBJECT DIRECTORY
    std::string directory_;

    /// Hardware
    Q8Usb& q8_;
    OpenWrist& ow_;
    Cuff& cuff_;
};

#endif  // MEL_HAPTICTRAINING_HPP

// Questions for O'Malley
// 1) How do I train feedback and feedforward? Separately, together?
//    Everyone get feedback and feedforward? 20. or Separate? 40
// 2)
