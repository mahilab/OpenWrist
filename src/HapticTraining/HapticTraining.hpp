#ifndef MEL_HAPTICTRAINING_HPP
#define MEL_HAPTICTRAINING_HPP

#include <MEL/Mechatronics/PdController.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include <MEL/Logging/Csv.hpp>
#include "Cuff/Cuff.hpp"
#include "Simulations/FurutaPendulum.hpp"
#include "MEL/Communications/MelShare.hpp"
#include "OpenWrist.hpp"
#include "PerlinNoise.hpp"

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

    //==========================================================================
    // STATE MACHINE SETUP
    //==========================================================================

    /// States
    enum States {
        ST_START,
        ST_BALANCE,
        ST_INVERT,
        ST_RESET,
        ST_STOP,
        ST_FAMILIAR,
        ST_NUM_STATES
    };

    /// STATE FUNCTIONS
    void sf_start(const NoEventData*);
    void sf_balance(const NoEventData*);
    void sf_invert(const NoEventData*);
    void sf_reset(const NoEventData*);
    void sf_stop(const NoEventData*);
    void sf_familiar(const NoEventData*);

    /// STATE ACTIONS
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_start> sa_start;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_balance> sa_balance;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_invert> sa_invert;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_reset> sa_reset;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_stop> sa_stop;
    StateAction<HapticTraining, NoEventData, &HapticTraining::sf_familiar> sa_familiar;

    /// STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_start,
            &sa_balance,
            &sa_invert,
            &sa_reset,
            &sa_stop,
            &sa_familiar,
        };
        return &STATE_MAP[0];
    }

    //==========================================================================
    // EXPERIMENT SETUP
    //==========================================================================

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
    // [ FAMILIARIZATION, EVALUATION, TRAINING, BREAK, GENERALIZATION ]
    const std::array<int, 5> num_trials_ = {1,6, 36, 1, 36};//1 3 12 1 12

    // EXPERIMENT TRIAL ORDERING
    void build_experiment();
    std::vector<BlockType> trials_block_types_;
    std::vector<std::string> all_trial_blocks_;
    std::vector<std::string> all_trial_tags_;
    std::vector<std::string> all_trial_names_;
    std::vector<int> all_trial_difficulty_;
    int num_trials_total_ = 0;
    bool trials_started_  = false;

    // SUBJECT DIRECTORY
    std::string directory_;

    Timer timer_;
    Timer perlintimer_;

    /// Hardware
    Q8Usb& q8_;
    OpenWrist& ow_;
    Cuff& cuff_;

    PdController pd0_;
    PdController pd1_;
    PdController pd2_;

    //CUFF 
    const double lqr_gains[3][4] = {//numbers taken from matlab lqr script
        {-1.00, 5.5835, -1.0031, 1.7767},//easy k1-k4
        {-1.00, 4.6172, -0.9072, 1.2218},//med k1-k4
        {-1.00, 3.4889, -0.7811, 0.6137}//hard k1-k4
    };
    const short int cuff_ff_gain_ = 250;
    const short int cuff_fb_gain_ = 1600;
    short int cuff_ref_pos_1_;
    short int cuff_ref_pos_2_;
    short int offset_[2];
    double opt_torque_ = 0.0;
    double cuff_angle_ = 0.0;
    bool cuff_active = true;
    
    short int trial=0;
    short int difficulty=1;

    /// PENDULUM
    FurutaPendulum fp_;

    double wall_ = 50 * mel::DEG2RAD;
    double k_wall_ = 50;
    double b_wall_ = 1;
    double K_player_ = 25;
    double B_player_ = 1;
    double tau_ = 0.0;

    /// FUNCTIONS
    void render_pendulum();
    void render_walls();
    void lock_joints();
    void cuff_balance();
    void change_pendulum(int);
    void write_to_log();
    void save_log();
    void take_a_break();

    Time best_up_time = Time::Zero;

    /// MELSHARES
    MelShare ms_scores_;
    std::vector<double> data_scores_;
    MelShare ms_active;
    MelShare ms_noise;
    MelShare ms_text;

    //LOGGING
    std::array<double,11> logdata;
    std::vector<std::array<double,11>> trialdata;
    const int loops_per_log = 10;
    std::vector<std::string> logheader = {"trial","difficulty","time_bal","q1","q2","q3","q4","opt_torque","ow_pos","ow_vel","pernoise"};

    //NOISE
    siv::PerlinNoise pnoise;
    double ball_perturb = 0;
    const double noise_gain = 0.5;


};

#endif  // MEL_HAPTICTRAINING_HPP

// Questions for O'Malley
// 1) How do I train feedback and feedforward? Separately, together?
//    Everyone get feedback and feedforward? 20. or Separate? 40
// 2)
