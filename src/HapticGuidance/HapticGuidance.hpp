#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Core/PdController.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <map>
#include "Cuff/Cuff.hpp"
#include "OpenWrist.hpp"
#include "Pendulum.hpp"

//----------------------------------------------------------------------------
// Hatpic guidance experiements with OpenWrist, CUFF, and MahiExo-II
//----------------------------------------------------------------------------
// Evan Pezent, Simone Fanni, Josh Bradely (August 2017 - October 2017)
//----------------------------------------------------------------------------
// Condition 0: No Devices (for debugging)
// Condition 1: OpenWrist P/S + CUFF Haptic Guidance Ipsilateral      x10 subj
// Condition 2: OpenWrist P/S + CUFF Haptic Guidance Contralateral    x10 subj
// Condition 3: OpenWrist P/S + ME-II Haptic Guidance Contralateral   x10 subj
// Condition 4: OpenWrist P/S only (for debugging)
//----------------------------------------------------------------------------
// EXPERIMENT BREAKDOWN              | BLOCK |
//----------------------------------------------------------------------------
// Familiarization Trial (x1 - 1min) | F1    |
// Training Trials (x12 - 20s ea)    | T1    |
// Training Trials (x12 - 20s ea)    | T2    |
// Training Trials (x12 - 20s ea)    | T3    |
// 5 Minute Break                    | B1    |
// Training Trials (x12 - 20s ea)    | T4    |
// Training Trials (x12 - 20s ea)    | T5    |
// Training Trials (x12 - 20s ea)    | T6    |
// Generalization (x12 - 20s ea)     | G     |
//----------------------------------------------------------------------------

using namespace mel;

class HapticGuidance : public StateMachine {

public:

    HapticGuidance(Q8Usb& q8_ow, OpenWrist& ow, Cuff& cuff, int subject_number, int condition, const std::string& start_trial = "F1-1");

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_START,
        ST_FAMILIARIZATION,
        ST_TRAINING,
        ST_BREAK,
        ST_GENERALIZATION,
        ST_TRANSITION,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_start(const NoEventData*);
    void sf_familiarization(const NoEventData*);
    void sf_training(const NoEventData*);
    void sf_break(const NoEventData*);
    void sf_generalization(const NoEventData*);
    void sf_transition(const NoEventData*);
    void sf_stop(const NoEventData*);

    // STATE ACTIONS
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_start> sa_start;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_familiarization> sa_familiarization;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_training> sa_training;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_break> sa_break;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_generalization> sa_generlization;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_transition> sa_transition;
    StateAction<HapticGuidance, NoEventData, &HapticGuidance::sf_stop> sa_stop;

    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_start,
            &sa_familiarization,
            &sa_training,
            &sa_break,
            &sa_generlization,
            &sa_transition,
            &sa_stop,
        };
        return &STATE_MAP[0];
    }

    //-------------------------------------------------------------------------
    // EXPERIMENT SETUP
    //-------------------------------------------------------------------------

    // SUBJECT/CONDITION
    const int subject_number_;
    const int condition_;

    // BLOCK TYPES
    enum BlockType {
        FAMILIARIZATION = 0,
        TRAINING = 1,
        BREAK = 2,
        GENERALIZATION = 3
    };

    const std::array<std::string, 4> block_names_ = { "FAMILIARIZATION", "TRAINING", "BREAK", "GENERALIZATION" };
    const std::array<std::string, 4> block_tags_ =  { "F",               "T",        "B",     "G"              };

    // EXPERIMENT BLOCK ORDER (SET MANUALLY)
    const std::vector<BlockType> block_order_ = {
        FAMILIARIZATION,
        TRAINING,
        TRAINING,
        TRAINING,
        BREAK,
        TRAINING,
        TRAINING,
        TRAINING,
        GENERALIZATION
    };

    // NUMBER OF BLOCKS PER BLOCK TYPE (SET DURING CONSTRUCTION)
    // [ FAMILIARIZATION, TRAINING, BREAK, GENERALIZATION ]
    std::array<int, 4> num_blocks_ = { 0, 0, 0, 0 };

    // NUMBER OF TRIALS PER BLOCK TYPE PER BLOCK NUMBER (SET MANUALLY)
    // [ FAMILIARIZATION, TRAINING, BREAK, GENERALIZATION ]
    const std::array<int, 4> num_trials_ = { 1, 12, 1, 12 };

    // LENGTH IN SECONDS OF EACH BLOCK TYPE TRIAL (SET MANUALLY)
    // [ FAMILIARIZATION, TRAINING, BREAK, GENERALIZATION ]
    const std::array<double, 4> length_trials_ = { 120, 20, 300, 20 };

    // EXPERIMENT TRIAL ORDERING
    void build_experiment();
    int current_trial_index_ = 0;
    std::vector<BlockType> trials_block_types_;
    std::vector<std::string> all_trial_blocks_;
    std::vector<std::string> all_trial_tags_;
    std::vector<std::string> all_trial_names_;
    int num_trials_total_ = 0;
    bool trials_started_ = false;

    // SUBJECT DIRECTORY
    std::string directory_;

    //-------------------------------------------------------------------------
    // CONTROL LOOP UTILS
    //-------------------------------------------------------------------------

    double move_to_speed_ = 60; // [deg/s]

    void step_system_ui();
    void step_system_play();
    void step_system_idle();

    //-------------------------------------------------------------------------
    // EXPERIMENT COMPONENTS
    //-------------------------------------------------------------------------

    // HARDWARE TIMER
    Timer timer_;

    // HARDWARE
    Q8Usb& q8_ow_;
    OpenWrist& ow_;
    // Daq* meii_daq_;
    // MahiExoII& meii_;

    Cuff& cuff_;

    // PD CONTROLLERS
    PdController pd1_ = PdController(60, 1);   // OpenWrist Joint 1 (FE)
    PdController pd2_ = PdController(40, 0.5); // OpenWrist Joint 2 (RU)

    // MAHI-EXO II
    //void step_meii();
    //double meii_move_to_speed = 15;
    //double meii_offset = 22.5;
   // PdController pd1_meii_ = PdController(4, 0.05); // half of Craig's default PD Controller

    // CUFF
    const short int cuff_normal_force_ = 3;
    const short int cuff_ff_gain_ = 250;
    const short int cuff_fb_gain_ = 175;
    short int offset[2];
    short int scaling_factor[2];

    void release_cuff();
    void cinch_cuff();
    void step_cuff();

    // PENDULUM
    Pendulum pendulum_;

    // TRAJECTORY CLASS
    struct Trajectory {

        Trajectory(std::string name,
            double A, double B, double C,
            double a, double b, double c,
            double amp, double norm) :
            name_(name),
            A_(A), B_(B), C_(C),
            a_(a), b_(b), c_(c),
            amp_(amp), norm_(norm) {}
        Trajectory() : Trajectory("name", 0, 0, 0, 0, 0, 0, 0, 0) {}

        std::string name_;

        double amp_;

        double A_;
        double B_;
        double C_;

        double a_;
        double b_;
        double c_;

        double norm_;

        double eval(double time) {
            return amp_ / norm_ * (
                A_ * std::sin(2 * PI * a_ * time) +
                B_ * std::sin(2 * PI * b_ * time) +
                C_ * std::sin(2 * PI * c_ * time)
                );
        }
    };

    // TRAJECTORIES
    Trajectory traj_familiarization_ = Trajectory("FAMILIARIZATION", 1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 30.0, 1.0);
    Trajectory traj_break_ = Trajectory("BREAK", 0, 0, 0, 0, 0, 0, 0, 0);

    std::vector<Trajectory> trajs_training_ =  {
        Trajectory("EASY", 1.0, 0.0, 0.0, 0.20, 0.0, 0.0, 30.0, 1.0),
        Trajectory("MEDIUM", -1.0, -0.5, 0.0, 0.25, 0.1, 0.0, 30.0, 1.4773031358),
        Trajectory("HARD", 0.5, -0.5, -1.0, 0.2, 0.1, 0.4, 30.0, 1.82293041893)
    };

    std::vector<Trajectory> trajs_generalization_ = {
        Trajectory("EASY+", -1.0, 0.0, 0.0, 0.30, 0.0, 0.0, 35.0, 1.0),
        Trajectory("MEDIUM+", -1.0, -0.5, 0.0, 0.25, 0.1, 0.0, 35.0, 1.69999991826),
        Trajectory("HARD+", 0.75, 0.4, -1.0, 0.3, 0.5, 0.3, 35.0, 0.649999962214)
    };

    std::vector<Trajectory> all_trajs_; // random generated for all trials
    Trajectory traj_; // current trajectory

    // TRACKED VARIABLES
    double ps_comp_torque_;
    double ps_total_torque_;
    short int cuff_ref_pos_1_;
    short int cuff_ref_pos_2_;
    short int cuff_act_pos_1_;
    short int cuff_act_pos_2_;
    short int cuff_act_current_1_;
    short int cuff_act_current_2_;
    double player_angle_ = 0;
    double error_angle_ = 0;
    double expert_angle_ = 0;

    // SCORING
    double error_window_ = 10; // +/- error window (size)
    double expert_score_ = 0;
    double player_score_ = 0;
    double high_score_ = 0;
    double max_score_ = 0;

    std::map<std::string, double> high_score_records_;

    void update_scores();

    // SUBJECT USER INPUT
    bool confirmed_ = false;
    double confirm_angle_window_ = 3;
    double reset_angle_window_ = 30;
    bool reset_triggered_ = false;
    double confirm_length_ = 2.0;
    double confirmation_percent_ = 0.0;

    // CONDUCTOR USER INPUT
    void wait_for_input();
    bool check_stop();
    bool manual_stop_ = false;
    bool auto_stop_ = false;

    // DATA LOGGING
    //DataLog main_log_ = DataLog("main_log", true, 100);
    //DataLog trial_log_ = DataLog("trial_log", false, 180000);
    void init_logs();
    void log_step();
    void log_trial();

    // MELSHARE (DATA)
    MelShare ms_timer_;
    std::vector<double> timer_data_;

    MelShare ms_angles_;
    std::vector<double> angles_data_;

    MelShare ms_scores_;
    std::vector<double> scores_data_; // current score, expert score, high score, max score

    MelShare ms_confirmer_;

    // MelShare meii_state_ = MelShare("meii_state");
    // std::vector<double> meii_data_ = { 0, 0, 0, 0, 0 };

    // MELSHARE (MESSAGES)
    MelShare ms_reset_timer_;
    MelShare ms_trial_;
    MelShare ms_traj_name_;
    MelShare ms_ui_msg_;
    MelShare ms_score_msg_;
    MelShare ms_menu_msg_;
};
