#include "HapticTraining.hpp"
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/Csv.hpp>
#include <MEL/Core/Console.hpp>
#include <random>
#include <fstream>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include "PerlinNoise.hpp"


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
      pd0_(60.0, 1.0),
      pd1_(60.0, 1.0),
      pd2_(40.0, 0.5),
      timer_(hertz(1000)),
      perlintimer_(hertz(1000)),
      ms_scores_("scores"),
      ms_active("active"),
      ms_noise("noise"),
      ms_text("text"),
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

    // build the experiment
    build_experiment();

    // set the current trial index
    for (int i = 0; i < all_trial_tags_.size(); ++i) {
        if (start_trial == all_trial_tags_[i])
            trial = i;
    }

    LOG(Info) << "Starting Experiment";
    LOG(Info) << "Subject Number: " << subject_number;
    LOG(Info) << "Condition: " << condition_;
    LOG(Info) << "Start Trial: " << all_trial_tags_[trial];
}

void HapticTraining::build_experiment() {
    // for every block
    for (auto it = block_order_.begin(); it != block_order_.end(); ++it) {
        // increment the number of blocks of this type of block
        num_blocks_[*it]++;

        //create variable for difficulty to be shuffled for each trial
        std::vector<int> block_diff_shuffle_(num_trials_[*it]);
        block_diff_shuffle_.clear();
        if((num_trials_[*it] % 3)==0){//if divisible by 3
            for(int i=0; i<num_trials_[*it]/3;i++){//for large trials per block (training,eval,general)
                block_diff_shuffle_.push_back(1);
                block_diff_shuffle_.push_back(2);
                block_diff_shuffle_.push_back(3);
            }
        }
        else{
            for(int i=0; i<num_trials_[*it];i++){//for small trials per block (famil, break)
                block_diff_shuffle_.push_back(2);//give placeholder of 2    
            }
        }
        //add difficulty to overall trial difficulty list
        std::shuffle(std::begin(block_diff_shuffle_),std::end(block_diff_shuffle_),std::default_random_engine(rand()));
        all_trial_difficulty_.insert(all_trial_difficulty_.end(),block_diff_shuffle_.begin(),block_diff_shuffle_.end());
        

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

      //print results of build
     for (int i = 0; i < num_trials_total_; ++i) {
         print(all_trial_tags_[i] + " " + all_trial_blocks_[i]);
         print(all_trial_difficulty_[i]);
               
     }



}

void HapticTraining::sf_start(const NoEventData*) {

    pd0_.reset_move_to_hold();
    pd2_.reset_move_to_hold();

    // enable CUFF
    if (condition_ == OW_CUFF) {
        cuff_.enable();
        cuff_.pretension(offset_);
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
    double pert = (double)rand()/(double)RAND_MAX*0.04-0.02;
    fp_.reset(0,0,0,pert);//gives small random perturbation to joint 2

    int loop_count = loops_per_log;//start at max so that log is recorded on first frame

    bool balanced = true;
    Time unbalanced_time = seconds(1.0);
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
        cuff_balance();

        // logic
        if (!fp_.balance_upright) {
            if (unbalanced_clock.get_elapsed_time() > unbalanced_time)
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
        data_scores_[2] = 0.05 * curr_up_time.as_seconds();
        data_scores_[3] = 0.05 * best_up_time.as_seconds();
        ms_scores_.write_data(data_scores_);

        if(loop_count>=loops_per_log){
            loop_count=1;
            write_to_log();
        }
        loop_count++;

        // check limits
        if (ow_.any_torque_limit_exceeded()) {
            event(ST_STOP);
            return;
        }

        // update output
        q8_.update_output();
        timer_.wait();
    }

    //SAVE RESULTS OF LAST TRIAL
    if(trial>0){ 
        LOG(Info) << "Trial:"<<trial << " Difficulty:"<< all_trial_difficulty_[trial] << " Score:" << data_scores_[2]/0.05;
        save_log();
    }

    trial++;

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

void HapticTraining::sf_familiar(const NoEventData*) {
    // reset pendulum
    fp_.reset(0, PI, 0, 0);
    timer_.restart();
    while (!ctrlc && timer_.get_elapsed_time() < seconds(45)) {
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
        data_scores_[0] = (1.0 - timer_.get_elapsed_time().as_seconds() / 45);
        data_scores_[1] = data_scores_[1];
        data_scores_[2] = 0.0;
        data_scores_[3] = data_scores_[3];
        ms_scores_.write_data(data_scores_);
        // update output
        q8_.update_output();
        timer_.wait();
    }

    trial++;
    if (ctrlc)
        event(ST_STOP);
    else
        event(ST_RESET);

}

void HapticTraining::sf_reset(const NoEventData*) {
    
    ms_active.write_message("inactive");

    if(trial==0){
        ms_text.write_message(all_trial_blocks_[trial]);
    }
    else if(all_trial_blocks_[trial].compare(all_trial_blocks_[trial-1])){
        ms_text.write_message(all_trial_blocks_[trial]);
    }

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

       //END EXPERIMENT IF TRIALS UP
    if(trial >= num_trials_total_){
        LOG(Info) << "Stopping Experiment";
        cuff_.disable();
        ow_.disable();
        q8_.disable();
        event(ST_STOP);//end experiment
        return;
    }
    
       //CHOOSE PARAMETERS BASED ON BLOCK
     if(0==all_trial_tags_[trial].compare(0,1,"F")){//familiarization
        cuff_active=false;
        change_pendulum(all_trial_difficulty_[trial]);     
        }
     else if(0==all_trial_tags_[trial].compare(0,1,"E")){//evaluation
        cuff_active = false;
        change_pendulum(all_trial_difficulty_[trial]);
        }
     else if(0==all_trial_tags_[trial].compare(0,1,"T")){//training
        if(condition_==2){//check for cuff or control
            cuff_active=true;
            change_pendulum(all_trial_difficulty_[trial]);}
        else{
            cuff_active=false;
            change_pendulum(all_trial_difficulty_[trial]);  }  
        }
     else if(0==all_trial_tags_[trial].compare(0,1,"B")){//break
        cuff_active = false;
        take_a_break();
        }
     else if(0==all_trial_tags_[trial].compare(0,1,"G")){//generalization
        cuff_active=false;
        change_pendulum(all_trial_difficulty_[trial]+3);//use different pendulum for generalization    
        }
    else{
        cuff_active = false;
        }
    

     //BEGIN NEXT TRIAL
    print(all_trial_tags_[trial] + " " + all_trial_blocks_[trial]);
    //print(all_trial_difficulty_[trial]);
    
 
  
    while (!ctrlc && timer_.get_elapsed_time() < seconds(2.0)) {
        q8_.watchdog.kick();
        q8_.update_input();

        ow_[1].set_torque(pd1_.move_to_hold(0, ow_[1].get_position(),
            60 * mel::DEG2RAD, ow_[1].get_velocity(),
            0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
        lock_joints();
        if (ow_.any_torque_limit_exceeded()) {
            event(ST_STOP);
            return;
        }
        q8_.update_output();
        timer_.wait();
    }

    ms_text.write_message(" ");
    ms_active.write_message("active");

    if (ctrlc)
        event(ST_STOP);
    else if(trial==0)
        event(ST_FAMILIAR);
    else
        event(ST_BALANCE);

}

void HapticTraining::sf_stop(const NoEventData*) {
    LOG(Info) << "Stopping Experiment";
    cuff_.disable();
    ow_.disable();
    q8_.disable();
}

//==============================================================================
// OPENWRIST FUNCTIONS
//==============================================================================

void HapticTraining::render_pendulum() {
    tau_ = K_player_ * (ow_[1].get_position() - fp_.q1) + B_player_ * (ow_[1].get_velocity() - fp_.q1d);
 
    ball_perturb = noise_gain * data_scores_[2] * pnoise.octaveNoise(perlintimer_.get_elapsed_time().as_seconds()/3,3);
    std::vector<double> noises(1);
    noises[0] = ball_perturb;
    ms_noise.write_data(noises);

    fp_.update(timer_.get_elapsed_time(), tau_,ball_perturb);
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

void HapticTraining::change_pendulum(int difficulty_){//1-easy 2-medium 3-hard   4-easy+ 5-med+ 6-hard+
    if(difficulty_==1){//easy
        fp_.l2 = 1.4;
        fp_.r_mass=0.1;
    }
    if(difficulty_==2){//med
        fp_.l2 = 1;
        fp_.r_mass=0.1;
    }
    if(difficulty_==3){//hard
        fp_.l2 = 0.5;
        fp_.r_mass=0.1;
    }
    if(difficulty_==4){//easy+
        fp_.l2 = 1.4;
        fp_.r_mass=0.12;
    }
    if(difficulty_==5){//med+
        fp_.l2 = 1;
        fp_.r_mass=0.12;
    }
    if(difficulty_==6){//hard+
        fp_.l2 = 0.5;
        fp_.r_mass=0.12;
    }
    fp_.reset();
    fp_.update_properties();
    fp_.write_properties();


    
}

//==============================================================================
// CUFF FUNCTIONS
//==============================================================================
void HapticTraining::cuff_balance() {
            
    if (fp_.balance_upright && cuff_active) {
        opt_torque_ = -(lqr_gains[difficulty-1][0]*fp_.q1 + lqr_gains[difficulty-1][1]*fp_.q2 + lqr_gains[difficulty-1][2]*fp_.q1d + lqr_gains[difficulty-1][3]*fp_.q2d);
        cuff_angle_=(2000*log(2*mel::abs(opt_torque_)+0.5)+1400)*sign(opt_torque_);
        }
    else
        cuff_angle_ = 0.0;
           
    cuff_ref_pos_1_ = offset_[0] - cuff_angle_;
    cuff_ref_pos_2_ = offset_[1] - cuff_angle_;

    //cuff_ref_pos_1_ = saturate(cuff_ref_pos_1_, -5000-offset_[0], 5000-offset_[0]);
    //cuff_ref_pos_2_ = saturate(cuff_ref_pos_2_, -5000-offset_[1], 5000-offset_[1]);

    cuff_.set_motor_positions(cuff_ref_pos_1_, cuff_ref_pos_2_, true);
}

//==============================================================================
// MISC FUNCTIONS
//==============================================================================

void HapticTraining::write_to_log(){
    logdata[0]=trial-1;
    logdata[1]=all_trial_difficulty_[trial];
    logdata[2]=data_scores_[2]/0.05;
    logdata[3]=fp_.q1;
    logdata[4]=fp_.q2;
    logdata[5]=fp_.q1d;
    logdata[6]=fp_.q2d;
    logdata[7]=opt_torque_;
    logdata[8]=ow_[1].get_position();
    logdata[9]=ow_[1].get_velocity();
    logdata[10]=ball_perturb;

    trialdata.push_back(logdata);
}

void HapticTraining::save_log(){
    std::string filepath = "experimentdata/" + directory_ + "/" + all_trial_tags_[trial] + ".csv";
    //filepath = "/exper/help/data1.csv";
    //print(filepath);
    csv_write_row(filepath,logheader);
    csv_append_rows(filepath,trialdata);
    trialdata.clear();

}

void HapticTraining::take_a_break(){//DOESNT WORK - TRIGGERS WATCHDOG


        print("~~~~~~~BREAK TIME~~~~~~~");
        print("~press Escape to resume~");
        timer_.restart();

        cuff_.disable();
        ow_.disable();
        q8_.watchdog.stop();
        q8_.disable();

        while(!Keyboard::is_key_pressed(Key::Escape))
        {
            timer_.wait();
        }
        if(condition_==OW_CUFF)
        cuff_active=true;

        cuff_.enable(); 
        cuff_.pretension(offset_);
        timer_.restart();
        q8_.enable();
        ow_.enable();
        trial++;
        q8_.watchdog.start();
        q8_.watchdog.kick();
        event(ST_RESET);

}