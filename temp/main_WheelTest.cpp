#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"

#include "DCMotor.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // --- adding variables and objects and applying functions starts here ---

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                    // is a defined potential

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                    // 6.0f V if you only use one battery pack
    
    // motor M1
    const float gear_ratio_M1 = 78.125f; // gear ratio
    const float kn_M1 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // enable the motion planner for smooth movement
    motor_M1.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);

    // motor M2
    const float gear_ratio_M2 = 100.0f; // gear ratio
    const float kn_M2 = 140.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // enable the motion planner for smooth movement
    motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

    // set up states for state machine
    enum RobotState {
        INITIAL,
        SLEEP,
        RIGHTWHEEL,
        LEFTWHEEL
    } robot_state = RobotState::INITIAL;


    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---
        // print to the serial terminal
        printf("Motor1 Right Rotations: %f Motor2 Left Rotations: %f\n", motor_M1.getRotation(), motor_M2.getRotation());

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---
            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::SLEEP: {
                    printf("SLEEP\n");
                    motor_M1.setRotation(0.0f);
                    motor_M2.setRotation(0.0f);
                   
                    if (mechanical_button.read()){
                        robot_state = RobotState::RIGHTWHEEL;
                    }

                    break;
                }
                case RobotState::RIGHTWHEEL: {
                    printf("RIGHTWHEEL\n");
                    motor_M1.setRotation(3.0f);

                    if (motor_M1.getRotation() > 2.95f){
                        robot_state = RobotState::LEFTWHEEL;
                    }

                    break;
                }
                case RobotState::LEFTWHEEL: {
                    printf("LEFTWHEEL\n");
                    motor_M2.setRotation(3.0f);

                    if (motor_M2.getRotation() > 2.95f){
                        robot_state = RobotState::SLEEP;
                    }

                    break;
                }
                default: {

                    break; // do nothing
                }
            }


        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
                motor_M1.setMotionPlannerPosition(0.0f);
                motor_M1.setMotionPlannerVelocity(0.0f);
                motor_M1.enableMotionPlanner();
                motor_M2.setMotionPlannerPosition(0.0f);
                motor_M2.setMotionPlannerVelocity(0.0f);
                motor_M2.enableMotionPlanner();
                robot_state = RobotState::INITIAL;

            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
