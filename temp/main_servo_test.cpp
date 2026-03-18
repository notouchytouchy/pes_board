#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"

// driver servos
#include "Servo.h"


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

    // servo_DO = S-0090, servo_D1 = S-3003
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // reely S0090
    float servo_D0_ang_min = 0.0325f; // careful, these values might differ from servo to servo
    float servo_D0_ang_max = 0.1250f;
    // futuba S3001
    float servo_D1_ang_min = 0.0325f;
    float servo_D1_ang_max = 0.1300f;
    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(1.0e4f);
    servo_D1.setMaxAcceleration(1.0e4f);

    float servo_input_D0 = 0.0f;
    float servo_input_D1 = 0.0f;
    int servo_select = 0;   // ensures only one servo a time is turning (0 = servo_D0, 1 = servo_D1)


    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                    // is a defined potential

    // set up states for state machine
    enum RobotState {
    INITIAL,
    SERVO_1,
    SLEEP,
    SERVO_2
    } robot_state = RobotState::INITIAL;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---


        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    // enable the servo
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    if (!servo_D1.isEnabled())
                        servo_D1.enable();
                    robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::SERVO_1: {
                    printf("SERVO_1\n");
                    // function to map the distance to the servo movement -> (0.0f, 1.0f)
                    servo_input_D0 = 1.0f;
                    // values smaller than 0.0f or bigger than 1.0f are constrained to the range (0.0f, 1.0f) in setPulseWidth
                    servo_D0.setPulseWidth(servo_input_D0);
                    servo_select = 1;

                    // if the mechanical button is pressed go to SLEEP
                    if (mechanical_button.read())
                        robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::SLEEP: {
                    printf("SLEEP\n");

                    servo_input_D0 = 1.0f;
                    servo_input_D1 = 1.0f;
                    servo_D0.setPulseWidth(servo_input_D0);
                    servo_D0.setPulseWidth(servo_input_D1);

                    // if the mechanical button is pressed go to SERVO_1 or SERVO_2
                    if (mechanical_button.read() && servo_select == 0)
                        robot_state = RobotState::SERVO_1;
                    if (mechanical_button.read() && servo_select == 1)
                        robot_state = RobotState::SERVO_2;

                    break;
                }
                case RobotState::SERVO_2: {
                    printf("SERVO_2\n");
                    // function to map the distance to the servo movement -> (0.0f, 1.0f)
                    servo_input_D1 = 1.0f;
                    // values smaller than 0.0f or bigger than 1.0f are constrained to the range (0.0f, 1.0f) in setPulseWidth
                    servo_D0.setPulseWidth(servo_input_D0);
                    servo_select = 0;

                    // if the mechanical button is pressed go to SLEEP
                    if (mechanical_button.read())
                        robot_state = RobotState::SLEEP;

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
                servo_D0.disable();
                servo_D1.disable();
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
