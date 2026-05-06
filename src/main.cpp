#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "Linefollower_config.h"
// subfunction for servo usage
#include "move_servo.h"


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
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there is a defined potential

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f; // https://www.pololu.com/product/3490/specs
    const float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    LineFollowerConfig config; // uses struct from LineFollowerConfig to configure Linefollower-functions
    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, config.bar_dist, config.d_wheel, config.b_wheel, motor_M2.getMaxPhysicalVelocity());
    lineFollower.setRotationalVelocityControllerGains(config.Kp, config.Kp_nl);
    lineFollower.setMaxWheelVelocity(3.0f);
    

    // start timer
    main_task_timer.start();

    // color sensor
    float color_cal[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // define an array to store the calibrated measurement of the color sensor
    
    int color_num = 0; // define a variable to store the color number, e.g. 0 for red, 1 for green, 2 for blue, 3 for clear
    ColorSensor Color_Sensor(PB_3, PB_14, PA_4, PB_0, PA_0, PA_1); // create ColorSensor object, connect the frequency output pin of the sensor to PC_2
    Color_Sensor.setFrequency(FREQ_002);

    enum RobotState {
        INITIAL,
        DrivingStart,
        DrivingLeft,
        DrivingAlignment,
        DrivingBackwards,
        DrivingUntilCrossLine,
        Stopping,
        Repos,
        MoveArm,
        FINISHED

    } static robot_state = RobotState::INITIAL;

    bool armRetracted = false; // return value of function move_servo
    int actualColor = 0; // 0=undefined, 3=red, 4=yellow, 5=green, 7=blue
    int packageReceived = 0; // counts how many packages were collected
    int packageDelieverd = 0;  // counts how many packages were delieverd
    float lastPositionM1 = 0;  // sets the last position of the M1, when cross line detected
    float lastPositionM2 = 0;  // sets the last position of the M2, when cross line detected
    float overShootCorrection = 0; // correction variable against undershooting on the first house


    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---
            enable_motors = 1;
            
            // read the calibrated color measurement (unitless) and store it in the defined variable
            for (int i = 0; i < 4; i++) {
                color_cal[i] = Color_Sensor.readColorCalib()[i];
            }

            // read the classified color number and store it in the defined variable
            color_num = Color_Sensor.getColor();


            // state machine
            switch (robot_state){
                case RobotState::INITIAL: {

                    if (mechanical_button.read()){  // startbutton pressed. beginning of the cycle
                    overShootCorrection = 1.5f;
                    robot_state = RobotState::DrivingStart; 
                    }
                    
                    break;
                }

                case RobotState::DrivingStart: { // drive forward until cross line (loop)
                   
                    motor_M1.setVelocity(1*0.78125f); // set a desired speed for speed controlled dc motors M1, factor 078 because there are different gears in the two motors
                    motor_M2.setVelocity(1);  // set a desired speed for speed controlled dc motors M2

                    if(lineFollower.getAvgBit(2)>0.5 && lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 && lineFollower.getAvgBit(5)>0.5){ // if cross line is detected
                        motor_M1.setVelocity(0);  
                        motor_M2.setVelocity(0);
                        lastPositionM1 = motor_M1.getRotation();
                        lastPositionM2 = motor_M2.getRotation();

                        robot_state = RobotState::DrivingLeft;  
                    }
                    
                    break;
                }
                
                case RobotState::DrivingLeft:{ // driving a left curve
                        
                        motor_M1.setVelocity((2.0)*0.78125f); 
                        motor_M2.setVelocity(0.7);

                    if(motor_M1.getRotation() > (lastPositionM1 + 1.5)){ // if left curve rotation is reached
                        motor_M1.setVelocity(0); 
                        motor_M2.setVelocity(0);
                        lastPositionM1 = motor_M1.getRotation();
                        lastPositionM2 = motor_M2.getRotation();
                        robot_state = RobotState::DrivingAlignment; 
                        }
                    
                    break;
                }

                case RobotState::DrivingAlignment:{ // driving until the second cross line for alignment
                        
                    lineFollower.setMaxWheelVelocity(1.5f); // set velocity lower for better accuracy on the first house

                    motor_M1.setVelocity((lineFollower.getRightWheelVelocity())*0.78125f); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2

                        if(motor_M1.getRotation() > (lastPositionM1 + 3)){ // if the first cross line is reached we are start checking for the second

                            if((lineFollower.getAvgBit(2)>0.5 && lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 || lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 && lineFollower.getAvgBit(5)>0.5)){  // if 3 LED's detect black, we are at a cross line 
                                motor_M1.setVelocity(0); 
                                motor_M2.setVelocity(0);
                                lastPositionM1 = motor_M1.getRotation();
                                lastPositionM2 = motor_M2.getRotation();
                                lineFollower.setMaxWheelVelocity(3.0f); // set velocity back to original value
                                robot_state = RobotState::DrivingBackwards; 
                            }
                        }

                    break; 
                }

                case RobotState::DrivingBackwards:{ // driving backwards until the first cross line
                        
                    motor_M1.setVelocity((-0.8)*0.78125f); 
                    motor_M2.setVelocity(-0.82);

                    if(motor_M1.getRotation() < (lastPositionM1 - 0.5)){ // inhibit the detection of the starting cross line
                        if((lineFollower.getAvgBit(2)>0.5 && lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 || lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 && lineFollower.getAvgBit(5)>0.5)){  // if 3 LED's detect black, we are at a cross line 
                                robot_state = RobotState::Stopping; 
                        } 
                    }

                    break; 
                }

                case RobotState::DrivingUntilCrossLine: { // driving until a cross line is detected

                    motor_M1.setVelocity((lineFollower.getRightWheelVelocity())*0.78125f); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2

                    if((lineFollower.getAvgBit(2)>0.5 && lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 || lineFollower.getAvgBit(3)>0.5 && lineFollower.getAvgBit(4)>0.5 && lineFollower.getAvgBit(5)>0.5) && (motor_M2.getRotation() > (lastPositionM2 + 2.0))){  // if 3 LED's detect black, we are at a cross line
                        overShootCorrection = 0; // overwrite correction varable with 0, because we don't need them anymore
                        robot_state = RobotState::Stopping;  
                    }

                    break;
                }

                case RobotState::Stopping: { // waiting until robot is fully parked

                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    lastPositionM1 = 0;
                    lastPositionM2 = 0;

                    robot_state = RobotState::Repos;
                    
                    break;
                }

                case RobotState::Repos: { // repositioning the robot according to the detected color

                    if(color_num == 3 || color_num == 4 || color_num == 5 || color_num == 7){  // if valid color is detected, color is safed
                        actualColor = color_num; // saving color internally
                    }

                    if (lastPositionM2 == 0){ // if lastPosition was set to zero, we move the robot backwards
                        lastPositionM1 = motor_M1.getRotation();
                        lastPositionM2 = motor_M2.getRotation();
                        motor_M1.setVelocity((-0.5)*0.78125f); 
                        motor_M2.setVelocity(-0.5);  
                    }

                    if (packageReceived == 0){
                        
                        if(actualColor == 3){ // repositioning color red
                                motor_M1.setVelocity((0.5)*0.78125f); 
                                motor_M2.setVelocity(0.5);  

                            if(motor_M2.getRotation() > (lastPositionM2 + 0.10)){
                                motor_M1.setVelocity(0); 
                                motor_M2.setVelocity(0);
                                robot_state = RobotState::MoveArm; 
                            }
                        }

                        else if(actualColor == 4){ // repositioning color yellow
                        
                            if(motor_M2.getRotation() < (lastPositionM2 - 0.29)){
                                motor_M1.setVelocity(0); 
                                motor_M2.setVelocity(0);
                                robot_state = RobotState::MoveArm; 
                            }
                        }

                        else if(actualColor == 5){ // repositioning color green

                            if(motor_M2.getRotation() < (lastPositionM2 - 0.25)){
                                motor_M1.setVelocity(0); 
                                motor_M2.setVelocity(0);
                                robot_state = RobotState::MoveArm; 
                            }
                        }

                        else if(actualColor == 7){ // repositioning color blue

                            if(motor_M2.getRotation() < (lastPositionM2 - 0.6)){
                                motor_M1.setVelocity(0); 
                                motor_M2.setVelocity(0);
                                robot_state = RobotState::MoveArm; 
                            }
                        }
                    }

                    if(actualColor == 3 && packageReceived != 0){ // repositioning color red

                        if(motor_M2.getRotation() < (lastPositionM2 - 0.10)){
                            motor_M1.setVelocity(0); 
                            motor_M2.setVelocity(0);
                            robot_state = RobotState::MoveArm; 
                        }
                    }

                    else if(actualColor == 4 && packageReceived != 0){ // repositioning color yellow
                        
                        if(motor_M2.getRotation() < (lastPositionM2 - 0.58)){
                            motor_M1.setVelocity(0); 
                            motor_M2.setVelocity(0);
                            robot_state = RobotState::MoveArm; 
                        }
                    }

                    else if(actualColor == 5 && packageReceived != 0){ // repositioning color green

                        if(motor_M2.getRotation() < (lastPositionM2 - 0.55)){
                            motor_M1.setVelocity(0); 
                            motor_M2.setVelocity(0);
                            robot_state = RobotState::MoveArm; 
                        }
                    }

                    else if(actualColor == 7 && packageReceived != 0){ // repositioning color blue

                        if(motor_M2.getRotation() < (lastPositionM2 - 0.85)){
                            motor_M1.setVelocity(0); 
                            motor_M2.setVelocity(0);
                            robot_state = RobotState::MoveArm; 
                        }
                    }
                    
                    break;
                }

                case RobotState::MoveArm: { // move the servo arm

                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    armRetracted = move_servo (actualColor, packageReceived); // Ausfuehren des Armbewegungsprogramms  

                    if(armRetracted==true){
                        if(packageReceived<4){ // if we receive a package, we count them and are going back to DrivingUntilCrossLine for the next one
                            packageReceived++;
                            actualColor = 0; // 0=undefined, 3=red, 4=yellow, 5=green, 7=blue
                            robot_state = RobotState::DrivingUntilCrossLine;
                        }
                        else if(packageDelieverd<4 && packageReceived==4){  // if we deliver a package, we count them and are going back to DrivingUntilCrossLine for the next one
                            packageDelieverd++;
                            actualColor = 0; // 0=undefined, 3=red, 4=yellow, 5=green, 7=blue
                            robot_state = RobotState::DrivingUntilCrossLine;    
                        }

                        if(packageDelieverd==4 && packageReceived==4) {  // if we received 4 and delievered 4 packages, we are finished and are going to the finished state
                            robot_state = RobotState::FINISHED;  
                        }
                        armRetracted = false;
                    }

                    break;
                }

                case RobotState::FINISHED: {  // prepare everything for another cycle and go back to initial

                armRetracted = false;
                actualColor = 0; // 0=undefined, 3=red, 4=yellow, 5=green, 7=blue
                packageReceived = 0;
                packageDelieverd = 0;
                lastPositionM1 = 0;
                lastPositionM2 = 0;

                robot_state = RobotState::INITIAL;

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

                 enable_motors = 0;

                // reset variables and objects
                actualColor = 0; // 0=undefined, 3=red, 4=yellow, 5=green, 7=blue
                packageReceived = 0;
                packageDelieverd = 0;

                robot_state = RobotState::INITIAL;

                for (int i = 0; i < 4; i++) {
                    color_cal[i] = 0.0f;
                }
                color_num = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

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