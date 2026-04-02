#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"
// driver servos
#include "Servo.h"
// State Machine Utilities
#include "StateMachineUtils.h"


bool move_servo(int color_to_servo){ // wert color_to_sensor: 0 = no input, 3 = red, 4 = yello, 5 = green, 7 = blue
    static StateTimer transition_timer; // initialising timer funktion
    static bool initialized=false;


    // servo_DO = S-0090, servo_D1 = S-3003
    static Servo servo_D0(PB_D0);
    static Servo servo_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // reely S0090
    static float servo_D0_ang_min = 0.0325f; // careful, these values might differ from servo to servo
    static float servo_D0_ang_max = 0.1250f;
    // futuba S3001
    static float servo_D1_ang_min = 0.0325f;
    static float servo_D1_ang_max = 0.1300f;
    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)

    if(initialized == false){
        // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
        servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
        servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
        // default acceleration of the servo motion profile is 1.0e6f
        servo_D0.setMaxAcceleration(0.3f);
        servo_D1.setMaxAcceleration(0.3f);

        initialized = true;
    }
    static float servo_input_D0 = 1.0f;
    static float servo_input_D1 = 1.0f;


    // set up states for state machine
 enum ServoState {
        INITIAL,
        ROTATE_OUT,
        SLEEP_OUT,
        ROTATE_IN,
        SLEEP_IN,
        FINISHED
    } static servo_state = ServoState::INITIAL;


    // state machine
    switch (servo_state) {
        case ServoState::INITIAL: {
           
            if (!servo_D0.isEnabled()){
                servo_D0.enable(1.0f);
            }
            
            
            if (servo_state == ServoState::INITIAL){ // && (color_to_servo == 3 || color_to_servo == 4 || color_to_servo == 5 || color_to_servo == 7)
                servo_state = ServoState::ROTATE_OUT;
            }

            return false;
        }
        case ServoState::ROTATE_OUT: { // move robot out
            
            
            // function to map the distance to the servo movement -> (0.0f, 1.0f)
            servo_input_D0 = 0.3f;
            // values smaller than 0.0f or bigger than 1.0f are constrained to the range (0.0f, 1.0f) in setPulseWidth
            servo_D0.setPulseWidth(servo_input_D0);
            transition_timer.start(3000);

            servo_state = ServoState::SLEEP_OUT;

            break;
           
		 case ServoState::SLEEP_OUT:{ // waiting until robot is fully extended
		 	
		 	
		 	if (transition_timer.isExpired()) {
		 		servo_state = ServoState::ROTATE_IN;
		 		
		 	}
		 	
			break;
		 }  
		   
        }
        case ServoState::ROTATE_IN: { // move robot back in
           
            // function to map the distance to the servo movement -> (0.0f, 1.0f)
            servo_input_D0 = 1.0f;
            // values smaller than 0.0f or bigger than 1.0f are constrained to the range (0.0f, 1.0f) in setPulseWidth
            servo_D0.setPulseWidth(servo_input_D0);
            transition_timer.start(3000);

            servo_state = ServoState::SLEEP_IN;

            break;
        }
        case ServoState::SLEEP_IN: { // waiting until robot is fully parked
            
            if (transition_timer.isExpired()) {
                
                servo_state = ServoState::FINISHED;
                }
            
            break;
        }
        case ServoState::FINISHED: {
            
            servo_state = ServoState::INITIAL;

            return true; // signal to main, that the task is done
            
        }
        default: {

            break; // do nothing
        }
    }

    return false; // sequence is still running
}
