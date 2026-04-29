#include <cstdio>
#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"
// driver servos
#include "Servo.h"
// State Machine Utilities
#include "StateMachineUtils.h"


bool move_servo(int color_to_servo, int packageReceived){ // wert color_to_sensor: 0 = no input, 3 = red, 4 = yello, 5 = green, 7 = blue
    static StateTimer transition_timer; // initialising timer funktion
    static bool initializedDelievering=false;
    static bool initializedReceiving=false;


    // servo_DO = S-0090, servo_D1 = S-3003
    static Servo servo_D0(PB_D0);
    static Servo servo_D1(PB_D1);
    static Servo servo_D2(PB_D2);
    static Servo servo_D3(PB_D3);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // reely S0090
    static float servo_D1_ang_min = 0.0325f; // careful, these values might differ from servo to servo
    static float servo_D1_ang_max = 0.1300f;
    // futuba S3001
    static float servo_D0_ang_min = 0.0325f;
    static float servo_D0_ang_max = 0.1300f;
    // futuba S3001
    static float servo_D2_ang_min = 0.0325f;
    static float servo_D2_ang_max = 0.1250f;
    // futuba S3001
    static float servo_D3_ang_min = 0.0325f;
    static float servo_D3_ang_max = 0.1300f;
    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)

    if(initializedReceiving == false && packageReceived<4){ // initializing servos for receiving the packages
        // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
        servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
        servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
        servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);
        servo_D3.calibratePulseMinMax(servo_D3_ang_min, servo_D3_ang_max);
        // default acceleration of the servo motion profile is 1.0e6f
        servo_D0.setMaxAcceleration(0.5f);
        servo_D1.setMaxAcceleration(0.5f);
        servo_D2.setMaxAcceleration(0.5f);
        servo_D3.setMaxAcceleration(0.5f);
        // check to make sure servo outputs are disabled bevor start
        if(servo_D0.isEnabled())
            servo_D0.disable();
        if(servo_D1.isEnabled())
            servo_D1.disable();
        if(servo_D2.isEnabled())
            servo_D2.disable();
        if(servo_D3.isEnabled())
            servo_D3.disable();

        initializedReceiving = true;
    }

    if(initializedDelievering == false && packageReceived==4){ // initializing servos for delievering the packages
        // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
        servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
        servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
        servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);
        servo_D3.calibratePulseMinMax(servo_D3_ang_min, servo_D3_ang_max);
        // default acceleration of the servo motion profile is 1.0e6f
        servo_D0.setMaxAcceleration(0.2f);
        servo_D1.setMaxAcceleration(0.2f);
        servo_D2.setMaxAcceleration(0.2f);
        servo_D3.setMaxAcceleration(0.2f);
        // check to make sure servo outputs are disabled bevor start
        if(servo_D0.isEnabled())
            servo_D0.disable();
        if(servo_D1.isEnabled())
            servo_D1.disable();
        if(servo_D2.isEnabled())
            servo_D2.disable();
        if(servo_D3.isEnabled())
            servo_D3.disable();

        initializedDelievering = true;
    }
    static float servo_input_D0 = 0.0f;
    static float servo_input_D1 = 1.0f;
    static float servo_input_D2 = 0.0f;
    static float servo_input_D3 = 1.0f;


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
            
            servo_state = ServoState::ROTATE_OUT;

            break;
        }
        case ServoState::ROTATE_OUT: { // move robot out
           
            if(color_to_servo == 4){ //yellow servo
                servo_D0.enable(0.0f);
                servo_input_D0 = 0.41f;
                servo_D0.setPulseWidth(servo_input_D0);
            }
            else if(color_to_servo == 3){ //red servo
                servo_D1.enable(0.97f);
                servo_input_D1 = 0.47f;
                servo_D1.setPulseWidth(servo_input_D1);
            }
            else if(color_to_servo == 7){ //blue servo
                servo_D2.enable(0.0f);
                servo_input_D2 = 0.41f;
                servo_D2.setPulseWidth(servo_input_D2);
            }
            else if(color_to_servo == 5){ //green servo
                servo_D3.enable(1.0f);
                servo_input_D3 = 0.47f;
                servo_D3.setPulseWidth(servo_input_D3);
            }

            transition_timer.start(3000);
            servo_state = ServoState::SLEEP_OUT;

            break;
        }  
		case ServoState::SLEEP_OUT: { // waiting until robot is fully extended
		 	
		    if (transition_timer.isExpired()) {
		 		servo_state = ServoState::ROTATE_IN;
		 	}
		 	
			break;
		}  
        case ServoState::ROTATE_IN: { // move robot back in
            
            if(color_to_servo == 4){ //yello servo
                servo_input_D0 = 0.00f;
                servo_D0.setPulseWidth(servo_input_D0);
            }
            else if(color_to_servo == 3){ //red servo
                servo_input_D1 = 0.97f;
                servo_D1.setPulseWidth(servo_input_D1);
            }
            else if(color_to_servo == 7){ //blue servo
                servo_input_D2 = 0.00f;
                servo_D2.setPulseWidth(servo_input_D2);
            }
            else if(color_to_servo == 5){ //green servo
                servo_input_D3 = 0.98f;
                servo_D3.setPulseWidth(servo_input_D3);
            }

            transition_timer.start(3000);
            servo_state = ServoState::SLEEP_IN;

            break;
        }
        case ServoState::SLEEP_IN: { // waiting until robot is fully parked

            if (transition_timer.isExpired()) {
                if(servo_D0.isEnabled())
                    servo_D0.disable();
                if(servo_D1.isEnabled())
                    servo_D1.disable();
                if(servo_D2.isEnabled())
                    servo_D2.disable();
                if(servo_D3.isEnabled())
                    servo_D3.disable();
                
                servo_state = ServoState::FINISHED;
            }
            
            break;
        }
        case ServoState::FINISHED: {
            
            initializedDelievering = false;
            initializedReceiving = false;
            servo_state = ServoState::INITIAL;
            return true; // signal to main, that the task is done

        }
        default: {

            break; // do nothing
        }
    }

    return false; // sequence is still running
}
