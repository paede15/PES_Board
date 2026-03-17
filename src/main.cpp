#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "FastPWM.h"
#include <Eigen/Dense>
#include "SensorBar.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
void mechanical_button_pressed_fcn();
                                   // button gets pressed, definition at the end

int button_pressed = 0;

// main runs as an own thread
int main()
{
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

    // start timer
    main_task_timer.start();

    // mechanical button
    //DebounceIn mechanical_button(PC_6); // create DigitalIn object to evaluate mechanical button, you
                                       // need to specify the mode for proper usage, see below
    //mechanical_button.fall(&mechanical_button_pressed_fcn);    // sets pullup between pin and 3.3 V, so that there
                                       // is a defined potential
    // Motors

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;
    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // Differential Drive Robot Kinematics
    
    const float r_wheel = 0.057f / 2.0f; // wheel radius in meters
    const float b_wheel = 0.19f;          // wheelbase, distance from wheel to wheel in meters
    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / b_wheel, -r_wheel / b_wheel;
    //Eigen::Vector2f robot_coord = {0.0f, 0.0f};  // contains v and w (robot translational and rotational velocity)
    //Eigen::Vector2f wheel_speed = {0.0f, 0.0f};  // contains w1 and w2 (wheel speed)

    // set robot velocities
    //robot_coord(0) = 1.0f; // set desired translational velocity in m/s
    //robot_coord(1) = 1.5f; // set desired rotational velocity in rad/s

    // map robot velocities to wheel velocities in rad/sec
    //wheel_speed = Cwheel2robot.inverse() * robot_coord;

    // Line Array Sensor

    // sensor bar
    const float bar_dist = 0.074f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp{4.0f};
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    
    // States
    enum RobotState {
        INITIAL,
        SLEEP,
        FORWARD,
        BACKWARD
    } robot_state = RobotState::INITIAL;


    while (true) {
        main_task_timer.reset();
        // --- code that runs every cycle at the start goes here ---
        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---
            switch (robot_state) {
                case RobotState::INITIAL: {
                    enable_motors = 1;
                    robot_state = RobotState::FORWARD;
                    printf("initial\n");
                    break;
                }
                case RobotState::SLEEP: {
                    printf("sleep\n");
                    if (button_pressed % 2 == 0)
                        robot_state = RobotState::FORWARD;
                    if (button_pressed % 2 != 0)
                        robot_state = RobotState::BACKWARD;
                    break;
                }
                case RobotState::FORWARD: {
                    // setpoints for the dc motors in rps
                    // only update sensor bar angle if an led is triggered
                    if (sensor_bar.isAnyLedActive()) {
                        angle = sensor_bar.getAvgAngleRad();
                    } else {
                        angle = 0.0f;  // kein Signal → geradeaus, nicht drehen
                    }

                    printf("angle: %f", angle);
                   
                   // control algorithm for robot velocities
                   Eigen::Vector2f robot_coord = {0.4f * wheel_vel_max * r_wheel,  // half of the max. forward velocity
                                                  Kp * angle                    }; // simple proportional angle controller

                   // map robot velocities to wheel velocities in rad/sec
                   Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;
                    
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                    printf("RUN:\n");
                    printf("angle: %f | w0: %f | w1: %f\n", 
                        angle, 
                        wheel_speed(0) / (2.0f * M_PIf), 
                        wheel_speed(1) / (2.0f * M_PIf));
                    //printf("M1 max velocity: %f \n", motor_M1.getMaxVelocity());
                    //printf("M1 max physical velocity: %f \n", motor_M1.getMaxPhysicalVelocity());
                    //printf("wheel_speed 0 %f :", wheel_speed(0));
                    //printf("M1 speed 0 %f \n:", wheel_speed(0) / (2.0f * M_PIf));
                    //printf("M1 velocity: %f \n", motor_M1.getVelocity());
                    motor_M2.setVelocity(-wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                    //printf("M2 max velocity: %f \n", motor_M2.getMaxVelocity());
                    //printf("M2 max physical velocity: %f \n", motor_M2.getMaxPhysicalVelocity());
                    //printf("wheel_speed 1 %f :", wheel_speed(1));
                    //printf("M2 speed 1 %f \n:", wheel_speed(1) / (2.0f * M_PIf));
                    //printf("M2 velocity: %f \n", motor_M2.getVelocity());
                    printf("forward\n");
                    //robot_state = RobotState::SLEEP;
                    break;
                }
                case RobotState::BACKWARD: {
                    // move backwards to the initial position
                    // and go to the SLEEP state if reached
                    //motor_M3.setRotation(-3.0f);
                    // switching condition is slightly bigger for robustness
                    printf("backward\n");
                    //if (motor_M3.getRotation() < -2.89f)
                    //    robot_state = RobotState::SLEEP;
                    break;
                }
                default: {

                    break; // do nothing
                }
            }

            // visual feedback that the main task is executed, setting this once would actually be enough
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
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

void mechanical_button_pressed_fcn()
{
    button_pressed++;
    printf("increase button pressed %d", button_pressed);
    return;
}
