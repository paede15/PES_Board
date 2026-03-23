#include "mbed.h"
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "FastPWM.h"
#include <Eigen/Dense>
#include "SensorBar.h"
#include "ColorSensor.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and -  decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and - shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user

// sensor thread detection
volatile bool cross_detected = false;
SensorBar* sensor_bar_ptr = nullptr;
void detect_cross_thread();

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    Timer pickup_timer;

    // start timer
    main_task_timer.start();

    // Led on nucleo board
    DigitalOut user_led(LED1);

    // Motors

    DigitalOut enable_motors(PB_ENABLE_DCMOTORS); // create object to enable power electronics for the dc motors
    
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to - 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // Differential Drive Robot Kinematics
    
    const float r_wheel = 0.057f / 2.0f; // wheel radius in meters
    const float b_wheel = 0.19f;          // wheelbase, distance from wheel to wheel in meters
    const float max_speed_percentage{0.6f}; // is used to set the actual max speed

    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / b_wheel, -r_wheel / b_wheel;
    
    // Line Array Sensor

    // sensor bar
    const float bar_dist = 0.074f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};
    const float default_turning_angle(1.0f); // this is the default trurning angle if the linereader read nothing.

    // rotational velocity controller (PD)
    const float Kp{4.7f};
    const float Kd{0.25f};
    const float dt = main_task_period_ms * 1e-3f; // 0.020 s
    float prev_error{0.0f};
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // Color Sensor

    ColorSensor color_sensor(PB_3); // alle Pins auf Arduino-Header, LED floating

    int color{0};

    enum Color {
    UNKNOWN = 0,
    BLACK   = 1,
    WHITE   = 2,
    RED     = 3,
    YELLOW  = 4,
    GREEN   = 5,
    CYAN    = 6,
    BLUE    = 7,
    MAGENTA = 8
    };

    //int sleep_count = 0;

    // States
    enum RobotState {
        INITIAL,
        SLEEP,
        FORWARD,
        COLOR_SCAN,
        PICKUP_PARCEL,
        DROP_PARCEL
    } robot_state = RobotState::INITIAL;

   sensor_bar_ptr = &sensor_bar;
   Thread cross_thread;
   cross_thread.start(detect_cross_thread);    

    while (true) {
        main_task_timer.reset();
        // --- code that runs every cycle at the start goes here ---
        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---
            switch (robot_state) {
                case RobotState::INITIAL: {
                    enable_motors = 1;
                    robot_state = RobotState::FORWARD;
                    break;
                }
                case RobotState::SLEEP: {
                    enable_motors = 0;
                    break;
                }
                case RobotState::FORWARD: {
                    if (sensor_bar.isAnyLedActive()) {
                        angle = sensor_bar.getAvgAngleRad();
                        if (cross_detected) {
                            prev_error = 0.0f; // reset derivative on state change
                            robot_state = RobotState::COLOR_SCAN;
                            break;
                        }
                    } else {
                        angle = default_turning_angle; // no line read -> positive rotation
                    }

                    // PD controller for rotational velocity
                    float derivative = (angle - prev_error) / dt;
                    float omega      = -(Kp * angle + Kd * derivative);
                    prev_error       = angle;

                    // map robot velocities to wheel velocities in rad/sec
                    Eigen::Vector2f robot_coord = {max_speed_percentage * wheel_vel_max * r_wheel, omega};
                    Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity( wheel_speed(0) / (2.0f * M_PIf));
                    motor_M2.setVelocity(-wheel_speed(1) / (2.0f * M_PIf));

                    //printf("angle: %f | omega: %f\n", angle, omega);
                    break;
                }
                case RobotState::COLOR_SCAN: {
                    color_sensor.switchLed(ON);
    
                    int detected = color_sensor.getColor();
                    
                    if (detected == YELLOW ||
                        detected == GREEN  ||
                        detected == RED    ||
                        detected == BLUE) {
                        // Farbe gefunden
                        color = detected;
                        detected = 0;
                        printf("Color: %s\n", ColorSensor::getColorString(color));
                        color_sensor.switchLed(OFF);
                        robot_state = RobotState::PICKUP_PARCEL;
                    } else {
                        // noch keine Farbe -> weiterfahren
                        Eigen::Vector2f robot_coord = {0.1f * wheel_vel_max * r_wheel, 0.0f};
                        Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;
                        motor_M1.setVelocity( wheel_speed(0) / (2.0f * M_PIf));
                        motor_M2.setVelocity(-wheel_speed(1) / (2.0f * M_PIf));
                    }
                    break;
                }
                case RobotState::PICKUP_PARCEL: {
                    // Motoren stoppen
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    user_led = 1;
                    
                    // Timer beim ersten Mal starten
                    static bool timer_started = false;
                    if (!timer_started) {
                        timer_started = true;
                        pickup_timer.start();
                    }
                    
                    // nach 5 Sekunden weiter
                    if (duration_cast<milliseconds>(pickup_timer.elapsed_time()).count() >= 5000) {
                        timer_started = false;
                        pickup_timer.stop();
                        pickup_timer.reset();
                        cross_detected = false;
                        robot_state = RobotState::FORWARD;
                    }
                    break;
                }
                case RobotState::DROP_PARCEL: {
                    //printf("backward\n");
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
            }
        }

        // toggling the user led
        //user_led = !user_led;

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

void detect_cross_thread() {
    while (true) {
        uint8_t raw = sensor_bar_ptr->getRaw();
        if (raw == 0x3c) {
            cross_detected = true;
        }
        ThisThread::sleep_for(4ms); // gleich schnell wie SensorBar intern
    }
}
