#include "mbed.h"
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "FastPWM.h"
#include <Eigen/Dense>
#include "SensorBar.h"
#include "ColorSensor.h"
#include "Servo.h"
//#include "IIRFilter.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and -  decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and - shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user

// sensor thread detection
volatile bool pickup_cross  = false;
volatile bool drop_cross    = false;
volatile bool color_detected = false;
volatile int cross_bar_count{0};
volatile int cross_lock_ticks = 0;
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
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio, kn, voltage_max);

    // Motion-PLanner for M3 position controll
    motor_M3.enableMotionPlanner();
    motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.75f);

    // servo (min/max pulse widths from calibration)
    Servo servo_D0(PB_D0, 0.03f, 0.125f);
    Servo servo_D1(PB_D1, 0.03f, 0.1f);

    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(0.3f);
    servo_D1.setMaxAcceleration(0.3f);

    const float servo_D0_home = 0.8f;
    const float servo_D1_home = 0.6f;

    // servo calibration variables
    float servo_input = 0.0f;
    int servo_counter = 0;
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // Differential Drive Robot Kinematics
    
    const float r_wheel = 0.03f / 2.0f; // wheel radius in meters
    const float b_wheel = 0.1408f;          // wheelbase, distance from wheel to wheel in meters
    const float max_speed_percentage{1.0f}; // is used to set the actual max speed

    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / b_wheel, -r_wheel / b_wheel;
    
    // Line Array Sensor

    // sensor bar
    const float bar_dist = 0.18f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};
    const float default_turning_angle(1.0f); // this is the default trurning angle if the linereader read nothing.

    // rotational velocity controller (PD)
    const float Kp{12.5f};
    const float Kd{0.25f};
    const float dt = main_task_period_ms * 1e-3f; // 0.020 s
    float prev_error{0.0f};
    //IIRFilter angle_filter;
    //angle_filter.lowPass1Init(5.0f, dt); //low pass filter 5Hz to reduce Sesnor noise.
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // Color Sensor

    ColorSensor color_sensor(PB_3); // alle Pins auf Arduino-Header, LED floating

    int  current_color_run{0};
    bool color_done[9] = {};
    int  pickups_done  = 0;

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
        HANDLE_PARCEL,
        CALIBRATE_COLOR,
        CALIBRATE_SERVO
    } robot_state = RobotState::INITIAL;

   sensor_bar_ptr = &sensor_bar;
   Thread cross_thread;
   cross_thread.start(detect_cross_thread);

    bool is_pickup = true;

    // Crane_Controll.
    // controll parameters for crane: m3_dc_offset, srv_d0_offset, srv_d1_offset
    struct ParcelParams { float m3_offset, srv_out_d0, srv_out_d1; };
    const ParcelParams pickup_params[9] = {
        {},                            // UNKNOWN
        {},                            // BLACK
        {},                            // WHITE
        {1.75f, 0.95f, 0.15f },         // RED down right
        {1.75f, 0.95f, 0.2f },         // YELLOW up right
        {-0.55f, 0.85f, 0.1f },         // GREEN down left
        {},                            // CYAN
        {-0.55f, 1.0f, 0.15f},         // BLUE up left
        {},                            // MAGENTA
    };
    const ParcelParams drop_params[9] = {
        {},
        {},
        {},
        { 1.75f, 0.95f, 0.1f },         // RED
        { 1.75f, 0.9f, 0.2f },         // YELLOW
        {-0.55f, 0.85f, 0.1f },         // GREEN
        {},
        {-0.6f, 1.1f, 0.17f },         // BLUE
        {},
    };

    while (true) {
        main_task_timer.reset();
        // --- code that runs every cycle at the start goes here ---
        if (do_execute_main_task) {
            // --- code that runs when the blue button was pressed goes here ---
            switch (robot_state) {
                case RobotState::INITIAL: {
                    enable_motors = 1;
                    pickup_cross  = false;
                    drop_cross    = false;
                    cross_bar_count = 0;
                    if (!servo_D0.isEnabled()) servo_D0.enable();
                    if (!servo_D1.isEnabled()) servo_D1.enable();
                    servo_D0.setPulseWidth(servo_D0_home);
                    servo_D1.setPulseWidth(servo_D1_home);
                    if (fabsf(servo_D0.getPulseWidth() - servo_D0_home) < 0.02f &&
                        fabsf(servo_D1.getPulseWidth() - servo_D1_home) < 0.02f) {
                        robot_state = (pickups_done >= 4) ? RobotState::SLEEP : RobotState::FORWARD;
                    }
                    break;
                }
                case RobotState::SLEEP: {
                    enable_motors = 0;
                    break;
                }
                case RobotState::FORWARD: {
                    if (sensor_bar.isAnyLedActive()) {
                        uint8_t raw = sensor_bar.getRaw();
                        
                        int active_leds = 0;
                        for (int i = 0; i < 8; i++) {
                            if (raw & (1 << i)) {
                                active_leds++;
                            }
                        }
                        if (is_pickup == false && active_leds >= 4){
                            angle = -0.15f;
                        }
                        
                        else {
                            angle = sensor_bar.getAvgAngleRad();
                        }
                        
                        if ((pickup_cross || drop_cross) && !color_detected) {
                            prev_error = 0.0f;
                            robot_state = RobotState::COLOR_SCAN;
                            break;
                        }
                    } else {
                        //angle = 0.0f; //wenn nichts geradeaus        
                        angle = default_turning_angle; // no line read -> positive rotation
                    }

                    // PD controller for rotational velocity
                    color_detected = false;
                    const float angle_threshold = -0.3;
                    const float derivative_threshold = Kd * angle_threshold; // Kd * -0.3
                    const float derivative_correction_factor = 16.0f;
                    float derivative = (angle - prev_error) / dt;
                    float omega      = -(Kp * angle + Kd * derivative);
                    prev_error       = angle;

                    //if (Kd * derivative < derivative_threshold) {
                    //    omega *= derivative_correction_factor;
                    //    //printf("omega: %f, angle: %f, Kd, %f, derivative %f\n", omega, angle, Kd, derivative);
                    //}


                    // slow down on sharp turns, speed up on straights                                                                                                      
                    //float speed_factor = 1.0f - fabsf(angle) / default_turning_angle;                                                                                     
                    //speed_factor = (speed_factor < 0.3f) ? 0.3f : speed_factor;

                    // map robot velocities to wheel velocities in rad/sec
                    Eigen::Vector2f robot_coord = {max_speed_percentage * wheel_vel_max * r_wheel, omega};
                    //Eigen::Vector2f robot_coord = {speed_factor * max_speed_percentage * wheel_vel_max * r_wheel, omega};
                    Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
                    motor_M2.setVelocity(-wheel_speed(1) / (2.0f * M_PIf));

                    //printf("angle: %f | omega: %f\n | lin_speed: %f\n", angle, omega, max_speed_percentage * wheel_vel_max * r_wheel);
                    break;
                }
                case RobotState::COLOR_SCAN: {
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    color_sensor.switchLed(ON);
    
                    int detected = color_sensor.getColor();
                    
                    if (detected == YELLOW ||
                        detected == GREEN  ||
                        detected == RED    ||
                        detected == BLUE) {
                        color_sensor.switchLed(OFF);
                        if (!color_done[detected] && pickup_cross && current_color_run == 0) {
                            color_done[detected] = true;
                            current_color_run = detected;
                            color_detected = true;
                            //printf("Color: %s\n", ColorSensor::getColorString(current_color_run));
                            is_pickup   = true;
                            robot_state = RobotState::HANDLE_PARCEL;
                            //robot_state = RobotState::FORWARD;
                        } else if (current_color_run == detected){
                            // already picked — this is the drop-off spot
                            drop_cross  = false;
                            pickup_cross = false;
                            is_pickup   = false;
                            robot_state = RobotState::HANDLE_PARCEL;
                            //robot_state = RobotState::FORWARD;
                        } else {
                            drop_cross  = false;
                            pickup_cross = false;
                            robot_state = RobotState::FORWARD;
                        }
                    }
                    break;
                }
                case RobotState::HANDLE_PARCEL: {
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    user_led = 1;

                    static enum class HandleStep {
                        MOVE_OUT_SRV1,  // 1. move servo D1 (gripper)
                        MOVE_OUT_SRV0,  // 2. move servo D0 (arm)
                        MOVE_OUT_M3,    // 3. extend crane
                        MOVE_BACK_M3,   // 4. retract crane
                        MOVE_BACK_SRVS, // 5. return both servos
                    } step = HandleStep::MOVE_OUT_SRV1;
                    static float m3_target = 0.0f;

                    const ParcelParams& p = is_pickup ? pickup_params[current_color_run]
                                                      : drop_params[current_color_run];

                    switch (step) {
                        case HandleStep::MOVE_OUT_SRV1: {
                            if (!servo_D1.isEnabled()) servo_D1.enable();
                            servo_D1.setPulseWidth(p.srv_out_d1);
                            if (fabsf(servo_D1.getPulseWidth() - p.srv_out_d1) < 0.02f)
                                step = HandleStep::MOVE_OUT_SRV0;
                            break;
                        }
                        case HandleStep::MOVE_OUT_SRV0: {
                            if (!servo_D0.isEnabled()) servo_D0.enable();
                            servo_D0.setPulseWidth(p.srv_out_d0);
                            if (fabsf(servo_D0.getPulseWidth() - p.srv_out_d0) < 0.02f) {
                                m3_target = motor_M3.getRotation() + p.m3_offset;
                                step = HandleStep::MOVE_OUT_M3;
                            }
                            break;
                        }
                        case HandleStep::MOVE_OUT_M3: {
                            motor_M3.setRotation(m3_target);
                            if (fabsf(motor_M3.getRotation() - m3_target) < 0.05f) {
                                m3_target = motor_M3.getRotation() - p.m3_offset;
                                step = HandleStep::MOVE_BACK_M3;
                            }
                            break;
                        }
                        case HandleStep::MOVE_BACK_M3: {
                            motor_M3.setRotation(m3_target);
                            if (fabsf(motor_M3.getRotation() - m3_target) < 0.05f)
                                step = HandleStep::MOVE_BACK_SRVS;
                            break;
                        }
                        case HandleStep::MOVE_BACK_SRVS: {
                            servo_D0.setPulseWidth(servo_D0_home);
                            servo_D1.setPulseWidth(servo_D1_home);
                            if (fabsf(servo_D0.getPulseWidth() - servo_D0_home) < 0.02f &&
                                fabsf(servo_D1.getPulseWidth() - servo_D1_home) < 0.02f) {
                                step             = HandleStep::MOVE_OUT_SRV1;
                                pickup_cross     = false;
                                drop_cross       = false;
         

                       cross_lock_ticks = 125;
                                pickups_done++;
                                if (!is_pickup) current_color_run = 0;
                                robot_state = RobotState::INITIAL;
                            }
                            break;
                        }
                    }
                    break;
                }
                case RobotState::CALIBRATE_COLOR: {
                    color_sensor.switchLed(ON);
                    const float* colors = color_sensor.readColor();
                    printf("R: %.2f\t G: %.2f\t B: %.2f\t C: %.2f\n", colors[0], colors[1], colors[2], colors[3]);
                    break;
                }
                case RobotState::CALIBRATE_SERVO: {
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    if (!servo_D1.isEnabled())
                        servo_D1.enable();

                    //servo_D0.setPulseWidth(servo_input);
                    servo_D1.setPulseWidth(servo_input);

                    if ((servo_input < 1.0f) &&
                        (servo_counter % loops_per_seconds == 0) &&
                        (servo_counter != 0))
                        servo_input += 0.005f;
                    servo_counter++;

                    printf("Pulse width: %f\n", servo_input);
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

                // reset servo calibration
                servo_D0.disable();
                servo_D1.disable();
                servo_input = 0.0f;
                servo_counter = 0;
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
    bool prev_full_line = false;
    bool prev_cross_line = false;
    while (true) {
        uint8_t raw = sensor_bar_ptr->getRaw();
        bool full_line  = (raw == 0xff);
        bool cross_line = (raw == 0x3c);

        if (cross_lock_ticks > 0) cross_lock_ticks--;

        if (cross_lock_ticks == 0) {
            // rising edge on center-only pattern → drop
            if (!drop_cross && cross_line && !prev_cross_line)
                drop_cross = true;

            // rising edge on full bar → pickup (skip first)
            if (!pickup_cross && full_line && !prev_full_line) {
                // corss_bar_count skips the first cross_line to get on the track
                if (cross_bar_count == 0) cross_bar_count++;
                else pickup_cross = true;
            }
        }

        prev_full_line  = full_line;
        prev_cross_line = cross_line;
        ThisThread::sleep_for(4ms);
    }
}
