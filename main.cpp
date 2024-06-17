#include "Arduino.h"
#include "C610Bus.h"
#include <array>
#include "BasicLinearAlgebra.h"
#include "pid.h"
#include "kinematics.h"
#include "test_inv_kinematics.h"
#include "utils.h"
#include <bits/stdc++.h>

#define DO_TESTS =

const BodySide kLegSide = BodySide::kRight;

long last_command = 0;  // To keep track of when we last commanded the motors
C610Bus<CAN2> bus_back; // Initialize the Teensy's CAN bus to talk to the rear Pupper motors
C610Bus<CAN1> bus_front;

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates.

const float Kp = 5000;
const float Kd = 250;
const float kMaxCurrent = 8000;

// Define the signed hip offset and link lengths
const KinematicsConfig pupper_leg_config = KinematicsConfig{0.035, 0.08, 0.11};

BLA::Matrix<3> bus_back_actuator_angles{0, 0, 0};     // rad
BLA::Matrix<3> bus_back_actuator_velocities{0, 0, 0}; // rad/s
BLA::Matrix<3> bus_back_actuator_commands{0, 0, 0};   // mA

BLA::Matrix<3> bus_front_actuator_angles{0, 0, 0};     // rad
BLA::Matrix<3> bus_front_actuator_velocities{0, 0, 0}; // rad/s
BLA::Matrix<3> bus_front_actuator_commands{0, 0, 0};   // mA
void setup()
{
    clear_serial_buffer();
    wait_for_key('s', "Remember to start the arms vertically.");
#ifdef DO_TESTS
    test_inv_kinematics();
#endif
}

float flip = 1.0;

void loop()
{
    bus_back.PollCAN(); // Check for messages from the motors.
    bus_front.PollCAN();
    long now = millis();

    // Check to see if we received a 's' and if so, stop the program.
    if (Serial.available())
    {
        if (Serial.read() == 's')
        {
            bus_back.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
            bus_front.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
            Serial.println("Stopping.");
            while (true)
            {
            }
        }
    }

    // Check to see if it's time to run our control loop again.
    if (now - last_command >= LOOP_DELAY_MILLIS)
    {
        for (int i = 0; i < 3; i++)
        {
            bus_back_actuator_angles(i) = bus_back.Get(i).Position();
            bus_back_actuator_velocities(i) = bus_back.Get(i).Velocity();
            bus_front_actuator_angles(i) = bus_front.Get(i).Position();
            bus_front_actuator_velocities(i) = bus_front.Get(i).Velocity();
        }
        /**
         *  1. Calculate target Cartesian coordinate from the BUS-BACK arm's joint angles using forward kinematics.
         *  2. Add 0.14 to the x-coordinate of the target to meet the other arm's position.
         *  3. Compute joint angles for the BUS-FRONT arm to reach the target position using inverse kinematics.
         *  4. Generate actuator commands for the BUS-FRONT arm using vectorized PD control.
         *
         *  Expected code length ~ 4 lines
         *  Note: You can use print_vector to print vectors for debugging
         */
        // ========================= START CODE HERE ===========================
        BLA::Matrix<3> targetCart = forward_kinematics(bus_back_actuator_angles, pupper_leg_config);
        targetCart(0) += 0.14;
        bus_front_actuator_angles = inverse_kinematics(targetCart, pupper_leg_config, bus_front_actuator_angles);
        bus_front_actuator_commands = vectorized_pd(bus_front_actuator_angles,
                                                          bus_front_actuator_velocities,
                                                          bus_front_actuator_velocities,
                                                          Kp,
                                                          Kd);
                                                          
        bus_front_actuator_commands = vectorized_sanitize(bus_front_actuator_commands,
                                                          bus_front_actuator_angles,
                                                          bus_front_actuator_velocities);
        // ========================= END CODE HERE ===========================
        bus_front_actuator_commands = vectorized_sanitize(bus_front_actuator_commands,
                                                          bus_front_actuator_angles,
                                                          targetCart,
                                                          kMaxCurrent);

        // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
        bus_front.CommandTorques(bus_front_actuator_commands(0), bus_front_actuator_commands(1), bus_front_actuator_commands(2), 0, C610Subbus::kOneToFourBlinks);

        last_command = now;
        Serial.println("norm is:");
        Serial.println(std::norm(5));
        Serial.println();
    }
}