#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;     // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates. Change this in Step 10. 

const float m1_offset = 0.0;
const float m2_offset = 0.0;

 //Step 6. Function that returns the commanded current according to bang bang control
 float bang_bang_control(float m_pos) {
    float I_command = 800.0;
    if(m_pos < 0.0) {
      return I_command;
    } else {
      return -1.0 * I_command;
  }
 }

// Step 7. TODO Implement P control here
float proportional_control(float theta_cur, float theta_target, float Kp) {
    return 0.0; // current in mA
}

// Step 8. TODO Implement D control here. This function assumes that the desired velocity is determined by the arm fully stopping.
float derivative_control(float omega_cur, float Kd) {
    return 0.0; // Current in mA
}

// PD Control code, draws from proportional_control and derivative_control. This function assumes that the desired velocity is determined by the arm fully stopping. Returns a commanded current
float pd_control(float pos, float vel, float target, float Kp, float Kd)
{
    float I_command = proportional_control(pos, target, Kp) + derivative_control(vel, Kd); 
    return I_command; // Current in mA
}

void sanitize_current_command(float &command,float pos, float vel, float max_current = 2000, float max_pos = 3.141, float max_vel = 30, float reduction_factor = 0.1)
{
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    Serial.println("ERROR: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    command = 0;
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
  }
}

// Motorstate struct for keeping the updates together
typedef struct {
  float pos;
  float vel;
  float cmd;
} MotorState;

MotorState m_state; // MotorStrate struct object for updating the motor state of the left leg

// Update in Steps 7-11. PD Gains used in the updateCmd function in pd_control.
float Kp = 1000.0;
float Kd = 0.0;

// updates the given motorstate object with its current motor position and velocity according to the motor ID
void updateState(MotorState* state, int id) {
  state->pos = bus.Get(id).Position(); // update position in radians
  state->vel = bus.Get(id).Velocity(); // update velocity in radians/sec
}

// updates the given MotorStates command current
void updateCmd(MotorState* state, float target, float kp, float kd) {
  state->cmd = pd_control(state->pos, state->vel, target, kp, kd); // use this line for PD Control in Steps 7-11
  //state->cmd = bang_bang_control(state->pos); // TODO use this line for bang bang control in Step 6. Comment again before Step 7.
}

// This code waits for the user to type s before executing code.
void setup()
{
  // Remove all characters that might have been stored up in the serial input buffer prior to running this program
  while (Serial.available()) {
    Serial.read();
  }
  long last_print = millis();
  while (true)
  {
    char c = Serial.read();
    if (c == 's')
    {
      Serial.println("Starting code.");
      break;
    }
    if (millis() - last_print > 2000) {
      Serial.println("Press s to start.");
      last_print = millis();
    }
  }
}

void loop()
{
  bus.PollCAN(); // Check for messages from the motors.
  long now = millis();

  // if for breaking out with 's'
  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    updateState(&m_state, 0); // set your motor state to be ID=0, get current motor position and velocity

    // Block to print out the current position and velocity of the motor
    Serial.print("m_pos: ");
    Serial.print(m_state.pos); // Print the shaft position of motor 0 in radians.
    Serial.print("\tm_vel: ");
    Serial.print(m_state.vel); // Print the shaft velocity of motor 0 in radians/sec.

    float target_position = 0.0; // set target_position to the initial starting position (straight up) for Steps 5-10.
    //float time = millis() / 1000.0; // used in Step 11. for calculating periodic target positions
    //float target_position = sin(time); // set sinusoidal target position in Step 11.

    updateCmd(&m_state, target_position, Kp, Kd); // updates the MotorState object current command based on pd_control values

    sanitize_current_command(m_state.cmd, m_state.pos, m_state.vel); // Sanitizes your computed current commands to make the robot safer.
    bus.CommandTorques(m_state.cmd, 0, 0, 0, C610Subbus::kOneToFourBlinks); // Only call CommandTorques once per loop! Calling it multiple times will override the last command.

    last_command = now;
    Serial.println();
  }
}