// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"

/* Speed Variables */
float drive_speed = 0.5; // between 0 and 1

/* Joystick variables */
float l_stick_y = 0;
float r_stick_x = 0;
float joystick_threshold_center = 0.05;
float joystick_threshold_gap = 0.03;

/* Hardware (motor, controller, etc.) declarations */
frc::Joystick *controller; // the full controller, confusingly called a 'Joystick'
rev::CANSparkMax left_drive{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax right_drive{1, rev::CANSparkMax::MotorType::kBrushless};

rev::CANEncoder left_drive_enc = left_drive.GetEncoder();
rev::CANEncoder right_drive_enc = right_drive.GetEncoder();


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  controller = new frc::Joystick(0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  /*** THRESHOLD SYSTEM
   *
   * If there is only one threshold, then when the joystick is just on the edge
   * of that threshold, it quickly alternates between on and off, resulting in
   * the seizurebot.
   *
   * To avoid that, there are essentially 2 thresholds, one above the other. If
   * the joystick value is above the top threshold, then it is taken as is. If
   * it is below the lower value, it is ignored (set to 0). If it's in between,
   * it's in a "dead area," and it is left in whatever state it was in before.
   * This prevents the jumping back and forth over the threshold, and thus
   * prevents the seizurebot.
   *
   * The one catch is that it isn't implemented as 2 thresholds. It is
   * implemented as one center threshold and a gap for the size of the "dead
   * zone"
   ***/

  if (std::abs(controller->GetRawAxis(1)) > joystick_threshold_center + (joystick_threshold_gap / 2))
    l_stick_y = controller->GetRawAxis(1);
  else if (std::abs(controller->GetRawAxis(1)) < joystick_threshold_center - (joystick_threshold_gap / 2))
    l_stick_y = 0;

  if (std::abs(controller->GetRawAxis(2)) > joystick_threshold_center + (joystick_threshold_gap / 2))
    r_stick_x = controller->GetRawAxis(2);
  else if (std::abs(controller->GetRawAxis(2)) < joystick_threshold_center - (joystick_threshold_gap / 2))
    r_stick_x = 0;

  left_drive.Set((-l_stick_y + r_stick_x) * drive_speed);
  right_drive.Set((l_stick_y + r_stick_x) * drive_speed);

  /* Write encoder values to console */
  frc::SmartDashboard::PutNumber("Joystick position", controller->GetRawAxis(1));
  frc::SmartDashboard::PutNumber("Left encoder position", left_drive_enc.GetPosition());
  frc::SmartDashboard::PutNumber("Right encoder position", right_drive_enc.GetPosition());
  frc::SmartDashboard::PutNumber("Left encoder velocity", left_drive_enc.GetVelocity());
  frc::SmartDashboard::PutNumber("Right encoder velocity", right_drive_enc.GetVelocity());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
