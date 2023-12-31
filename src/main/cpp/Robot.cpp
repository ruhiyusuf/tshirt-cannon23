// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>


void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();

  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  m_leftLeadMotor->SetInverted(true);
  m_rightLeadMotor->SetInverted(false);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  }

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
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);
  m_robotDrive->ArcadeDrive(left_y, -1.0 * right_x);    

  // reached_max_pressure = (pcm->GetPressureSwitch());
  if (ctr->GetAButtonPressed() && !reached_max_pressure){
      lcompressor->Set(1.0);
      rcompressor->Set(1.0);
  }
  
  if (ctr->GetBButtonPressed() || reached_max_pressure){
      lcompressor->Set(0.0);
      rcompressor->Set(0.0);
  }

  if (ctr->GetXButton()) {
    solenoidValve->Set(1);
  } else {
    solenoidValve->Set(0);
  }
    
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
