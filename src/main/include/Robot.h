// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/AnalogInput.h>
#include <rev/CANSparkMax.h>
#include "SFDrive.h"
#include <frc/Solenoid.h>
#include <frc/PneumaticsControlModule.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  static const int leftLeadDeviceID = 1; // 8
  //static const int leftFollowDeviceID = 13;
  static const int rightLeadDeviceID = 4; // 11
  //static const int rightFollowDeviceID = 14;

  double left_y = 0.0;
  double right_x = 0.0; 

  static const int lcompressorID = 8;
  static const int rcompressorID = 15;
  

  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  bool reached_max_pressure = false;

  frc::Joystick* m_stick = new frc::Joystick{0};

  SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor);

  rev::CANSparkMax* lcompressor = new rev::CANSparkMax(lcompressorID, rev::CANSparkMax::MotorType::kBrushed);
  rev::CANSparkMax* rcompressor = new rev::CANSparkMax(rcompressorID, rev::CANSparkMax::MotorType::kBrushed);
  frc::XboxController * ctr = new frc::XboxController(0);
  // frc::PneumaticsControlModule * pcm = new frc::PneumaticsControlMocdule(20);

  frc::Solenoid * solenoidValve = new frc::Solenoid(20, frc::PneumaticsModuleType::CTREPCM, 0);
};