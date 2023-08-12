// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/TimedRobot.h>
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

  static const int leftLeadDeviceID = 12; // 12
  static const int leftFollowDeviceID = 13;
  static const int rightLeadDeviceID = 15; // 15
  static const int rightFollowDeviceID = 14;

  static const int lcompressorID = 1;
  static const int rcompressorID = 2;

  rev::CANSparkMax* m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  bool reached_max_pressure = false;

  rev::CANSparkMax* lcompressor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rcompressor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  frc::Joystick * joystick = new frc::Joystick(0);
  frc::PneumaticsControlModule * pcm = new frc::PneumaticsControlModule(20);

  frc::Solenoid * solenoidValve = new frc::Solenoid(20, frc::PneumaticsModuleType::CTREPCM, 1);
};
