// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/Servo.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMMotorController.h>

#include <cstdio>
#include <cameraserver/CameraServer.h>

#include <frc/RobotController.h>
//add these for SparkMax CAN 2 pairs of CIM motors (brushed)
#include <frc/drive/MecanumDrive.h>
#include <rev/CANSparkMax.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/motorcontrol/PWMSparkMax.h>

//Pneumatics
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>


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

//drive 2 pairs of brushed CIM motors using CAN & SparkMax motor controller
//static const int leftLeadDeviceID = 5, leftFollowDeviceID = 6, rightLeadDeviceID = 7, rightFollowDeviceID = 8;
//rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
//rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
//rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
//rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  //const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  private:
  static constexpr int kFrontLeftChannel = 8;
  static constexpr int kRearLeftChannel = 7;
  static constexpr int kFrontRightChannel = 5;
  static constexpr int kRearRightChannel = 6;
//CAN
  rev::CANSparkMax m_frontLeft{kFrontLeftChannel, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rearLeft{kRearLeftChannel, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_frontRight{kFrontRightChannel, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rearRight{kRearRightChannel, rev::CANSparkMax::MotorType::kBrushed};

  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};


  int it = 0;
  int ia = 0;

  frc::XboxController m_driverController{0};
  //frc::DigitalInput   limit_switch{9};
  frc::AnalogInput    ai{0};
  frc::AnalogInput    ultrasonic{1};
  

  
  frc::Timer         m_timer;

  std::string_view   DriveCam;

  //Pneumatics
  //pneumatics cylindar to raise and lower arm
  frc::DoubleSolenoid m_updownarm{1, frc::PneumaticsModuleType::REVPH, 0, 1};
  
  //Pneumatics cylindar to open and close grabber
  frc::DoubleSolenoid  m_grabber{1, frc::PneumaticsModuleType::REVPH, 2, 3};



  frc::Compressor phCompressor{1, frc::PneumaticsModuleType::REVPH};

  //PWM
  frc::Spark          m_left_motor_armextender{9};
  frc::Spark          m_right_motor_armextender{8};

  frc::MotorControllerGroup m_armextender{m_left_motor_armextender,  m_right_motor_armextender};


  bool leftbumper = false;
  bool rightbumper = false;

  double lefttriggeraxis = 0.0;
  double righttriggeraxis = 0.0;

  bool bA = false;
  bool bB = false;
  bool bX = false;
  bool bY = false;
  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;

  //bool limit_switch_value = false;
  //bool knife_switch_value = false;

  int ai_raw  = 0;
  double ai_voltage = 0.0;

  int ultra_raw = 0;
  double currentDistanceCentimeters = 0;
  double currentDistanceInches = 0;
  //double distance = 0.0;
  //double adc_to_mm = 1.25 / 1000.0 ;

  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double zRotation = 0.0;

  double kOff = 0.0;
  double kReverse = 0.0;
  double kForward = 0.0;

  double voltage_scale_factor = 0.0;

  //int kSize640x480 = 0;
};


