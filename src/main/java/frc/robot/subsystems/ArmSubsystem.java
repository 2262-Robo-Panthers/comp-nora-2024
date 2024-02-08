// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.SmartMotorController.SmartMotorController;

public class ArmSubsystem extends SubsystemBase {
  SmartMotorController m_intake;
  SmartMotorController m_launch;

  public ArmSubsystem(SmartMotorController intake, SmartMotorController launch) {
    m_intake = intake;
    m_launch = launch;
  }

  public void setIntakeSpeed(double velocity) {
    m_intake.forceTo(velocity);
  }

  public void setLaunchSpeed(double velocity) {
    m_launch.forceTo(velocity);
  }
}
