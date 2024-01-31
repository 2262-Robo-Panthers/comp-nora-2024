// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.SmartMotorController.SmartMotorController;

public class ArmSubsystem extends SubsystemBase {
  SmartMotorController m_launch;

  public ArmSubsystem(SmartMotorController launch) {
    m_launch = launch;
  }

  public void setLauncherSpeed(double velocity) {
    m_launch.forceTo(velocity);
  }
}
