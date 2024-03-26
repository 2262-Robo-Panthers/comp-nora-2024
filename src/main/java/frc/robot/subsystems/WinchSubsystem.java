// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.SmartMotorController.SmartMotorController;

public class WinchSubsystem extends SubsystemBase {
  private final SmartMotorController m_motor;

  public WinchSubsystem(SmartMotorController motor) {
    m_motor = motor;
  }

  public void setClimbSpeed(double velocity) {
    m_motor.forceTo(velocity);
  }

  public void stop() {
    m_motor.forceTo(0.0);
  }

  public SmartMotorController getMotor() {
    return m_motor;
  }

  public Command controlCommand(Double climb) {
    return runOnce(() -> {
      if (climb != null)
        setClimbSpeed(climb);
    });
  }
}
