// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.SmartMotorController.SmartMotorController;

public class ArmSubsystem extends SubsystemBase {
  private final SmartMotorController m_intake;
  private final SmartMotorController m_launch;

  public ArmSubsystem(
    SmartMotorController intake,
    SmartMotorController launch
  ) {
    m_intake = intake;
    m_launch = launch;
  }

  public void setIntakeSpeed(double velocity) {
    m_intake.forceTo(velocity);
  }

  public void setLaunchSpeed(double velocity) {
    m_launch.forceTo(velocity);
  }

  public void stop() {
    m_intake.forceTo(0.0);
    m_launch.forceTo(0.0);
  }

  public SmartMotorController getIntake() {
    return m_intake;
  }

  public SmartMotorController getLaunch() {
    return m_launch;
  }

  public Command controlCommand(Double intake, Double launch) {
    return runOnce(() -> {
      if (intake != null)
        setIntakeSpeed(intake);

      if (launch != null)
        setLaunchSpeed(launch);
    });
  }
}
