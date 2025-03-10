// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.IntakeCommand;
import frc.robot.lib.SmartMotorController.SmartMotorController;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SmartMotorController m_intake;
  private final SmartMotorController m_launch;
  private final DigitalInput m_photogate;
  private final DigitalOutput m_ledStrip;

  public ArmSubsystem(
    ShuffleboardTab dashboard,
    SmartMotorController intake,
    SmartMotorController launch,
    DigitalInput photogate,
    DigitalOutput ledStrip
  ) {
    m_intake = intake;
    m_launch = launch;
    m_photogate = photogate;
    m_ledStrip = ledStrip;

    populateDashboard(dashboard);
  }

  @Override
  public void periodic() {
    m_ledStrip.set(!m_photogate.get());
  }

  public void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.ArmInfo, List.of(
      new Pair<>("Has Note?", m_photogate::get)
    ), false);
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

  public Command intakeCommand() {
    return new IntakeCommand(this, m_photogate, 1.0);
  }
}
