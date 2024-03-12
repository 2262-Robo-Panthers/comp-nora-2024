// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class ArmCommand extends Command {
  private final ArmSubsystem m_arm;

  private final Supplier<Double> m_intake;
  private final Supplier<Double> m_launch;

  public ArmCommand(
    ShuffleboardTab shuffleboardTab,
    ArmSubsystem arm,
    Supplier<Double> intake, Supplier<Double> launch
  ) {
    m_arm = arm;

    m_intake = intake;
    m_launch = launch;

    populateDashboard(shuffleboardTab);

    addRequirements(arm);
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.EEorControl, "%.3f", List.of(
      new Pair<>("Intake", m_intake),
      new Pair<>("Launch", m_launch)
    ));
  }

  @Override
  public void initialize() {
    m_arm.stop();
  }

  @Override
  public void execute() {
    m_arm.setIntakeSpeed(m_intake.get());
    m_arm.setLaunchSpeed(m_launch.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
