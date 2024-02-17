// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
  private final ArmSubsystem m_arm;

  private final Supplier<Double> m_pivot;
  private final Supplier<Double> m_intake;
  private final Supplier<Double> m_launch;

  public ArmCommand(ArmSubsystem arm, Supplier<Double> pivot, Supplier<Double> intake, Supplier<Double> launch) {
    m_arm = arm;
    m_pivot = pivot;
    m_intake = intake;
    m_launch = launch;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.stop();
  }

  @Override
  public void execute() {
    m_arm.setPivotSpeed_TEMP(m_pivot.get());
    m_arm.setIntakeSpeed(m_intake.get());
    m_arm.setLaunchSpeed(m_launch.get());
  }

  @Override
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
