// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
  private final ArmSubsystem m_arm;

  private final Supplier<Double> m_input;

  public IntakeCommand(ArmSubsystem arm, Supplier<Double> input) {
    m_arm = arm;
    m_input = input;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.setIntakeSpeed(0.0);
  }

  @Override
  public void execute() {
    m_arm.setIntakeSpeed(m_input.get());
  }

  @Override
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public void end(boolean interrupted) {
    m_arm.setIntakeSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
