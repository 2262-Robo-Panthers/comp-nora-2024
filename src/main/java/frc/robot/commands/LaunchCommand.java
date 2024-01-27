// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LaunchSubsystem;

public class LaunchCommand extends Command {
  private final LaunchSubsystem m_launch;

  private final Supplier<Double> m_input;

  public LaunchCommand(LaunchSubsystem launch, Supplier<Double> input) {
    m_launch = launch;
    m_input = input;

    addRequirements(launch);
  }

  @Override
  public void initialize() {
    m_launch.getController().set(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launch.getController().set(m_input.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public void end(boolean interrupted) {
    m_launch.getController().set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
