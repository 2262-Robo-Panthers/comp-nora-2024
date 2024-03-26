// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
  private final ArmSubsystem m_arm;
  private final DigitalInput m_photogate;
  private final double m_velocity;

  public IntakeCommand(ArmSubsystem arm, DigitalInput photogate, double velocity) {
    m_arm = arm;
    m_photogate = photogate;
    m_velocity = velocity;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.setIntakeSpeed(m_velocity);

    System.out.printf("IntakeCommand attempting to intake note with velocity %f...\n", m_velocity);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setIntakeSpeed(0.0);

    System.out.printf("IntakeCommand %s.\n", interrupted ? "was interrupted" : "has finished");
  }

  @Override
  public boolean isFinished() {
    return m_photogate.get();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }
}
