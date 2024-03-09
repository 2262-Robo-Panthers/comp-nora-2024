// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.Extremum;

public class HomeCommand extends Command {
  private final ShoulderSubsystem m_shoulder;

  private final Extremum m_preferredDirection;
  private boolean m_shouldBackOff = false;

  public HomeCommand(ShoulderSubsystem shoulder, Extremum preferredDirection) {
    m_shoulder = shoulder;
    m_preferredDirection = preferredDirection;

    addRequirements(shoulder);
  }

  @Override
  public void initialize() {
    m_shouldBackOff = m_shoulder.isAtLimitLower() ^ m_shoulder.isAtLimitUpper();
  }

  @Override
  public void execute() {
    if (!m_shoulder.isAtLimitLower() && m_preferredDirection == Extremum.kLower) {
      m_shouldBackOff = false;
    }
    else if (!m_shoulder.isAtLimitUpper() && m_preferredDirection == Extremum.kUpper) {
      m_shouldBackOff = false;
    }

    m_shoulder.movePivotPosition(0.5 * (m_preferredDirection == Extremum.kLower ^ m_shouldBackOff ? -1 : 1));
  }

  @Override
  public boolean isFinished() {
    return !m_shouldBackOff && (
      (m_shoulder.isAtLimitLower() && m_preferredDirection == Extremum.kLower) ||
      (m_shoulder.isAtLimitUpper() && m_preferredDirection == Extremum.kUpper)
    );
  }
}
