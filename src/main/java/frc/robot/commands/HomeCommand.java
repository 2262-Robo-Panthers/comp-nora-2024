// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ShoulderSubsystem.Extremum;
import frc.robot.Constants.PivotConstants;

public class HomeCommand extends Command {
  private final ShoulderSubsystem m_shoulder;

  private final Extremum m_preferredDirection;
  private boolean m_finishedBackOff = false;

  private final double m_searchSpeed;

  public HomeCommand(ShoulderSubsystem shoulder, Extremum preferredDirection, double searchSpeed) {
    m_shoulder = shoulder;
    m_preferredDirection = preferredDirection;
    m_searchSpeed = searchSpeed;

    addRequirements(shoulder);
  }

  public HomeCommand(ShoulderSubsystem shoulder, Extremum preferredDirection) {
    this(shoulder, preferredDirection, PivotConstants.kSensitivity);
  }

  @Override
  public void initialize() {
    m_finishedBackOff = !(m_shoulder.isAtLimitLower() ^ m_shoulder.isAtLimitUpper());

    System.out.printf("HomeCommand prefers %s %s to back off.\n",
      m_preferredDirection.toString(),
      m_finishedBackOff ? "but needs" : "and doesn't need");
  }

  @Override
  public void execute() {
    if (!m_finishedBackOff
      && (!m_shoulder.isAtLimitLower() && m_preferredDirection == Extremum.kLower)
      || (!m_shoulder.isAtLimitUpper() && m_preferredDirection == Extremum.kUpper)) {
      m_finishedBackOff = true;
      System.out.println("HomeCommand finished backing off.");
    }

    m_shoulder.movePivotPosition(m_searchSpeed * (m_preferredDirection == Extremum.kLower ^ !m_finishedBackOff ? -1.0 : +1.0));
  }

  @Override
  public boolean isFinished() {
    return m_finishedBackOff && (
      (m_shoulder.isAtLimitLower() && m_preferredDirection == Extremum.kLower) ||
      (m_shoulder.isAtLimitUpper() && m_preferredDirection == Extremum.kUpper)
    );
  }

  @Override
  public void end(boolean interrupted) {
    System.out.printf("HomeCommand %s.\n",
      interrupted ? "was interrupted" : "is completed");
  }
}
