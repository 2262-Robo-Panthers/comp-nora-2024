// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.util.ShuffleboardTabWithMaps;

public class ShoulderCommand extends Command {
  private final ShoulderSubsystem m_shoulder;

  private final Supplier<Double> m_pivot;

  public ShoulderCommand(
    ShuffleboardTab shuffleboardTab,
    ShoulderSubsystem shoulder,
    Supplier<Double> pivot
  ) {
    m_shoulder = shoulder;

    m_pivot = pivot;

    populateDashboard(shuffleboardTab);

    addRequirements(shoulder);
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, "EEor Controls", "%.3f", List.of(
      new Pair<>("dPivot", m_pivot)
    ));
  }

  @Override
  public void execute() {
    m_shoulder.movePivotPosition(m_pivot.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
