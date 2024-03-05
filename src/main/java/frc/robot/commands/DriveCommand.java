// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.FunctionalUtil;
import frc.robot.util.ShuffleboardTabWithMaps;

public class DriveCommand extends Command {
  private final DriveSubsystem m_drive;

  private final Supplier<Double> m_strafeX;
  private final Supplier<Double> m_strafeY;
  private final Supplier<Double> m_rotation;

  public DriveCommand(
    ShuffleboardTab shuffleboardTab,
    DriveSubsystem drive,
    Supplier<Double> strafeX, Supplier<Double> strafeY, Supplier<Double> rotation,
    double deadband
  ) {
    m_drive = drive;

    UnaryOperator<Double> deadbandify = i -> 0 - MathUtil.applyDeadband(i, deadband);
    m_strafeX = FunctionalUtil.supplyThenOperate(strafeX, deadbandify);
    m_strafeY = FunctionalUtil.supplyThenOperate(strafeY, deadbandify);
    m_rotation = FunctionalUtil.supplyThenOperate(rotation, deadbandify);

    populateDashboard(shuffleboardTab);

    addRequirements(drive);
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, "Drive Controls", "%.3f", List.of(
      new Pair<>("Strafe X", m_strafeX),
      new Pair<>("Strafe Y", m_strafeY),
      new Pair<>("Rotation", m_rotation)
    ))
      .withPosition(0, 0)
      .withSize(2, 2);
  }

  @Override
  public void execute() {
    double strafeX = m_strafeX.get();
    double strafeY = m_strafeY.get();
    double rotation = m_rotation.get();

    m_drive.drive(strafeX, strafeY, rotation);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
