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
import frc.robot.util.FormatUtil;
import frc.robot.util.FunctionalUtil;

public class DriveCommand extends Command {
  private final ShuffleboardTab m_dashboard;

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
    m_dashboard = shuffleboardTab;

    m_drive = drive;

    UnaryOperator<Double> deadbandify = i -> 0 - MathUtil.applyDeadband(i, deadband);
    m_strafeX = FunctionalUtil.supplyThenOperate(strafeX, deadbandify);
    m_strafeY = FunctionalUtil.supplyThenOperate(strafeY, deadbandify);
    m_rotation = FunctionalUtil.supplyThenOperate(rotation, deadbandify);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    double strafeX = m_strafeX.get();
    double strafeY = m_strafeY.get();
    double rotation = m_rotation.get();

    m_drive.drive(strafeX, strafeY, rotation);

    m_dashboard.addStringArray("OI",
      FormatUtil.formatted(List.of(
        new Pair<>("v", m_strafeX),
        new Pair<>("w", m_strafeY),
        new Pair<>("ω", m_rotation)
      )));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
