// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

// TEMP
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveCommand extends Command {
  private final DriveSubsystem m_drive;

  private final Supplier<Double> m_strafeY;
  private final Supplier<Double> m_strafeX;
  private final Supplier<Double> m_rotation;

  public DriveCommand(DriveSubsystem drive, Supplier<Double> strafeY, Supplier<Double> strafeX, Supplier<Double> rotation) {
    m_drive = drive;
    m_strafeY = strafeY;
    m_strafeX = strafeX;
    m_rotation = rotation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.m_modules[0].freeze();
  }

  @Override
  public void execute() {
    m_drive.m_modules[0].setDesiredState(
      new SwerveModuleState(
        -m_strafeY.get() * DriveConstants.kMaxSpeed_m_s,
        new Rotation2d(
          -m_rotation.get()
            * 0.5 * Math.PI
        )
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.m_modules[0].freeze();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
