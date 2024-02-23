// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.FunctionalUtil;
import frc.robot.Constants.DriveConstants;

// TEMP
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveCommand extends Command {
  private final DriveSubsystem m_drive;

  private final Supplier<Double> m_strafeY;
  private final Supplier<Double> m_strafeX;
  private final Supplier<Double> m_rotation;

  public DriveCommand(DriveSubsystem drive, Supplier<Double> strafeY, Supplier<Double> strafeX, Supplier<Double> rotation, double deadband) {
    m_drive = drive;

    UnaryOperator<Double> deadbandify = i -> MathUtil.applyDeadband(i, deadband);
    m_strafeY = FunctionalUtil.supplyThenOperate(strafeY, deadbandify);
    m_strafeX = FunctionalUtil.supplyThenOperate(strafeX, deadbandify);
    m_rotation = FunctionalUtil.supplyThenOperate(rotation, deadbandify);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.zeroVelocity();
  }

  @Override
  public void execute() {
    m_drive.setDesiredStates_TEMP(
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
    m_drive.zeroVelocity();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
