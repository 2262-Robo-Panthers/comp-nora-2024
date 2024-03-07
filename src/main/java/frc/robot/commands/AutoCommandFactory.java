// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

public class AutoCommandFactory {
  private static Command movementHelper(DriveSubsystem drive, List<Pose2d> points) {
    return new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        points,
        new TrajectoryConfig(2.0, 2.0) {{
          setKinematics(DriveConstants.kDriveKinematics);
        }}
      ),
      drive::getPose,
      DriveConstants.kDriveKinematics,
      new PIDController(1.0, 0.0, 0.0),
      new PIDController(1.0, 0.0, 0.0),
      new ProfiledPIDController(1.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(Math.PI, Math.PI)) {{
        enableContinuousInput(-Math.PI, Math.PI);
      }},
      drive::setModuleStates,
      drive
    );
  }

  private static Command driveDirectlyTo(DriveSubsystem drive, Pose2d pose) {
    return
      movementHelper(drive, List.of(pose));
  }

  private static Command speaker(ArmSubsystem arm, ShoulderSubsystem shoulder, double aimPosition) {
    return
      new InstantCommand(() -> shoulder.setPivotPosition(aimPosition), shoulder)
      .alongWith(
      new WaitCommand(2.0))
      
      .andThen(

      new InstantCommand(() -> arm.setLaunchSpeed(1.0), arm)
      .alongWith(
      new WaitCommand(2.0)))
      
      .andThen(

      new InstantCommand(() -> arm.setIntakeSpeed(1.0), arm)
      .alongWith(
      new WaitCommand(2.0)));
  }

  private static Command loadFromGround(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      new InstantCommand(() -> shoulder.setPivotPosition(0.0), shoulder)
      .alongWith(
      new WaitCommand(2.0))

      .andThen(

      new InstantCommand(() -> arm.setIntakeSpeed(0.7), arm)
      .alongWith(
      new WaitCommand(1.0)))

      .andThen(

      new InstantCommand(() -> arm.setIntakeSpeed(0.0), arm));
  }

  /********************************/

  public static Command Leave(DriveSubsystem drive) {
    return
      driveDirectlyTo(drive, new Pose2d(2.0, 0.0, new Rotation2d(0.0)));
  }

  public static Command SpeakerLoadLeave(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      new HomeCommand(shoulder, ShoulderSubsystem.Extremum.kUpper)
      .alongWith(
      new WaitCommand(2.0))

      .andThen(

      speaker(arm, shoulder, 0.5))

      .andThen(

      driveDirectlyTo(drive, new Pose2d(0.5, 0.0, new Rotation2d(0.0))))

      .andThen(

      loadFromGround(drive, arm, shoulder))

      .andThen(

      new InstantCommand(() -> shoulder.setPivotPosition(1.0), shoulder)
      .alongWith(
      new InstantCommand(() ->
        { arm.setLaunchSpeed(0.0);
          arm.setIntakeSpeed(0.0); }, arm)))

      .andThen(

      driveDirectlyTo(drive, new Pose2d(2.0, 0.0, new Rotation2d(0.0))));
  }
}
