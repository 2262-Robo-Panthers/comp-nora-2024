// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


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

  /********************************/

  public static Command Mobility(DriveSubsystem drive) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        AutoConstants.kPoseSpeakerLeave
      )));
  }

  public static Command MobilitySpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      new HomeCommand(shoulder, ShoulderSubsystem.Extremum.kLower))

      .andThen(

      Commands.runOnce(() -> shoulder.setPivotPosition(AutoConstants.kAimSpeakerFront), shoulder)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      Commands.runOnce(() -> arm.setLaunchSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      Commands.runOnce(() -> arm.setIntakeSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      Commands.runOnce(() ->
        { arm.setLaunchSpeed(-0.01);
          arm.setIntakeSpeed(0.0); }, arm)
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        AutoConstants.kPoseSpeakerLeave))))

      .andThen(
      Commands.runOnce(() -> arm.setLaunchSpeed(0.0), arm));
  }

  public static Command MobilitySpeakerSpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      new HomeCommand(shoulder, ShoulderSubsystem.Extremum.kLower))

      .andThen(

      Commands.runOnce(() -> shoulder.setPivotPosition(AutoConstants.kAimSpeakerFront), shoulder)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      Commands.runOnce(() -> arm.setLaunchSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      Commands.runOnce(() -> arm.setIntakeSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      Commands.runOnce(() -> arm.setLaunchSpeed(-0.01), arm)
      .alongWith(
      Commands.runOnce(() -> shoulder.setPivotPosition(-0.12), shoulder))
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        AutoConstants.kPoseSpeakerLeave)))
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      Commands.runOnce(() -> arm.setIntakeSpeed(-0.25), arm)
      .alongWith(
      Commands.runOnce(() -> shoulder.setPivotPosition(AutoConstants.kAimSpeakerNote), shoulder))
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      Commands.runOnce(() -> arm.setLaunchSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      Commands.runOnce(() -> arm.setIntakeSpeed(1.0), arm)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      Commands.runOnce(() ->
        { arm.setLaunchSpeed(-0.1);
          arm.setIntakeSpeed(0.0); }, arm)
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(

      Commands.runOnce(() -> arm.setLaunchSpeed(0.0), arm)
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeakerLeave,
        AutoConstants.kPoseSpeakerNote1)))
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(
      Commands.runOnce(() -> shoulder.setPivotPosition(-0.09), shoulder));
  }
}
