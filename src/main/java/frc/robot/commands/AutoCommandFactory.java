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
    return Mobility(drive, AutoConstants.kPoseSpeakerLeave);
  }

  public static Command Mobility(DriveSubsystem drive, Pose2d position) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        position
      )));
  }

  public static Command MobilitySpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return MobilitySpeaker(drive, arm, shoulder, AutoConstants.kPoseSpeakerLeave);
  }

  public static Command MobilitySpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder, Pose2d position) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      shoulder.controlCommand(AutoConstants.kAimSpeakerFront)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      arm.controlCommand(null, 1.0)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      arm.controlCommand(1.0, null)
      .alongWith(
      Commands.waitSeconds(2.0)))

      .andThen(

      arm.controlCommand(0.0, -0.01)
      .alongWith(
      Commands.waitSeconds(7.5))) // TEMPORARY!!

      .andThen(

      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        position)))

      .andThen(
      arm.controlCommand(null, 0.0));
  }

  public static Command MobilitySpeakerSpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      shoulder.controlCommand(AutoConstants.kAimSpeakerFront)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, 1.0)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      arm.controlCommand(1.0, null)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, -0.01)
      .alongWith(
      shoulder.controlCommand(AutoConstants.kAimGround))
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        AutoConstants.kPoseSpeakerLeave)))
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      arm.controlCommand(-0.25, null)
      .alongWith(
      shoulder.controlCommand(AutoConstants.kAimSpeakerNote))
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, 1.0)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      arm.controlCommand(1.0, null)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(0.0, -0.1)
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(

      arm.controlCommand(null, 0.0)
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeakerLeave,
        AutoConstants.kPoseSpeakerNote1)))
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(
      shoulder.controlCommand(AutoConstants.kAimGround));
  }

  public static Command MobilitySpeakerSpeaker(DriveSubsystem drive, ArmSubsystem arm, ShoulderSubsystem shoulder) {
    return
      Commands.runOnce(drive::useCurrentPoseAsOrigin, drive)

      .andThen(

      shoulder.controlCommand(AutoConstants.kAimSpeakerFront)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, 1.0)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      arm.controlCommand(1.0, null)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, -0.01))

      .andThen(

      arm.intakeCommand()
      .alongWith(
      shoulder.controlCommand(AutoConstants.kAimGround))
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeaker,
        AutoConstants.kPoseSpeakerLeave)))
      .alongWith(
      Commands.waitSeconds(1.5))
      .raceWith(
      Commands.waitSeconds(3.0))) // in the event that IntakeCommand fails

      .andThen(

      arm.controlCommand(0.0, 0.0)
      .alongWith(
      shoulder.controlCommand(AutoConstants.kAimSpeakerNote))
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(null, 1.0)
      .alongWith(
      Commands.waitSeconds(1.5)))

      .andThen(

      arm.controlCommand(1.0, null)
      .alongWith(
      Commands.waitSeconds(1.0)))

      .andThen(

      arm.controlCommand(0.0, -0.1)
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(

      arm.controlCommand(null, 0.0)
      .alongWith(
      movementHelper(drive, List.of(
        AutoConstants.kPoseSpeakerLeave,
        AutoConstants.kPoseSpeakerNote1)))
      .alongWith(
      Commands.waitSeconds(0.5)))

      .andThen(
      shoulder.controlCommand(AutoConstants.kAimGround));
  }
}
