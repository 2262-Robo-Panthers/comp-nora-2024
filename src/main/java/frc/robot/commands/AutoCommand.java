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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class AutoCommand {
  public static Command basicMove(DriveSubsystem drive) {
    return new SwerveControllerCommand(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(),
        new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
        new TrajectoryConfig(1.0, 1.0) {{
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
}
