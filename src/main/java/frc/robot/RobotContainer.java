// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.lib.MAXSwerve.MAXSwerveModule;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.ModuleId.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(
    OIConstants.USB.kDriverControllerPort
  );

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(
    new MAXSwerveModule(
      Constants.DriveConstants.CAN.kMotorPort(Fr, Rt, Dv),
      Constants.DriveConstants.CAN.kMotorPort(Fr, Rt, Tn),
      Constants.DriveConstants.kSwerveModuleAngularOffset(Fr, Rt)
    ),
    new MAXSwerveModule(
      Constants.DriveConstants.CAN.kMotorPort(Fr, Lf, Dv),
      Constants.DriveConstants.CAN.kMotorPort(Fr, Lf, Tn),
      Constants.DriveConstants.kSwerveModuleAngularOffset(Fr, Lf)
    ),
    new MAXSwerveModule(
      Constants.DriveConstants.CAN.kMotorPort(Bk, Rt, Dv),
      Constants.DriveConstants.CAN.kMotorPort(Bk, Rt, Tn),
      Constants.DriveConstants.kSwerveModuleAngularOffset(Bk, Rt)
    ),
    new MAXSwerveModule(
      Constants.DriveConstants.CAN.kMotorPort(Bk, Lf, Dv),
      Constants.DriveConstants.CAN.kMotorPort(Bk, Lf, Tn),
      Constants.DriveConstants.kSwerveModuleAngularOffset(Bk, Lf)
    )
  );

  private final Command m_driveCommand = new DriveCommand(
    m_driveSubsystem,
    m_driverController::getLeftY,
    m_driverController::getLeftX,
    m_driverController::getRightX,
    OIConstants.kDeadband
  );

  private final CommandXboxController m_endEffectorController = new CommandXboxController(
    OIConstants.USB.kEndEffectorControllerPort
  );

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(
    new SmartMotorControllerGroup<>(
      PivotConstants.kIsInverted,
      PivotConstants.kMaxSpeed,
      (master, follower) -> follower.setControl(new Follower(master.getDeviceID(), false)),
      new TalonFX(PivotConstants.CAN.kMotorPortA),
      new TalonFX(PivotConstants.CAN.kMotorPortB)
    ),
    new SmartMotorControllerGroup<>(
      IntakeConstants.kIsInverted,
      IntakeConstants.kMaxSpeed,
      (master, follower) -> follower.follow(master),
      new CANSparkMax(IntakeConstants.CAN.kMotorPortA, MotorType.kBrushed),
      new CANSparkMax(IntakeConstants.CAN.kMotorPortB, MotorType.kBrushed)
    ),
    new SmartMotorControllerGroup<>(
      LaunchConstants.kIsInverted,
      LaunchConstants.kMaxSpeed,
      (master, follower) -> follower.follow(master),
      new CANSparkMax(LaunchConstants.CAN.kMotorPortA, MotorType.kBrushless),
      new CANSparkMax(LaunchConstants.CAN.kMotorPortB, MotorType.kBrushless)
    )
  );

  private final ArmCommand m_armCommand = new ArmCommand(
    m_armSubsystem,
    () -> m_endEffectorController.getRightTriggerAxis() - m_endEffectorController.getLeftTriggerAxis(),
    m_endEffectorController::getRightY,
    m_endEffectorController::getLeftY
  );

  private final Orchestra m_orchestra = new Orchestra();

  private final ChirpManager m_sfxManager = new ChirpManager(
    m_armSubsystem,
    m_orchestra,
    "game-sounds/start-auto",
    "game-sounds/start-teleop",
    "game-sounds/match-end"
  );

  private final ChirpManager m_chirpManager = new ChirpManager(
    m_armSubsystem,
    m_orchestra,
    "music/happy-birthday",
    "music/mary-had-a-little-lamb",
    "music/twinkle-twinkle",
    "music/jingle-bells",
    "music/star-spangled-banner"
  );

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_armSubsystem.setDefaultCommand(m_armCommand);

    configureBindings();
  }

  private void configureBindings() {
    m_endEffectorController.start().onTrue(m_chirpManager.getSongSelectCommand(i -> i + 1));
    m_endEffectorController.back().onTrue(m_chirpManager.getSongSelectCommand(i -> i - 1));
    m_endEffectorController.y().onTrue(m_chirpManager.getPlayPauseCommand());

    m_endEffectorController.x().onTrue(m_sfxManager.getSongPlayCommand(__ -> 0));
    m_endEffectorController.a().onTrue(m_sfxManager.getSongPlayCommand(__ -> 1));
    m_endEffectorController.b().onTrue(m_sfxManager.getSongPlayCommand(__ -> 2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
