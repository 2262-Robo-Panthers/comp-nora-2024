// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HomeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.lib.MAXSwerve.MAXSwerveModule;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ShuffleboardConstants.CardMetadata;

import static frc.robot.Constants.DriveConstants.ModuleId.*;

public class RobotContainer {
  private final ShuffleboardTab m_dashboard = Shuffleboard.getTab("2024");

  private final CommandXboxController m_driverController = new CommandXboxController(
    OIConstants.USB.kDriverControllerPort
  );

  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(
    m_dashboard,
    DriveConstants.kMaxSpeedLin_m_s,
    DriveConstants.kMaxSpeedAng_rad_s,
    DriveConstants.kIsGyroReversed,
    DriveConstants.kSlewRateMovement,
    DriveConstants.kSlewRateDirection,
    DriveConstants.kSlewRateRotation,
    DriveConstants.kDriveKinematics,
    new MAXSwerveModule(
      DriveConstants.CAN.kMotorPort(Fr, Rt, Dv),
      DriveConstants.CAN.kMotorPort(Fr, Rt, Tn),
      DriveConstants.kSwerveModuleAngularOffset(Fr, Rt)
    ),
    new MAXSwerveModule(
      DriveConstants.CAN.kMotorPort(Fr, Lf, Dv),
      DriveConstants.CAN.kMotorPort(Fr, Lf, Tn),
      DriveConstants.kSwerveModuleAngularOffset(Fr, Lf)
    ),
    new MAXSwerveModule(
      DriveConstants.CAN.kMotorPort(Bk, Rt, Dv),
      DriveConstants.CAN.kMotorPort(Bk, Rt, Tn),
      DriveConstants.kSwerveModuleAngularOffset(Bk, Rt)
    ),
    new MAXSwerveModule(
      DriveConstants.CAN.kMotorPort(Bk, Lf, Dv),
      DriveConstants.CAN.kMotorPort(Bk, Lf, Tn),
      DriveConstants.kSwerveModuleAngularOffset(Bk, Lf)
    )
  );

  private final Command m_driveCommand = new DriveCommand(
    m_dashboard,
    m_driveSubsystem,
    m_driverController::getLeftY,
    m_driverController::getLeftX,
    m_driverController::getRightX,
    OIConstants.kDeadband
  );

  private final CommandXboxController m_endEffectorController = new CommandXboxController(
    OIConstants.USB.kEndEffectorControllerPort
  );

  public final ArmSubsystem m_armSubsystem = new ArmSubsystem(
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
    m_dashboard,
    m_armSubsystem,
    m_endEffectorController::getRightY,
    m_endEffectorController::getLeftY,
    OIConstants.kDeadband
  );

  public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(
    m_dashboard,
    PivotConstants.kIsInverted,
    PivotConstants.kRange,
    PivotConstants.kHyperextension,
    PivotConstants.kP,
    PivotConstants.kI,
    PivotConstants.kD,
    new DigitalInput(PivotConstants.DIO.kLimitSwitchLower),
    new DigitalInput(PivotConstants.DIO.kLimitSwitchUpper),
    new TalonFX(PivotConstants.CAN.kMotorPortA),
    new TalonFX(PivotConstants.CAN.kMotorPortB)
  );

  private final ShoulderCommand m_shoulderCommand = new ShoulderCommand(
    m_dashboard,
    m_shoulderSubsystem,
    () ->
      m_endEffectorController.getRightTriggerAxis() -
      m_endEffectorController.getLeftTriggerAxis(),
    PivotConstants.kSensitivity
  );

  private final Orchestra m_orchestra = new Orchestra();

  private final ChirpManager m_sfxManager = new ChirpManager(
    m_dashboard,
    m_shoulderSubsystem,
    m_orchestra,
    ShuffleboardConstants.SfxInfo,
    "game-sounds/start-auto",
    "game-sounds/start-teleop",
    "game-sounds/match-end"
  );

  private final ChirpManager m_musicManager = new ChirpManager(
    m_dashboard,
    m_shoulderSubsystem,
    m_orchestra,
    ShuffleboardConstants.MusicInfo,
    "music/happy-birthday",
    "music/mary-had-a-little-lamb",
    "music/twinkle-twinkle",
    "music/jingle-bells",
    "music/star-spangled-banner"
  );

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>() {{
    setDefaultOption(
      "Leave",
      AutoCommandFactory.Leave(m_driveSubsystem)
    );
    addOption(
      "Preload\u00bbSpeaker, Leave",
      AutoCommandFactory.SpeakerLeave(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem)
    );
    addOption(
      "Preload\u00bbSpeaker, Ground\u00bbIntake, Leave",
      AutoCommandFactory.SpeakerLoadLeave(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem)
    );
  }};

  public RobotContainer() {
    m_driveSubsystem.setReferencePlane(true);

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_armSubsystem.setDefaultCommand(m_armCommand);
    m_shoulderSubsystem.setDefaultCommand(m_shoulderCommand);

    populateDashboard();

    configureBindings();
  }

  private void populateDashboard() {
    CardMetadata metadata = ShuffleboardConstants.AutoChooser;

    m_dashboard
      .add(metadata.name, m_autoChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(metadata.x, metadata.y)
      .withSize(metadata.w, metadata.h);
  }

  private void configureBindings() {
    m_driverController.back()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.setXFormation(true), m_driveSubsystem));
    m_driverController.start()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.setXFormation(false), m_driveSubsystem));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.usePoseTranslationX(0.0), m_driveSubsystem));
    m_driverController.y()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.usePoseTranslationY(0.0), m_driveSubsystem));
    m_driverController.a()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.usePoseRotation(new Rotation2d(0.0)), m_driveSubsystem));
    m_driverController.b()
      .onTrue(new InstantCommand(() -> m_driveSubsystem.usePoseRotation(new Rotation2d(Math.PI)), m_driveSubsystem));

    // TODO test chirps
    m_driverController.leftBumper()
      .onTrue(m_sfxManager.getSongSelectCommand(i -> i + 1));
    m_driverController.back()
      .onTrue(m_sfxManager.getPlayPauseCommand());
    m_driverController.rightBumper()
      .onTrue(m_musicManager.getSongSelectCommand(i -> i + 1));
    m_driverController.start()
      .onTrue(m_musicManager.getPlayPauseCommand());

    m_endEffectorController.povUp()
      .onTrue(new HomeCommand(m_shoulderSubsystem, ShoulderSubsystem.Extremum.kUpper));
    m_endEffectorController.povDown()
      .onTrue(new HomeCommand(m_shoulderSubsystem, ShoulderSubsystem.Extremum.kLower));
    m_endEffectorController.povLeft()
      .onTrue(new InstantCommand(m_shoulderSubsystem::neutralizeMotors, m_shoulderSubsystem)
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    m_endEffectorController.povRight()
      .onTrue(new InstantCommand(m_shoulderSubsystem::deneutralizeMotors, m_shoulderSubsystem));
    m_endEffectorController.leftBumper()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.resetPosition(0.0), m_shoulderSubsystem));
    m_endEffectorController.rightBumper()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.resetPosition(1.0), m_shoulderSubsystem));

    m_endEffectorController.a()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.setPivotPosition(0.0), m_shoulderSubsystem));
    m_endEffectorController.b()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.setPivotPosition(AutoConstants.kSpeakerSideAim), m_shoulderSubsystem));
    m_endEffectorController.x()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.setPivotPosition(AutoConstants.kSpeakerFrontAim), m_shoulderSubsystem));
    m_endEffectorController.y()
      .onTrue(new InstantCommand(() -> m_shoulderSubsystem.setPivotPosition(1.0), m_shoulderSubsystem));

    // m_endEffectorController.back()
    //   .onTrue(m_chirpManager.getSongSelectCommand(i -> i - 1));
    // m_endEffectorController.start()
    //   .onTrue(m_chirpManager.getSongSelectCommand(i -> i + 1));
    // m_endEffectorController.y()
    //   .onTrue(m_chirpManager.getPlayPauseCommand());

    // m_endEffectorController.x()
    //   .onTrue(m_sfxManager.getSongPlayCommand(__ -> 0));
    // m_endEffectorController.a()
    //   .onTrue(m_sfxManager.getSongPlayCommand(__ -> 1));
    // m_endEffectorController.b()
    //   .onTrue(m_sfxManager.getSongPlayCommand(__ -> 2));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
