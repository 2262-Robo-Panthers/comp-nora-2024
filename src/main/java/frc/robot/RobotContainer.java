// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.DriveCommand;
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
    m_dashboard,
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
    ),
    new DigitalInput(IntakeConstants.DIO.kPhotogatePort)
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
    PivotConstants.kPositionLower,
    PivotConstants.kPositionUpper,
    PivotConstants.kP,
    PivotConstants.kI,
    PivotConstants.kD,
    PivotConstants.kMaxSpeed_m_s,
    PivotConstants.kMaxAccel_m_s_s,
    new DutyCycleEncoder(PivotConstants.DIO.kEncoderPort),
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
      "Mobility (Front)",
      AutoCommandFactory.Mobility(m_driveSubsystem)
    );
    addOption(
      "Mobility (Left)",
      AutoCommandFactory.Mobility(m_driveSubsystem, AutoConstants.kPoseSpeakerLeaveLeft)
    );
    addOption(
      "Mobility (Right)",
      AutoCommandFactory.Mobility(m_driveSubsystem, AutoConstants.kPoseSpeakerLeaveRight)
    );
    addOption(
      "Mobility + 1 Note (Front)",
      AutoCommandFactory.MobilitySpeaker(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem)
    );
    addOption(
      "Mobility + 1 Note (Left)",
      AutoCommandFactory.MobilitySpeaker(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem, AutoConstants.kPoseSpeakerLeaveLeft)
    );
    addOption(
      "Mobility + 1 Note (Right)",
      AutoCommandFactory.MobilitySpeaker(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem, AutoConstants.kPoseSpeakerLeaveRight)
    );
    addOption(
      "Mobility + 2 Notes (Front)",
      AutoCommandFactory.MobilitySpeakerSpeaker(m_driveSubsystem, m_armSubsystem, m_shoulderSubsystem)
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
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.setXFormation(true), m_driveSubsystem));
    m_driverController.start()
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.setXFormation(false), m_driveSubsystem));

    m_driverController.x()
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.usePoseTranslationX(0.0), m_driveSubsystem));
    m_driverController.y()
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.usePoseTranslationY(0.0), m_driveSubsystem));
    m_driverController.a()
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.usePoseRotation(new Rotation2d(0.0)), m_driveSubsystem));
    m_driverController.b()
      .onTrue(Commands.runOnce(() -> m_driveSubsystem.usePoseRotation(new Rotation2d(Math.PI)), m_driveSubsystem));

    m_driverController.povLeft()
      .onTrue(m_sfxManager.getSongSelectCommand(i -> i + 1));
    m_driverController.povUp()
      .onTrue(m_sfxManager.getPlayPauseCommand());
    m_driverController.povRight()
      .onTrue(m_musicManager.getSongSelectCommand(i -> i + 1));
    m_driverController.povDown()
      .onTrue(m_musicManager.getPlayPauseCommand());

    m_endEffectorController.povUp()
      .onTrue(m_armSubsystem.intakeCommand()
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    m_endEffectorController.povLeft()
      .onTrue(Commands.runOnce(m_shoulderSubsystem::neutralizeMotors, m_shoulderSubsystem)
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    m_endEffectorController.povRight()
      .onTrue(Commands.runOnce(m_shoulderSubsystem::deneutralizeMotors, m_shoulderSubsystem));

    m_endEffectorController.a()
      .onTrue(Commands.runOnce(() -> m_shoulderSubsystem.setPivotPosition(0.0), m_shoulderSubsystem));
    m_endEffectorController.b()
      .onTrue(Commands.runOnce(() -> m_shoulderSubsystem.setPivotPosition(AutoConstants.kAimSpeakerSide), m_shoulderSubsystem));
    m_endEffectorController.x()
      .onTrue(Commands.runOnce(() -> m_shoulderSubsystem.setPivotPosition(AutoConstants.kAimSpeakerFront), m_shoulderSubsystem));
    m_endEffectorController.y()
      .onTrue(Commands.runOnce(() -> m_shoulderSubsystem.setPivotPosition(1.0), m_shoulderSubsystem));

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
