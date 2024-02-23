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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.Constants.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(
    OIConstants.USB.kDriverControllerPort
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
    () -> m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
    m_driverController::getRightY,
    m_driverController::getLeftY
  );

  private final Orchestra m_orchestra = new Orchestra();

  private final ChirpController m_sfxController = new ChirpController(
    m_armSubsystem,
    m_orchestra,
    "game-sounds/start-auto",
    "game-sounds/start-teleop",
    "game-sounds/match-end"
  );

  private final ChirpController m_chirpController = new ChirpController(
    m_armSubsystem,
    m_orchestra,
    "music/happy-birthday",
    "music/mary-had-a-little-lamb",
    "music/twinkle-twinkle",
    "music/jingle-bells",
    "music/star-spangled-banner"
  );

  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(m_armCommand);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.start().onTrue(m_chirpController.getSongSelectCommand(i -> i + 1));
    m_driverController.back().onTrue(m_chirpController.getSongSelectCommand(i -> i - 1));
    m_driverController.y().onTrue(m_chirpController.getPlayPauseCommand());

    m_driverController.x().onTrue(m_sfxController.getSongPlayCommand(__ -> 0));
    m_driverController.a().onTrue(m_sfxController.getSongPlayCommand(__ -> 1));
    m_driverController.b().onTrue(m_sfxController.getSongPlayCommand(__ -> 2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
