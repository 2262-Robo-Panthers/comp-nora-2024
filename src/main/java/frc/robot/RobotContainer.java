// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.commands.LaunchCommand;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.lib.SmartMotorController.SmartMotorController;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.Constants.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(
    OperatorConstants.USB.kDriverControllerPort
  );

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(
    new SmartMotorController(
      IntakeConstants.kIsInverted,
      IntakeConstants.kMaxSpeed,
      new Spark(IntakeConstants.PWM.kMotorPort)
    ),
    new SmartMotorControllerGroup<>(
      LaunchConstants.kIsInverted,
      LaunchConstants.kMaxSpeed,
      (master, follower) -> follower.follow(master),
      new CANSparkMax(LaunchConstants.CAN.kMotorPortA, MotorType.kBrushless),
      new CANSparkMax(LaunchConstants.CAN.kMotorPortB, MotorType.kBrushless)
    )
  );

  private final LaunchCommand m_intakeCommand = new LaunchCommand(
    m_armSubsystem,
    m_driverController::getLeftY
  );

  private final LaunchCommand m_launchCommand = new LaunchCommand(
    m_armSubsystem,
    m_driverController::getRightY
  );

  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(
      Commands.parallel(
        m_intakeCommand,
        m_launchCommand
      )
    );

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
