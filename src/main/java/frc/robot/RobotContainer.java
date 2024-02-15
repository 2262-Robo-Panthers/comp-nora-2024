// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.Constants.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(
    OperatorConstants.USB.kDriverControllerPort
  );

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(
    new SmartMotorControllerGroup<>(
      IntakeConstants.kIsInverted,
      IntakeConstants.kMaxSpeed,
      (master, follower) -> follower.follow(master),
      new CANSparkMax(IntakeConstants.CAN.kMotorPortA, MotorType.kBrushless),
      new CANSparkMax(IntakeConstants.CAN.kMotorPortB, MotorType.kBrushless)
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
    m_driverController::getRightY,
    m_driverController::getLeftY
  );

  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(m_armCommand);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
