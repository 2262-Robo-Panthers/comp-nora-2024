// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.LaunchCommand;
import frc.robot.subsystems.LaunchSubsystem;

import frc.robot.lib.SmartMotorController.SmartMotorController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final LaunchSubsystem m_launchSubsystem = new LaunchSubsystem(
    new SmartMotorController(
      true, 0.8, 0.5,
      new Spark(0)
    )
  );

  private final LaunchCommand m_launchCommand = new LaunchCommand(
    m_launchSubsystem,
    m_driverController::getLeftY
  );

  public RobotContainer() {
    m_launchSubsystem.setDefaultCommand(m_launchCommand);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
