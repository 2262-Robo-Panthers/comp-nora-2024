// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private Orchestra m_orchestra;

  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(m_armCommand);

    configureBindings();

    setupOrchestra();
  }

  private void configureBindings() {
    m_driverController.start().onTrue(new InstantCommand(this::queueMusic, m_armSubsystem));
  }

  private void setupOrchestra() {
    m_orchestra = new Orchestra("happy-birthday.chrp");

    for (MotorController controller : ((SmartMotorControllerGroup<TalonFX>) m_armSubsystem.getPivot()).getControllers()) {
      m_orchestra.addInstrument((TalonFX) controller);
    }
  }

  public void queueMusic() {
    if (!m_orchestra.isPlaying()) {
      m_orchestra.play();
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
