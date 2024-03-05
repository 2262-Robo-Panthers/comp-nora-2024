// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.lib.SmartMotorController.SmartMotorController;
import frc.robot.lib.SmartMotorController.SmartMotorControllerGroup;
import frc.robot.util.ShuffleboardTabWithMaps;

public class ArmSubsystem extends SubsystemBase {
  private final SmartMotorController m_pivot;
  private final SmartMotorController m_intake;
  private final SmartMotorController m_launch;

  private final double m_pivotDistance;
  private final DigitalInput m_pivotLimitLower;
  private final DigitalInput m_pivotLimitUpper;
  private final PositionVoltage m_pivotRequest = new PositionVoltage(0).withSlot(0);
  private double m_pivotPosition;
  private TalonFX[] m_pivotControllers;
  private TalonFX m_pivotMaster;

  public ArmSubsystem(
    ShuffleboardTab shuffleboardTab,
    double pivotDistance,
    double pivotKP, double pivotKI, double pivotKD,
    DigitalInput pivotLimitLower, DigitalInput pivotLimitUpper,
    SmartMotorController pivot,
    SmartMotorController intake,
    SmartMotorController launch
  ) {
    m_pivot = pivot;
    m_pivotDistance = pivotDistance;
    m_pivotLimitLower = pivotLimitLower;
    m_pivotLimitUpper = pivotLimitUpper;
    m_intake = intake;
    m_launch = launch;

    setupPivotPid(pivotKP, pivotKI, pivotKD);
    setupPivotLimits();

    populateDashboard(shuffleboardTab);
  }

  @SuppressWarnings("unchecked")
  private void setupPivotPid(double p, double i, double d) {
    m_pivotControllers = (TalonFX[]) ((SmartMotorControllerGroup<TalonFX>) m_pivot).getControllers();
    m_pivotMaster = m_pivotControllers[0];

    Slot0Configs config = new Slot0Configs();
    config.kP = p;
    config.kI = i;
    config.kD = d;

    for (TalonFX controller : m_pivotControllers) {
      controller.getConfigurator().apply(config);
    }
  }

  private void setupPivotLimits() {
    new Trigger(m_pivotLimitLower::get)
      .onTrue(new InstantCommand(() -> {
        resetPosition(0.0);
        setPivotPosition(0.0);
      }, this));

    new Trigger(m_pivotLimitUpper::get)
      .onTrue(new InstantCommand(() -> {
        resetPosition(1.0);
        setPivotPosition(1.0);
      }, this));
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, "Pivot", List.of(
      new Pair<>("Hit Lower", m_pivotLimitLower::get),
      new Pair<>("Hit Upper", m_pivotLimitUpper::get)
    ), false)
      .withPosition(4, 2)
      .withSize(2, 2)
      .addDouble(
        "Position", m_pivotMaster.getPosition().asSupplier()::get
      );
  }

  public void movePivotPosition(double positionDelta) {
    m_pivotPosition = MathUtil.clamp(m_pivotPosition + positionDelta, 0.0, 1.0);
    setPivotPosition(m_pivotPosition);
  }

  public void setPivotPosition(double position) {
    m_pivotMaster.setControl(m_pivotRequest.withPosition(position * m_pivotDistance));
  }

  public void setIntakeSpeed(double velocity) {
    m_intake.forceTo(velocity);
  }

  public void setLaunchSpeed(double velocity) {
    m_launch.forceTo(velocity);
  }

  public void stop() {
    m_intake.forceTo(0.0);
    m_launch.forceTo(0.0);
  }

  public void resetPosition(double position) {
    for (TalonFX controller : m_pivotControllers) {
      controller.setPosition(position * m_pivotDistance);
    }
  }

  public SmartMotorController getPivot() {
    return m_pivot;
  }

  public SmartMotorController getIntake() {
    return m_intake;
  }

  public SmartMotorController getLaunch() {
    return m_launch;
  }
}
