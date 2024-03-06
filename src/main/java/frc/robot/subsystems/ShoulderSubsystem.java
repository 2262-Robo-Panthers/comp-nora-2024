// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.ShuffleboardTabWithMaps;

public class ShoulderSubsystem extends SubsystemBase {
  private final double m_totalRange;
  private final double m_hyperextension;
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  private double m_positionNow;
  private double m_positionZero;

  private final DigitalInput m_limitSwitchLower;
  private final DigitalInput m_limitSwitchUpper;

  private TalonFX[] m_talons;
  private TalonFX m_master;

  public ShoulderSubsystem(
    ShuffleboardTab shuffleboardTab,
    boolean isInverted, double totalRange, double hyperextension,
    double p, double i, double d,
    DigitalInput limitSwitchLower, DigitalInput limitSwitchUpper,
    TalonFX... talons
  ) {
    m_totalRange = totalRange;
    m_hyperextension = hyperextension;

    m_limitSwitchLower = limitSwitchLower;
    m_limitSwitchUpper = limitSwitchUpper;
    m_talons = talons;

    for (TalonFX controller : talons) {
      controller.setInverted(isInverted);
    }

    setupPid(p, i, d);
    setupLimits();

    populateDashboard(shuffleboardTab);
  }

  private void setupPid(double p, double i, double d) {
    Slot0Configs config = new Slot0Configs();
    config.kP = p;
    config.kI = i;
    config.kD = d;

    for (TalonFX controller : m_talons) {
      controller.getConfigurator().apply(config);
    }

    resetPosition(0.5);
  }

  private void setupLimits() {
    new Trigger(m_limitSwitchLower::get)
      .onFalse(new InstantCommand(() -> {
        resetPosition(0.0);
        setPivotPosition(0.0);
      }, this));

    new Trigger(m_limitSwitchUpper::get)
      .onFalse(new InstantCommand(() -> {
        resetPosition(1.0);
        setPivotPosition(1.0);
      }, this));
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, "Pivot", List.of(
      new Pair<>("Hit Lower", m_limitSwitchLower::get),
      new Pair<>("Hit Upper", m_limitSwitchUpper::get)
    ), false)
      .withPosition(4, 2)
      .withSize(2, 3)
      .addDouble("Position Requested", () -> m_positionNow * m_totalRange)
      .getParent()
      .addDouble("Position Reported", () -> m_master.getPosition().asSupplier().get() - m_positionZero);
  }

  public void movePivotPosition(double positionDelta) {
    setPivotPosition(MathUtil.clamp(
      m_positionNow + positionDelta * 0.01,
      0.0 - m_hyperextension,
      1.0 + m_hyperextension
    ));
  }

  public void setPivotPosition(double position) {
    m_positionNow = position;
    m_master.setControl(m_request.withPosition(m_positionZero + position * m_totalRange));
  }

  public void resetPosition(double position) {
    m_positionZero = m_master.getPosition().getValue() - position * m_totalRange;
  }

  public TalonFX[] getControllers() {
    return m_talons;
  }
}
