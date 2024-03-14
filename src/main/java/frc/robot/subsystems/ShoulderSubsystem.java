// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.util.WPIUtilJNI;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.commands.HomeCommand;
import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class ShoulderSubsystem extends SubsystemBase {
  private final double m_totalRange;
  private final double m_hyperextension;

  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private final TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(1.0, 0.0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(1.0, 0.0);

  private double m_positionZero;
  private boolean m_isNeutralized = false;

  private boolean m_shouldCheckLimits = false;

  private final DigitalInput m_limitSwitchLower;
  private final DigitalInput m_limitSwitchUpper;

  private TalonFX[] m_talons;
  private TalonFX m_master;

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public enum Extremum {
    kLower,
    kUpper;
  };

  public ShoulderSubsystem(
    ShuffleboardTab shuffleboardTab,
    boolean isInverted,
    double totalRange, double hyperextension,
    double p, double i, double d,
    double maxSpeed, double maxAccel,
    DigitalInput limitSwitchLower, DigitalInput limitSwitchUpper,
    TalonFX... talons
  ) {
    m_totalRange = totalRange;
    m_hyperextension = hyperextension;

    m_limitSwitchLower = limitSwitchLower;
    m_limitSwitchUpper = limitSwitchUpper;
    m_talons = talons;
    m_master = talons[0];

    Follower follower = new Follower(m_master.getDeviceID(), false);

    for (TalonFX controller : talons) {
      controller.setInverted(isInverted);

      if (controller != m_master) {
        controller.setControl(follower);
      }
    }

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxSpeed, maxAccel));

    setupPid(p, i, d);
    setupLimitChecks();
    resetPosition(1.0);

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
  }

  private void populateDashboard(ShuffleboardTab dashboard) {
    ShuffleboardTabWithMaps.addMap(dashboard, ShuffleboardConstants.ShoulderInfo, "%.3f", List.of(
      new Pair<>("Input", () -> m_goal.position * m_totalRange),
      new Pair<>("Request", () -> m_setpoint.position * m_totalRange),
      new Pair<>("Reported", () -> m_master.getPosition().getValue() - m_positionZero),
      new Pair<>("Zero", () -> m_positionZero)
    ))
      .addDouble("Mtr 0 Temp", m_talons[0].getDeviceTemp().asSupplier()::get).getParent()
      .addDouble("Mtr 1 Temp", m_talons[1].getDeviceTemp().asSupplier()::get).getParent()
      .addString("Too Far?", () -> String.join(" ",
        isAtLimitLower() ? "LWR" : "",
        isAtLimitUpper() ? "UPR" : ""));
  }

  @Override
  public void periodic() {
    if (!m_isNeutralized) {
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      m_prevTime = currentTime;

      m_setpoint = m_profile.calculate(elapsedTime, m_setpoint, m_goal);
      m_master.setControl(m_request.withPosition(m_positionZero + m_setpoint.position * m_totalRange));
    }
  }

  private void setupLimitChecks() {
    new Trigger(() -> m_limitSwitchLower.get() && m_shouldCheckLimits)
      .onFalse(Commands.runOnce(() -> {
        resetPosition(0.0);
      }, this));

    new Trigger(() -> m_limitSwitchUpper.get() && m_shouldCheckLimits)
      .onFalse(Commands.runOnce(() -> {
        resetPosition(1.0);
      }, this));
  }

  public void enableLimitChecks() {
    m_shouldCheckLimits = true;
  }

  public void disableLimitChecks() {
    m_shouldCheckLimits = false;
  }

  public void movePivotPosition(double positionDelta) {
    setPivotPosition(MathUtil.clamp(
      m_goal.position + positionDelta,
      0.0 - m_hyperextension,
      1.0 + m_hyperextension
    ));
  }

  public void setPivotPosition(double position) {
    m_goal.position = position;
    m_goal.velocity = 0.0;
  }

  public void resetPosition(double position) {
    m_positionZero = m_master.getPosition().getValue() - position * m_totalRange;
    setPivotPosition(position);
    m_setpoint = m_goal;
  }

  public void neutralizeMotors() {
    m_isNeutralized = true;
    m_master.setControl(new NeutralOut());
  }

  public void deneutralizeMotors() {
    m_isNeutralized = false;
  }

  public boolean isAtLimitLower() {
    return !m_limitSwitchLower.get();
  }

  public boolean isAtLimitUpper() {
    return !m_limitSwitchUpper.get();
  }

  public TalonFX[] getControllers() {
    return m_talons;
  }

  public Command controlCommand(double position) {
    return runOnce(() -> setPivotPosition(position));
  }

  public Command homeCommand(Extremum direction) {
    return new HomeCommand(this, direction);
  }
}
