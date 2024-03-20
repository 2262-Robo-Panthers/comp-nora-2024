// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.WPIUtilJNI;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.ShuffleboardTabWithMaps;
import frc.robot.Constants.ShuffleboardConstants;

public class ShoulderSubsystem extends SubsystemBase {
  private final double m_positionLower;
  private final double m_positionUpper;

  private Pair<Double, Double> m_referenceA;
  private Pair<Double, Double> m_referenceB;

  private final DutyCycleEncoder m_encoder;

  private boolean m_isNeutralized = false;

  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private final TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(1.0, 0.0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(1.0, 0.0);

  private final TalonFX[] m_talons;
  private final TalonFX m_master;

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public enum Extremum {
    kLower,
    kUpper;
  };

  public ShoulderSubsystem(
    ShuffleboardTab shuffleboardTab,
    boolean isInverted,
    double positionLower, double positionUpper,
    double p, double i, double d,
    double maxSpeed, double maxAccel,
    DutyCycleEncoder encoder,
    TalonFX... talons
  ) {
    m_positionLower = positionLower;
    m_positionUpper = positionUpper;

    m_encoder = encoder;

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

    // TODO magic numbers
    m_referenceA = getPositionPair();
    m_referenceB = new Pair<>(m_referenceA.getFirst() + 180, m_referenceA.getSecond() - 4);

    setupPid(p, i, d);
    populateDashboard(shuffleboardTab);

    // TEMP
    neutralizeMotors();
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
      new Pair<>("Input", () -> m_goal.position),
      new Pair<>("Request", () -> m_setpoint.position),
      new Pair<>("Lowest", () -> absoluteToRelative(m_positionLower)),
      new Pair<>("Uppest", () -> absoluteToRelative(m_positionUpper)),
      new Pair<>("Absolute", this::getAbsolutePosition),
      new Pair<>("Relative", this::getRelativePosition),
      new Pair<>("Mtr 0 Temp", m_talons[0].getDeviceTemp().asSupplier()::get),
      new Pair<>("Mtr 1 Temp", m_talons[1].getDeviceTemp().asSupplier()::get)
    ));
  }

  @Override
  public void periodic() {
    if (m_isNeutralized)
      return;

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    m_prevTime = currentTime;
    m_setpoint = m_profile.calculate(elapsedTime, m_setpoint, m_goal);

    updateReferenceB();

    m_master.setControl(m_request.withPosition(inputToRelative(m_setpoint.position)));
  }

  private void updateReferenceB() {
    double differenceNew = m_referenceA.getFirst() - getAbsolutePosition();
    double differenceOld = m_referenceA.getFirst() - m_referenceB.getFirst();

    if (Math.abs(differenceNew) > Math.abs(differenceOld))
      m_referenceB = getPositionPair();
  }

  public void movePivotPosition(double positionDelta) {
    setPivotPosition(MathUtil.clamp(
      m_goal.position + positionDelta,
      0.0, 1.0
    ));
  }

  public void setPivotPosition(double position) {
    m_goal.position = position;
    m_goal.velocity = 0.0;
  }

  private double getAbsolutePosition() {
    return m_encoder.getAbsolutePosition();
  }

  private double getRelativePosition() {
    return m_master.getPosition().getValue();
  }

  private Pair<Double, Double> getPositionPair() {
    return new Pair<>(getAbsolutePosition(), getRelativePosition());
  }

  private double inputToAbsolute(double input) {
    return m_positionLower + input * (m_positionUpper - m_positionLower);
  }

  private double inputToRelative(double input) {
    return absoluteToRelative(inputToAbsolute(input));
  }

  private double absoluteToRelative(double absolute) {
    return absolute
      - m_referenceA.getFirst()
      / (m_referenceB.getFirst() - m_referenceA.getFirst())
      * (m_referenceB.getSecond() - m_referenceA.getSecond())
      + m_referenceA.getSecond();
  }

  public void neutralizeMotors() {
    m_isNeutralized = true;
    m_master.setControl(new NeutralOut());
  }

  public void deneutralizeMotors() {
    m_isNeutralized = false;
  }

  public TalonFX[] getControllers() {
    return m_talons;
  }

  public Command controlCommand(double position) {
    return runOnce(() -> setPivotPosition(position));
  }
}
