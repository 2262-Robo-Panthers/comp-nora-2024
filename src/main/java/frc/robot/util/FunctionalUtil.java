// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;

import com.revrobotics.CANSparkMax;

import frc.robot.lib.SmartMotorController.SmartMotorControllerFollower;

public class FunctionalUtil {
  public static <T> Supplier<T> supplyThenOperate(Supplier<T> supplier, UnaryOperator<T> operator) {
    return () -> operator.apply(supplier.get());
  }

  public static <T, U> Supplier<U> supplyThenProcess(Supplier<T> supplier, Function<T, U> processor) {
    return () -> processor.apply(supplier.get());
  }

  public static UnaryOperator<Double> deadbandify(double deadband) {
    return i -> 0 - MathUtil.applyDeadband(i, deadband);
  }

  public static SmartMotorControllerFollower<CANSparkMax> followifierSparkMax =
    (master, follower) -> follower.follow(master);
  public static SmartMotorControllerFollower<TalonFX> followifierTalonFx =
    (master, follower) -> follower.setControl(new Follower(master.getDeviceID(), false));
}
