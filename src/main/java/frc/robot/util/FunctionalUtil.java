// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public class FunctionalUtil {
  public static <T> Supplier<T> supplyThenOperate(Supplier<T> supplier, UnaryOperator<T> operator) {
    return () -> operator.apply(supplier.get());
  }

  public static <T, U> Supplier<U> supplyThenProcess(Supplier<T> supplier, Function<T, U> processor) {
    return () -> processor.apply(supplier.get());
  }
}
