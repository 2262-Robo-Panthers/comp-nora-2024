// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;

public class FormatUtil {
  public static <V> String[] format(List<Pair<String, V>> values) {
    return values
      .stream()
      .map(x -> String.format("%s: %.3f", x.getFirst(), x.getSecond()))
      .toArray(n -> new String[n]);
  }

  public static <V> Supplier<String[]> formatted(List<Pair<String, Supplier<V>>> values) {
    return () -> format(values
      .stream()
      .map(x -> new Pair<>(x.getFirst(), x.getSecond().get()))
      .toList());
  }

  public static <T, V> Supplier<String[]> formatted(Supplier<T> source, Function<T, List<Pair<String, V>>> values) {
    return () -> format(values.apply(source.get()));
  }
}
