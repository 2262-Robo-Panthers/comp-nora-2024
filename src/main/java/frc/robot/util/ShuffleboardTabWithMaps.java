// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public class ShuffleboardTabWithMaps {
  public static <T> ShuffleboardLayout addMap(
    ShuffleboardTab tab, String name,
    String fmt, List<Pair<String, Supplier<T>>> data
  ) {
    return addMap(tab, name,
      data
        .stream()
        .map(x -> new Pair<>(x.getFirst(), new Pair<>(fmt, x.getSecond())))
        .toList()
    );
  }

  public static <T> ShuffleboardLayout addMap(
    ShuffleboardTab tab, String name,
    List<Pair<String, Pair<String, Supplier<T>>>> data
  ) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addString(x.getFirst(),
        () -> String.format(
          x.getSecond().getFirst(),
          x.getSecond().getSecond().get()
        ));
    });
 
    return list.withProperties(Map.of("Label position", "TOP"));
  }

  public static <T, V> ShuffleboardLayout addMap(
    ShuffleboardTab tab, String name, Supplier<T> source,
    String fmt, List<Pair<String, Function<T, V>>> data
  ) {
    return addMap(tab, name, source,
      data
        .stream()
        .map(x -> new Pair<>(x.getFirst(), new Pair<>(fmt, x.getSecond())))
        .toList()
    );
  }

  public static <T, V> ShuffleboardLayout addMap(
    ShuffleboardTab tab, String name, Supplier<T> source,
    List<Pair<String, Pair<String, Function<T, V>>>> data
  ) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addString(x.getFirst(),
        FunctionalUtil.supplyThenProcess(source, y -> String.format(
          x.getSecond().getFirst(),
          x.getSecond().getSecond().apply(y)
        )));
    });
 
    return list.withProperties(Map.of("Label position", "TOP"));
  }

  public static ShuffleboardLayout addMap(
    ShuffleboardTab tab, String name,
    List<Pair<String, Supplier<Boolean>>> data, boolean __) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addBoolean(x.getFirst(), x.getSecond()::get);
    });

    return list.withProperties(Map.of("Label position", "TOP"));
  }
}
