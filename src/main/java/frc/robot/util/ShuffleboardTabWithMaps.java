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

import frc.robot.Constants.ShuffleboardConstants.CardMetadata;

public class ShuffleboardTabWithMaps {
  public static <T> ShuffleboardLayout addMap(
    ShuffleboardTab tab, CardMetadata metadata,
    String fmt, List<Pair<String, Supplier<T>>> data
  ) {
    return addMap(tab, metadata,
      data
        .stream()
        .map(x -> new Pair<>(x.getFirst(), new Pair<>(fmt, x.getSecond())))
        .toList()
    );
  }

  public static <T> ShuffleboardLayout addMap(
    ShuffleboardTab tab, CardMetadata metadata,
    List<Pair<String, Pair<String, Supplier<T>>>> data
  ) {
    ShuffleboardLayout list = tab.getLayout(metadata.name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addString(x.getFirst(),
        () -> String.format(
          x.getSecond().getFirst(),
          x.getSecond().getSecond().get()
        ));
    });
 
    return list
      .withPosition(metadata.x, metadata.y)
      .withSize(metadata.w, metadata.h)
      .withProperties(Map.of("Label position", "TOP"));
  }

  public static <T, V> ShuffleboardLayout addMap(
    ShuffleboardTab tab, CardMetadata metadata, Supplier<T> source,
    String fmt, List<Pair<String, Function<T, V>>> data
  ) {
    return addMap(tab, metadata, source,
      data
        .stream()
        .map(x -> new Pair<>(x.getFirst(), new Pair<>(fmt, x.getSecond())))
        .toList()
    );
  }

  public static <T, V> ShuffleboardLayout addMap(
    ShuffleboardTab tab, CardMetadata metadata, Supplier<T> source,
    List<Pair<String, Pair<String, Function<T, V>>>> data
  ) {
    ShuffleboardLayout list = tab.getLayout(metadata.name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addString(x.getFirst(),
        FunctionalUtil.supplyThenProcess(source, y -> String.format(
          x.getSecond().getFirst(),
          x.getSecond().getSecond().apply(y)
        )));
    });
 
    return list
      .withPosition(metadata.x, metadata.y)
      .withSize(metadata.w, metadata.h)
      .withProperties(Map.of("Label position", "TOP"));
  }

  public static ShuffleboardLayout addMap(
    ShuffleboardTab tab, CardMetadata metadata,
    List<Pair<String, Supplier<Boolean>>> data, boolean __) {
    ShuffleboardLayout list = tab.getLayout(metadata.name, BuiltInLayouts.kList);

    data.forEach(x -> {
      list.addBoolean(x.getFirst(), x.getSecond()::get);
    });

    return list
      .withPosition(metadata.x, metadata.y)
      .withSize(metadata.w, metadata.h)
      .withProperties(Map.of("Label position", "TOP"));
  }
}
