// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;
import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public class ShuffleboardTabWithMaps {
  public static <T> ShuffleboardLayout addMap(ShuffleboardTab tab, String name, String fmt, Map<String, Supplier<T>> data) {
    return addMap(tab, name,
      data
        .entrySet()
        .stream()
        .collect(
          () -> new HashMap<String, Pair<String, Supplier<T>>>(),
          (m, e) -> m.put(e.getKey(), new Pair<>(fmt, e.getValue())),
          Map::putAll
        )
    );
  }

  public static <T> ShuffleboardLayout addMap(ShuffleboardTab tab, String name, Map<String, Pair<String, Supplier<T>>> data) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach((k, v) -> {
      list.addString(k, () -> String.format(v.getFirst(), v.getSecond().get()));
    });
 
    return list;
  }

  public static <T, V> ShuffleboardLayout addMap(ShuffleboardTab tab, String name, Supplier<T> source, String fmt, Map<String, Function<T, V>> data) {
    return addMap(tab, name, source,
      data
        .entrySet()
        .stream()
        .collect(
          () -> new HashMap<String, Pair<String, Function<T, V>>>(),
          (m, e) -> m.put(e.getKey(), new Pair<>(fmt, e.getValue())),
          Map::putAll
        )
    );
  }

  public static <T, V> ShuffleboardLayout addMap(ShuffleboardTab tab, String name, Supplier<T> source, Map<String, Pair<String, Function<T, V>>> data) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach((k, v) -> {
      list.addString(k, FunctionalUtil.supplyThenProcess(source, x -> String.format(v.getFirst(), v.getSecond().apply(x))));
    });
 
    return list;
  }

  public static ShuffleboardLayout addMap(ShuffleboardTab tab, String name, Map<String, Supplier<Boolean>> data, boolean __) {
    ShuffleboardLayout list = tab.getLayout(name, BuiltInLayouts.kList);

    data.forEach((k, v) -> {
      list.addBoolean(k, (BooleanSupplier) v);
    });

    return list;
  }
}
