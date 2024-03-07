// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public class ShuffleboardHelper {
  abstract public static class DataPoint<T> {
    protected String m_name;

    protected DataPoint(String name) {
      m_name = name;
    }

    public static <T> BooleanDataPoint<T> ofBoolean(String name, Function<T, Boolean> data) {
      return new BooleanDataPoint<>(name, data);
    }

    public static <T> BooleanDataPoint<T> ofBoolean(String name, Supplier<Boolean> data) {
      return new BooleanDataPoint<>(name, data);
    }

    public static <T> StringDataPoint<T> ofString(String name, Function<T, String> data) {
      return new StringDataPoint<>(name, data);
    }

    public static <T> StringDataPoint<T> ofString(String name, Supplier<String> data) {
      return new StringDataPoint<>(name, data);
    }

    public static <T, V> FormattedDataPoint<T, V> ofFormatted(String name, String fmt, Function<T, V> data) {
      return new FormattedDataPoint<>(name, fmt, data);
    }

    public static <T, V> FormattedDataPoint<T, V> ofFormatted(String name, String fmt, Supplier<V> data) {
      return new FormattedDataPoint<>(name, fmt, data);
    }

    public static <T> DoubleDataPoint<T> ofDouble(String name, String prec, String units, Function<T, Double> data) {
      return new DoubleDataPoint<>(name, prec, units, data);
    }

    public static <T> DoubleDataPoint<T> ofDouble(String name, String prec, String units, Supplier<Double> data) {
      return new DoubleDataPoint<>(name, prec, units, data);
    }

    abstract public void addTo(ShuffleboardLayout list, Supplier<T> common);

    public void addTo(ShuffleboardLayout list) {
      addTo(list, () -> null);
    }
  }

  private static class BooleanDataPoint<T> extends DataPoint<T> {
    private Function<T, Boolean> m_data;

    public BooleanDataPoint(String name, Function<T, Boolean> data) {
      super(name);
      m_data = data;
    }

    public BooleanDataPoint(String name, Supplier<Boolean> data) {
      this(name, __ -> data.get());
    }

    @Override
    public void addTo(ShuffleboardLayout list, Supplier<T> common) {
      list.addBoolean(m_name, FunctionalUtil.supplyThenProcess(common, m_data)::get);
    }
  }

  private static class StringDataPoint<T> extends DataPoint<T> {
    private Function<T, String> m_data;

    public StringDataPoint(String name, Function<T, String> data) {
      super(name);
      m_data = data;
    }

    public StringDataPoint(String name, Supplier<String> data) {
      this(name, __ -> data.get());
    }

    @Override
    public void addTo(ShuffleboardLayout list, Supplier<T> common) {
      list.addString(m_name, FunctionalUtil.supplyThenProcess(common, m_data));
    }
  }

  private static class FormattedDataPoint<T, V> extends DataPoint<T> {
    private String m_fmt;
    private Function<T, V> m_data;

    public FormattedDataPoint(String name, String fmt, Function<T, V> data) {
      super(name);
      m_fmt = fmt;
      m_data = data;
    }

    public FormattedDataPoint(String name, String fmt, Supplier<V> data) {
      this(name, fmt, __ -> data.get());
    }

    @Override
    public void addTo(ShuffleboardLayout list, Supplier<T> common) {
      list.addString(m_name, () -> String.format(m_fmt, FunctionalUtil.supplyThenProcess(common, m_data)));
    }
  }

  private static class DoubleDataPoint<T> extends FormattedDataPoint<T, Double> {
    public DoubleDataPoint(String name, String prec, String units, Function<T, Double> data) {
      super(name, String.format("%%%sf%s", prec, units), data);
    }

    public DoubleDataPoint(String name, String prec, String units, Supplier<Double> data) {
      this(name, prec, units, __ -> data.get());
    }
  }

  public static ShuffleboardLayout add(ShuffleboardTab tab, String header, List<DataPoint<?>> data) {
    ShuffleboardLayout list = tab.getLayout(header, BuiltInLayouts.kList);

    data.forEach(dataPoint -> dataPoint.addTo(list));
 
    return list.withProperties(Map.of("Label position", "TOP"));
  }

  public static <T> ShuffleboardLayout add(ShuffleboardTab tab, String header, Supplier<T> common, List<DataPoint<T>> data) {
    ShuffleboardLayout list = tab.getLayout(header, BuiltInLayouts.kList);

    data.forEach(dataPoint -> dataPoint.addTo(list, common));
 
    return list.withProperties(Map.of("Label position", "TOP"));
  }
}
