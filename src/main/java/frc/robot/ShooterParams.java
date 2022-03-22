// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.Limelight;

/** Add your docs here. */
public class ShooterParams {
  private ShooterParams() {}

  private static final TreeMap<Double, Double> angleToRPMTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(18.5, 1260.0),
              entry(16.4, 1325.0),
              entry(14.0, 1400.0),
              entry(11.5, 1475.0),
              entry(8.7, 1600.0 - 50.0),
              entry(5.4, 1650.0 - 50.0),
              entry(4.45, 1750.0 - 50.0),
              entry(3.8, 1775.0 - 50.0),
              entry(2.95, 1800.0 - 50.0),
              entry(1.92, 1825.0 - 50.0),
              entry(1.45, 1910.0 - 50.0),
              entry(0.4, 1925.0 - 50.0),
              entry(-0.26, 1950.0 - 50.0),
              entry(-1.59, 1980.0 - 50.0),
              entry(-2.75, 2030.0 - 50.0),
              entry(-3.3, 2175.0),
              entry(-4.0, 2250.0),
              entry(-4.6, 2275.0),
              entry(-5.8, 2300.0),
              entry(-6.5, 2425.0),
              entry(-7.4, 2610.0),
              entry(-8.4, 2650.0),
              entry(-8.9, 2750.0)));

  private static final TreeMap<Double, Double> angleToShootOffsetTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(19.2, 2.0),
              entry(1.0, 2.0),
              entry(-5.0, 0.0)));

  private static Double lookUpValue(double value, TreeMap<Double, Double> table) {
    Entry<Double, Double> ceiling = table.ceilingEntry(value);
    Entry<Double, Double> floor = table.floorEntry(value);

    if (ceiling == null) return floor.getValue();
    if (floor == null) return ceiling.getValue();
    if (ceiling.getValue().equals(floor.getValue())) return ceiling.getValue();

    return interpolate(
        floor.getValue(),
        ceiling.getValue(),
        (value - floor.getKey()) / (ceiling.getKey() - floor.getKey()));
  }

  public static Double getRPMFromAngle(double angle) {
    return lookUpValue(angle, angleToRPMTable);
  }

  public static Double getRPMFromAngleSupplier(DoubleSupplier angle) {
    return getRPMFromAngle(angle.getAsDouble());
  }

  public static Double getShootOffsetFromAngle(double angle) {
    return lookUpValue(angle, angleToShootOffsetTable);
  }

  public static Double getShootOffsetFromAngleSupplier(DoubleSupplier angle) {
    return getShootOffsetFromAngle(angle.getAsDouble());
  }

  private static Double interpolate(double y1, double y2, double t) {
    return y1 + t * (y2 - y1);
  }
}
