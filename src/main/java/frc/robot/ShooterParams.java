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

  private static final TreeMap<Double, Double> distanceMetersToRPMTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.944, 1260.0),
              entry(2.202, 1375.0),
              entry(2.421, 1450.0),
              entry(2.844, 1500.0),
              entry(3.734, 1600.0),
              entry(4.068, 1750.0),
              entry(4.191, 1850.0),
              entry(4.342, 2000.0),
              entry(4.433, 2100.0),
              entry(4.753, 2250.0),
              entry(4.999, 2350.0),
              entry(5.238, 2500.0),
              entry(5.566, 2650.0),
              entry(5.858, 3000.0)));

  private static final TreeMap<Double, Double> distanceMetersToShootOffsetDegreesTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.944, 2.0),
              entry(4.009, 2.0),
              entry(5.566, 0.0)));

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

  public static Double getRPMFromDistanceMeters(double distanceMeters) {
    return lookUpValue(distanceMeters, distanceMetersToRPMTable);
  }

  public static Double getRPMFromDistanceMetersSupplier(DoubleSupplier distanceMeters) {
    return getRPMFromDistanceMeters(distanceMeters.getAsDouble());
  }

  public static Double getShootOffsetFromDistanceMeters(double distanceMeters) {
    return lookUpValue(distanceMeters, distanceMetersToShootOffsetDegreesTable);
  }

  public static Double getShootOffsetFromDistanceMetersSupplier(DoubleSupplier distanceMeters) {
    return getShootOffsetFromDistanceMeters(distanceMeters.getAsDouble());
  }

  private static Double interpolate(double y1, double y2, double t) {
    return y1 + t * (y2 - y1);
  }
}
