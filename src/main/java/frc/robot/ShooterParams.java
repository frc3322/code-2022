// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Limelight;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

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

  private static final TreeMap<Double, Double> angleToRPMTableOld =
      new TreeMap<>(
          Map.ofEntries(
              entry(18.5, 1260.0),
              entry(16.4, 1325.0),
              entry(14.0, 1400.0),
              entry(11.5, 1475.0),
              entry(8.7, 1550.0),
              entry(5.4, 1600.0),
              entry(4.45, 1700.0),
              entry(3.8, 1725.0),
              entry(2.95, 1750.0),
              entry(1.92, 1775.0),
              entry(1.45, 1860.0),
              entry(0.4, 1875.0),
              entry(-0.26, 1900.0),
              entry(-1.59, 1930.0),
              entry(-2.75, 1980.0),
              entry(-3.3, 2175.0),
              entry(-4.0, 2250.0),
              entry(-4.6, 2275.0),
              entry(-5.8, 2300.0),
              entry(-6.5, 2425.0),
              entry(-7.4, 2610.0),
              entry(-8.4, 2650.0),
              entry(-8.9, 2750.0)));

  private static final TreeMap<Double, Double> distanceMetersToShootOffsetDegreesTable =
      new TreeMap<>(Map.ofEntries(entry(1.944, 2.0), entry(4.009, 2.0), entry(5.566, 0.0)));

  private static Double interpolate(double y1, double y2, double t) {
    return y1 + t * (y2 - y1);
  }

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

  private static double distMetersToAngleDegrees(double distMeters, double mountingAngle) {
    double angleRadians =
        Math.atan(
            (Limelight.visionTargetHeightInches - Limelight.mountingHeightInches) / distMeters);

    return Units.radiansToDegrees(angleRadians);
  }

  public static Double getOldRPMFromDistanceMeters(Double distanceMeters) {
    return lookUpValue(distMetersToAngleDegrees(distanceMeters, 22.0), angleToRPMTableOld);
  }
}
