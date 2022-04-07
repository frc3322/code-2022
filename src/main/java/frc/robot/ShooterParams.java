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

  private static final TreeMap<Double, Double> distanceMetersToRPMTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.96, 1260.0),
              entry(2.11, 1325.0),
              entry(2.30, 1400.0),
              entry(2.52, 1475.0),
              entry(2.81, 1550.0),
              entry(3.21, 1600.0),
              entry(3.35, 1700.0),
              entry(3.44, 1725.0),
              entry(3.58, 1750.0),
              entry(3.75, 1775.0),
              entry(3.83, 1860.0),
              entry(4.03, 1875.0),
              entry(4.16, 1900.0),
              entry(4.45, 1930.0),
              entry(4.74, 1980.0),
              entry(4.89, 2175.0 - 180),
              entry(5.09, 2250.0 - 200),
              entry(5.27, 2275.0 - 200),
              entry(5.67, 2300.0 - 200),
              entry(5.93, 2425.0 - 150),
              entry(6.31, 2610.0 - 100),
              entry(6.78, 2650.0 - 100),
              entry(7.03, 2750.0 - 100)));

  private ShooterParams() {}

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

  private static double angleToDistMeters(double angleY, double mountingAngle) {
    double angleRadians = Units.degreesToRadians(angleY + mountingAngle);
    double distanceToGoalInches =
        (Limelight.visionTargetHeightInches - Limelight.mountingHeightInches)
            / Math.tan(angleRadians);
    return Units.inchesToMeters(distanceToGoalInches);
  }
}
