// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Limelight;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterParams {
  private static ShuffleboardTab shooterTuning = Shuffleboard.getTab("Shoot Tuning");
  private static ShuffleboardLayout rpmTable = shooterTuning    
      .getLayout("RPM Table", BuiltInLayouts.kGrid)
      .withSize(2, 5);

  private static TreeMap<Double, NetworkTableEntry> shooterTableValues = new TreeMap<>(Map.ofEntries());

  private static final TreeMap<Double, Double> distanceMetersToRPMTable =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.0, 1260.0),
              entry(2.0, 1325.0),
              entry(3.0, 1600.0),
              entry(4.0, 1875.0),
              entry(5.0, 2050.0),
              entry(6.0, 2200.0),
              entry(7.0, 2650.0)));

  private ShooterParams() {
    for(Map.Entry<Double, Double> entry : distanceMetersToRPMTable.entrySet()) {
      NetworkTableEntry networkTableEntry = rpmTable.addPersistent(entry.getKey() + " meters", entry.getValue()).getEntry();
      Double key = entry.getKey();
      shooterTableValues.entrySet().add(Map.entry(key, networkTableEntry));
    }

    SmartDashboard.putNumber("Shooter First tuning", shooterTableValues.get(1.0).getValue().getDouble());
  }

  public static void updateShooterTunings() {

    // for(Map.Entry<Double, Double> entry : distanceMetersToRPMTable.entrySet()) {
    //   entry.setValue(shooterTableValues.get(entry.getKey()).getValue().getDouble());
    // }
  }

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
