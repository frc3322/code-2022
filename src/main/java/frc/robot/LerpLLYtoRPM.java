// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;
import java.util.TreeMap;

/** Add your docs here. */
public class LerpLLYtoRPM {
    private LerpLLYtoRPM() {}

    private static final TreeMap<Double, Double> table =
            new TreeMap<>(
                    Map.ofEntries(
                            entry(1.0, 0.0),
                            entry(2.0, 5.0),
                            entry(3.0, 10.0),
                            entry(4.0, 20.0)));

    public static Double getRPM(double limelightAngleY) {
        Entry<Double, Double> ceiling = table.ceilingEntry(limelightAngleY);
        Entry<Double, Double> floor = table.floorEntry(limelightAngleY);

        if (ceiling == null) return floor.getValue();
        if (floor == null) return ceiling.getValue();
        if (ceiling.getValue().equals(floor.getValue())) return ceiling.getValue();

        return interpolate(
            floor.getValue(),
            ceiling.getValue(),
            (limelightAngleY - floor.getKey()) / (ceiling.getKey() - floor.getKey()));
    }

    public static Double getRPMFromSupplier(DoubleSupplier limelightAngleY) {
        return getRPM(limelightAngleY.getAsDouble());
    }

    private static Double interpolate(double y1, double y2, double t) {
        return y1 + t * (y2 - y1);
    }
}