// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class Constants {

  public static final class Shooter {
    public static final double ksVolts = 0; // 0.10413 0.012354, -0.033976
    public static final double kvVoltSecondsPerRotation =
        0.13672; // 0.012655, 0.13123  / (2 * Math.PI)
    public static final double kaVoltSecondsSquaredPerRotation =
        0.066727; // 0.00050103, 0.013611  / (2 * Math.PI)

    public static final double kvVoltSecondsPerRadian = kvVoltSecondsPerRotation / (2 * Math.PI);
    public static final double kaVoltSecondsSquaredPerRadian =
        kaVoltSecondsSquaredPerRotation / (2 * Math.PI);

    public static final LinearSystem<N1, N1, N1> kFlywheelPlant =
        LinearSystemId.identifyVelocitySystem(
            kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kFlywheelGearbox = DCMotor.getNEO(2);

    public static final double kFlywheelGearing = 1;
  }

  public static final class CAN {

    // fl = front left
    public static final int driveFL = 20;
    public static final int driveFR = 30; // 36
    public static final int driveBR = 25;
    public static final int driveBL = 15;

    public static final int flywheelL = 10;
    public static final int flywheelR = 5;
    public static final int transfer = 36;
    public static final int intake = 50;

    public static final int extIntakeLift = 35; // TODO: placeholder, give real id
    public static final int extIntakeTurn = 11;

    public static final int climberL = 45;
    public static final int climberR = 40;
  }

  public static final class DIO {
    public static final int breakBeamA = 0;
    public static final int breakBeamB = 2;
  }

  public static final class Drive {
    public static final double kTrackwidthMeters = 0.703; // 0.66
    public static final DifferentialDriveKinematics kKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kWheelDiameterMeters = 0.1524; // 0.15
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.1474; // 0.20747   0.22
    public static final double kvVoltSecondsPerMeter = 2.7911; // 2.819 1.98
    public static final double kaVoltSecondsSquaredPerMeter = 0.68337; // 0.30444  0.2

    // TODO: Find these values

    public static final double kvAngularVoltSecondsPerRadian =
        1.3894 * .5 * kTrackwidthMeters; // 1.3894
    public static final double kaAngularVoltSecondsSquaredPerRadian =
        0.17271 * .5 * kTrackwidthMeters; // 0.17271
    public static final double ksAngularVolts = 0.45806; // 0.45806

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 10.71; // 8

    public static final double kPVel = 6; // 8.5  1.8422
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.7;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Drive.ksVolts, Drive.kvVoltSecondsPerMeter, Drive.kaVoltSecondsSquaredPerMeter),
            Drive.kKinematics,
            9);

    // Create config for trajectory
    public static final TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Drive.kKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(false);

    // Create reversed config for trajectory
    public static final TrajectoryConfig reversedConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Drive.kKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    public static final double flywheelIdleRPM = 3000;
  }

  public static class XBOX {
    // Buttons
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int STICK_LEFT = 9;
    public static final int STICK_RIGHT = 10;

    // Axes
    public static final int STICK_L_X_AXIS = 0;
    public static final int STICK_L_Y_AXIS = 1;
    public static final int STICK_R_X_AXIS = 4;
    public static final int STICK_R_Y_AXIS = 5;
    public static final int TRIGGER_L_AXIS = 2;
    public static final int TRIGGER_R_AXIS = 3;

    public static final double MIN_DEADZONE = 0.13;
    public static final double MAX_DEADZONE = 0.15;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
  }
}
