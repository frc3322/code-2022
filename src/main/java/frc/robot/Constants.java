// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public final class Constants {

  public static final class Shooter {
    public static final double ksVolts = -0.033976;
    public static final double kvVoltSecondsPerRadian = 0.13123 / (2 * Math.PI);
    public static final double kaVoltSecondsSquaredPerRadian = 0.013611 / (2 * Math.PI);

    public static final LinearSystem<N1, N1, N1> kFlywheelPlant =
        LinearSystemId.identifyVelocitySystem(
            kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kFlywheelGearbox = DCMotor.getNEO(2);

    public static final double kFlywheelGearing = 1;

    public static final int targetRPM = 2000;
  }

  public static final class CAN {

    // fl = front left
    public static final int driveFL = 0;
    public static final int driveFR = 1;
    public static final int driveBR = 2;
    public static final int driveBL = 3;

    public static final int flywheelL = 11;
    public static final int flywheelR = 10;
    public static final int transfer = 14;
    public static final int intake = 6;

    public static final int climberL = 7;
    public static final int climberR = 8;
  }

  public static final class DIO {
    public static final int breakBeamA = 0;
    public static final int breakBeamB = 2;
  }

  public static final class Drive {
    public static final double kTrackwidthMeters = 0.69; // 0.66
    public static final DifferentialDriveKinematics kKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kWheelDiameterMeters = 0.15; // 6*0.0254

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.22; // 0.161
    public static final double kvVoltSecondsPerMeter = 1.98; // 3.05
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; // 0.475

    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 8; // 242.0/2480.0

    public static final double kPVel = 8.5; // 10.0
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
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
