// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Shooter {
        public static final double Ks = -0.033976;
        public static final double Kv = 0.13123;
        public static final double Ka = 0.013611;

    }
   
    public static final class CAN{

        //fl = front left
        public static final int FL_ID = 0;
        public static final int FR_ID = 1;
        public static final int BR_ID = 2;
        public static final int BL_ID = 3;

        public static final int flywheel1_ID = 11;
        public static final int flywheel2_ID = 10;
        public static final int transfer_ID = 14;
        public static final int intake_ID = 6;

        public static final int ClimberR_ID = 7;
        public static final int ClimberL_ID = 8;


    }

    public static final class Drive {

        
        public static final double kTrackwidthMeters = 0.69; //0.66
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final double kWheelDiameterMeters = 0.15; //6*0.0254
    
        public static final boolean kGyroReversed = true;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22; //0.161
        public static final double kvVoltSecondsPerMeter = 1.98; //3.05
        public static final double kaVoltSecondsSquaredPerMeter = 0.2; //0.475
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;
    
        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter,
                kvVoltSecondsPerRadian,
                kaVoltSecondsSquaredPerRadian);
    
        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getCIM(2); //.getNEO(2)
        public static final double kDriveGearing = 8; //242.0/2480.0

        public static final double kPVel = 8.5; //10.0

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }


}
