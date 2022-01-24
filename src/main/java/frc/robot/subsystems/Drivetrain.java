// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANEncoderSim;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;

public class Drivetrain extends SubsystemBase implements Loggable {

  private final CANSparkMax FL = new CANSparkMax(CAN.driveFL, MotorType.kBrushless);
  private final CANSparkMax FR = new CANSparkMax(CAN.driveFR, MotorType.kBrushless);
  private final CANSparkMax BL = new CANSparkMax(CAN.driveBL, MotorType.kBrushless);
  private final CANSparkMax BR = new CANSparkMax(CAN.driveBR, MotorType.kBrushless);

  private final RelativeEncoder FL_ENC;
  private final RelativeEncoder FR_ENC;
  private final RelativeEncoder BR_ENC;
  private final RelativeEncoder BL_ENC;

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDrive robotDrive;

  private final DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  private DifferentialDrivetrainSim m_drivetrainSimulator;
  private CANEncoderSim m_leftEncoderSim;
  private CANEncoderSim m_rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private SimDouble m_gyroSim;

  @Log private double rightVoltage;

  @Log private double leftVoltage;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    FL.restoreFactoryDefaults();
    FR.restoreFactoryDefaults();
    BL.restoreFactoryDefaults();
    BR.restoreFactoryDefaults();
    // do we need to reverse one side?
    BL.follow(FL);
    BR.follow(FR);

    FL_ENC = FL.getEncoder();
    FR_ENC = FR.getEncoder();
    BL_ENC = BL.getEncoder();
    BR_ENC = BR.getEncoder();

    rightVoltage = 0;
    leftVoltage = 0;

    robotDrive = new DifferentialDrive(FL, FR);
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    // the Field2d class lets us visualize our robot in the simulation GUI.
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    if (RobotBase.isSimulation()) {

      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              Drive.kDrivetrainPlant,
              Drive.kDriveGearbox,
              Drive.kDriveGearing,
              Drive.kTrackwidthMeters,
              Drive.kWheelDiameterMeters / 2.0,
              null
              /* VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) */ );

      m_leftEncoderSim = new CANEncoderSim(false, CAN.driveFL);
      m_rightEncoderSim = new CANEncoderSim(false, CAN.driveFR);
      m_gyroSim =
          new SimDouble(
              SimDeviceDataJNI.getSimValueHandle(
                  SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(gyro.getRotation2d(), FL_ENC.getPosition(), FR_ENC.getPosition());

    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setPosition(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setPosition(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    m_gyroSim.set(-m_drivetrainSimulator.getHeading().getDegrees());

    m_fieldSim.setRobotPose(getPose());
  }

  @Config
  public void arcadeDrive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);

    if (Robot.isSimulation()) {
      WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
      leftVoltage = wheelSpeeds.left * 12;
      rightVoltage = wheelSpeeds.right * 12;
    }
  }

  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getHeading() {
    return -gyro.getYaw();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(FL_ENC.getVelocity(), FR_ENC.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    FL.setVoltage(leftVolts);
    FR.setVoltage(-rightVolts);

    leftVoltage = leftVolts;
    rightVoltage = rightVolts;
    robotDrive.feed();
  }

  public void resetEncoders() {
    FL_ENC.setPosition(0);
    FR_ENC.setPosition(0);
  }

  public TrajectoryConfig getTrajConfig(boolean reversed) {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Drive.ksVolts, Drive.kvVoltSecondsPerMeter, Drive.kaVoltSecondsSquaredPerMeter),
            Drive.kKinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Drive.kKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(reversed);

    return config;
  }

  public Trajectory getExampleTraj() {
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at (1, 2) facing the +X direction
            new Pose2d(1, 2, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, 2, new Rotation2d(0)),
            // Pass config
            getTrajConfig(false));

    return exampleTrajectory;
  }

  public Trajectory getTrajFromFieldWidget(String traj, boolean reversed) {
    Trajectory trajectory;
    try {
      trajectory =
          TrajectoryGenerator.generateTrajectory(
              m_fieldSim.getObject(traj).getPoses(), getTrajConfig(reversed));
      ;
    } catch (Exception NegativeArraySizeException) {

      trajectory = getExampleTraj();
    }

    return trajectory;
  }

  public Command getRamseteCommand(Drivetrain robotDrive, Trajectory trajectory) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Drive.ksVolts, Drive.kvVoltSecondsPerMeter, Drive.kaVoltSecondsSquaredPerMeter),
            Drive.kKinematics,
            robotDrive::getWheelSpeeds,
            new PIDController(Drive.kPVel, 0, 0),
            new PIDController(Drive.kPVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robotDrive::tankDriveVolts,
            robotDrive);

    // Reset odometry to starting pose of trajectory.
    robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
  }

  public Command turnToAngleCommand(double targetAngle) {
    double Kp = SmartDashboard.getNumber("rotKp", 0);
    double Ki = SmartDashboard.getNumber("rotKi", 0);
    double Kd = SmartDashboard.getNumber("rotKd", 0);
    double maxVel = SmartDashboard.getNumber("rotMaxVel", 0);
    double maxAcc = SmartDashboard.getNumber("rotMaxAcc", 0);
    ProfiledPIDController controller =
        new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVel, maxAcc));
    ProfiledPIDCommand command =
        new ProfiledPIDCommand(
            controller,
            this::getHeading,
            targetAngle,
            (output, setpoint) -> tankDriveVolts(output, -output));

    return command;
  }
}
