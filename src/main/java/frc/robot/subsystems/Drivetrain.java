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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive;
import frc.robot.RelativeEncoderSim;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase implements Loggable {

  private final CANSparkMax FL = new CANSparkMax(CAN.driveFL, MotorType.kBrushless);
  private final CANSparkMax FR = new CANSparkMax(CAN.driveFR, MotorType.kBrushless);
  private final CANSparkMax BL = new CANSparkMax(CAN.driveBL, MotorType.kBrushless);
  private final CANSparkMax BR = new CANSparkMax(CAN.driveBR, MotorType.kBrushless);

  private final RelativeEncoder FL_ENC = FL.getEncoder();
  private final RelativeEncoder FR_ENC = FR.getEncoder();

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDrive robotDrive = new DifferentialDrive(FL, FR);

  @Log private double limelightAngleX = 0;
  @Log private double limelightAngleY = 0;

  private double angleSetpoint = 0;

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  private final SimpleMotorFeedforward angleFF =
      new SimpleMotorFeedforward(
          Drive.ksAngularVolts,
          Drive.kvAngularVoltSecondsPerRadian,
          Drive.kaAngularVoltSecondsSquaredPerRadian);

  // P = 12, D = 0.06
  private final ProfiledPIDController profiledTurnToAngleController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(12.0, 8.0));

  private final PIDController turnToAngleController = 
      new PIDController(0, 0, 0);

  // These classes help us simulate our drivetrain
  private DifferentialDrivetrainSim drivetrainSimulator;
  private RelativeEncoderSim leftEncoderSim;
  private RelativeEncoderSim rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d fieldSim = new Field2d();
  private SimDouble gyroSim;

  @Log private double rightVoltage = 0;

  @Log private double leftVoltage = 0;

  @Log private double heading;
  @Log private double headingRad;
  @Log private double angVelRad;
  @Log private double angAccelRad;
  private double lastHeadingRad;
  private double lastAngVelRad;

  private double turnToAngleLastVel;

  @Log private double leftPosition;
  @Log private double rightPosition;

  @Log private double leftVel;
  @Log private double rightVel;

  @Log private double xPosition;
  @Log private double yPosition;

  // Account for different wheel directions between sim and test chassis
  private double wheelDirection = -1;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    FL.restoreFactoryDefaults();
    FR.restoreFactoryDefaults();
    BL.restoreFactoryDefaults();
    BR.restoreFactoryDefaults();

    // do we need to reverse one side?
    BL.follow(FL);
    BR.follow(FR);

    FL.setInverted(true);

    FL_ENC.setPositionConversionFactor(
        0.4788 / 10.71); // 10.71 gearing reduction and 0.4788 meters per rotation
    FR_ENC.setPositionConversionFactor(0.4788 / 10.71);

    FL_ENC.setVelocityConversionFactor(
        0.4788 / 10.71
            / 60); // 10.71 gearing reduction and 0.4788 meters per rotation and convert to per
    // second
    FR_ENC.setVelocityConversionFactor(0.4788 / 10.71 / 60);

    SmartDashboard.putData("turnToAngleProfiled", (SequentialCommandGroup) profiledTurnToAngleCommand(() -> getLimelightAngleX() + getHeading()));
    SmartDashboard.putData("turnToLimelight", (RunCommand) turnToLimelightCommand());
    SmartDashboard.putNumber("TurnToAngle/kP", 0);

    // the Field2d class lets us visualize our robot in the simulation GUI.
    SmartDashboard.putData("Field", fieldSim);
    fieldSim.getObject("Example Trajectory").setTrajectory(getExampleTraj());

    robotDrive.setSafetyEnabled(false);

    if (RobotBase.isSimulation()) {

      drivetrainSimulator =
          DifferentialDrivetrainSim.createKitbotSim(
              KitbotMotor.kDoubleNEOPerSide, // 2 NEOs per side.
              KitbotGearing.k10p71, // 10.71:1
              KitbotWheelSize.kSixInch, // 6" diameter wheels.
              null // No measurement noise.
              );

      leftEncoderSim = new RelativeEncoderSim(false, CAN.driveFL);
      rightEncoderSim = new RelativeEncoderSim(false, CAN.driveFR);
      gyroSim =
          new SimDouble(
              SimDeviceDataJNI.getSimValueHandle(
                  SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

      wheelDirection = 1;
    }
  }

  @Override
  public void periodic() {
    leftPosition = wheelDirection * FL_ENC.getPosition();
    rightPosition = wheelDirection * FR_ENC.getPosition();

    odometry.update(gyro.getRotation2d(), leftPosition, rightPosition);

    fieldSim.setRobotPose(odometry.getPoseMeters());

    leftVel = getWheelSpeeds().leftMetersPerSecond;
    rightVel = getWheelSpeeds().rightMetersPerSecond;

    xPosition = odometry.getPoseMeters().getX();
    yPosition = odometry.getPoseMeters().getY();

    double[] llpython =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("llpython")
            .getDoubleArray(new double[8]);

    double limelightTX = 
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("tx")
            .getDouble(0);

    double limelightTY = 
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("ty")
            .getDouble(0);

    limelightAngleX = -limelightTX; //llpython[0]
    limelightAngleY = limelightTY; //llpython[1]

    heading = getHeading();
    headingRad = getHeadingRad();
    angVelRad = getAngVelRad();
    angAccelRad = getAngAccelRad();
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
    drivetrainSimulator.update(0.020);

    leftEncoderSim.setPosition(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setVelocity(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setPosition(drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setVelocity(drivetrainSimulator.getRightVelocityMetersPerSecond());

    gyroSim.set(-drivetrainSimulator.getHeading().getDegrees());

    fieldSim.setRobotPose(getPose());
  }

  @Config
  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("rotation prop", rotation);
    robotDrive.arcadeDrive(speed, rotation);

    if (Robot.isSimulation()) {
      WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
      leftVoltage = wheelSpeeds.left * 12;
      rightVoltage = wheelSpeeds.right * 12;
    }
  }

  public double getDrawnCurrentAmps() {
    return drivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getHeadingRad() {
    return Math.toRadians(getHeading());
  }

  public double getAngVelRad() {
    double angVel = (getHeadingRad() - lastHeadingRad) / 0.02;
    lastHeadingRad = getHeadingRad();
    return angVel;
  }

  public double getLimelightAngleX() {
    return limelightAngleX;
  }

  public double getLimelightAngleY() {
    return limelightAngleY;
  }

  public double getAngAccelRad() {
    double angAccel = (angVelRad - lastAngVelRad) / 0.02;
    lastAngVelRad = angVelRad;
    return angAccel;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        wheelDirection * FL_ENC.getVelocity(), wheelDirection * FR_ENC.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());

    if (RobotBase.isSimulation()) {
      drivetrainSimulator.setPose(pose);
    }
  }

  @Config
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    FL.setVoltage(wheelDirection * leftVolts);
    FR.setVoltage(wheelDirection * rightVolts);

    leftVoltage = leftVolts;
    rightVoltage = rightVolts;
    robotDrive.feed();
  }

  public void resetEncoders() {
    FL_ENC.setPosition(0);
    FR_ENC.setPosition(0);
  }

  public void resetPoseAndSensors() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));
    resetEncoders();
    gyro.reset();
  }

  @Config
  public void setProfiledTurnToAnglePID(double P, double I, double D){
    profiledTurnToAngleController.setPID(P, I, D);
  }

  @Config
  public void setTurnToAnglePID(double P, double I, double D){
    turnToAngleController.setPID(P, I, D);
  }

  public void setProfiledTurnToAngleGoalSource(DoubleSupplier goalSource) {
    double goal = goalSource.getAsDouble();
    double goalRad = Math.toRadians(goal);
    profiledTurnToAngleController.reset(getHeadingRad(), getAngVelRad());
    profiledTurnToAngleController.setGoal(new TrapezoidProfile.State(goalRad, 0));
  }

  public void profiledTurnToAngle(DoubleSupplier goalSource) {
    double velSetpoint = profiledTurnToAngleController.getSetpoint().velocity;
    double accelSetpoint = (velSetpoint - turnToAngleLastVel) / 0.02;
    turnToAngleLastVel = velSetpoint;

    profiledTurnToAngleController.calculate(getHeadingRad());
    double FF = angleFF.calculate(velSetpoint, accelSetpoint);
    double kP = SmartDashboard.getNumber("TurnToAngle/kP", 0);
    double PID = kP * getLimelightAngleX();

    SmartDashboard.putNumber("TurnToAngle/velSetpoint", velSetpoint);
    SmartDashboard.putNumber("TurnToAngle/accelSetpoint", accelSetpoint);
    SmartDashboard.putNumber("TurnToAngle/FF", FF);
    SmartDashboard.putNumber("TurnToAngle/PID", PID);
    SmartDashboard.putNumber("TurnToAngle/AngleGoal", profiledTurnToAngleController.getGoal().position);

    tankDriveVolts(-(FF + PID), FF + PID);
  }

  public void turnToLimelight() {
    double PID = turnToAngleController.calculate(getLimelightAngleX(), 0);
    tankDriveVolts(PID, -PID);
  }

  public Command profiledTurnToAngleCommand(DoubleSupplier goalSource) {
    return new InstantCommand(() -> setProfiledTurnToAngleGoalSource(goalSource))
        .andThen(new RunCommand(() -> profiledTurnToAngle(goalSource), this)/*.withInterrupt(() -> turnToAngleController.atGoal())*/);
  }

  public Command turnToLimelightCommand() {
    return new RunCommand(this::turnToLimelight);
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
              fieldSim.getObject(traj).getPoses(), getTrajConfig(reversed));
    } catch (Exception NegativeArraySizeException) {

      trajectory = getExampleTraj();
    }

    return trajectory;
  }

  public Command getRamseteCommand(Drivetrain robotDrive, Trajectory trajectory) {
    RamseteController disabledRamsete = new RamseteController();
    disabledRamsete.setEnabled(false);

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

    // Run path following command, then stop at the end.
    return ramseteCommand
        .beforeStarting(() -> robotDrive.resetOdometry(trajectory.getInitialPose()))
        .andThen(() -> robotDrive.tankDriveVolts(0, 0));
  }
}
