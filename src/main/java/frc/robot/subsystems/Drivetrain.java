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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive;
import frc.robot.LerpLLYtoRPM;
import frc.robot.RelativeEncoderSim;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase implements Loggable {

  // Create motors
  private final CANSparkMax FL = new CANSparkMax(CAN.driveFL, MotorType.kBrushless);
  private final CANSparkMax FR = new CANSparkMax(CAN.driveFR, MotorType.kBrushless);
  private final CANSparkMax BL = new CANSparkMax(CAN.driveBL, MotorType.kBrushless);
  private final CANSparkMax BR = new CANSparkMax(CAN.driveBR, MotorType.kBrushless);

  // Create encoders
  private final RelativeEncoder FL_ENC = FL.getEncoder();
  private final RelativeEncoder FR_ENC = FR.getEncoder();

  // Create gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Create diff drive
  private final DifferentialDrive robotDrive = new DifferentialDrive(FL, FR);

  // Create odometry
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  // Controllers
  private final SimpleMotorFeedforward angleFF =
      new SimpleMotorFeedforward(
          Drive.ksAngularVolts,
          Drive.kvAngularVoltSecondsPerRadian,
          Drive.kaAngularVoltSecondsSquaredPerRadian);

  private final ProfiledPIDController profiledTurnToAngleController =
      new ProfiledPIDController(4, 0, 0.0006, new TrapezoidProfile.Constraints(10.0, 15.0));
  // P = 12, D = 0.06

  private final PIDController turnToAngleController = new PIDController(0.18, 0, 0.007);

  // Drivetrain sim
  private DifferentialDrivetrainSim drivetrainSimulator;
  private RelativeEncoderSim leftEncoderSim;
  private RelativeEncoderSim rightEncoderSim;
  private SimDouble gyroSim;

  // Field gui
  private Field2d fieldSim = new Field2d();

  @Log private double limelightAngleX = 0;
  @Log private double limelightAngleY = 0;
  @Log private boolean limelightHasTarget;

  @Log private double rightVoltage = 0;
  @Log private double leftVoltage = 0;

  @Log private double heading;
  @Log private double headingRad;
  @Log private double angVelRad;
  @Log private double angAccelRad;
  @Log private double angAccelDeg;
  private double lastHeadingRad;
  private double lastAngVelRad;

  private double turnToAngleLastVel;

  @Log private double leftPosition;
  @Log private double rightPosition;
  @Log private double leftVel;
  @Log private double rightVel;
  private double lastLeftVel;
  private double lastRightVel;
  @Log private double leftAccel;
  @Log private double rightAccel;
  @Log private double xPosition;
  @Log private double yPosition;

  private LinearFilter accelFilter = LinearFilter.movingAverage(40);
  SlewRateLimiter accelLimit = new SlewRateLimiter(2);
  SlewRateLimiter turnLimit = new SlewRateLimiter(2);

  @Log private boolean profiledTurnToAngleAtGoal;

  @Log private double profiledTurnToAngleVelSetpoint;
  @Log private double profiledTurnToAngleAccelSetpoint;
  @Log private double profiledTurnToAngleGoal;

  private double limelightThreshold = 2.1;

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

    SmartDashboard.putData(
        "turnToAngleProfiled", (SequentialCommandGroup) profiledTurnToAngleCommand(() -> 195.6));
    SmartDashboard.putData("turnToLimelight", (RunCommand) getTurnToLimelightCommand());
    SmartDashboard.putNumber("TurnToAngle/kP", 0);

    turnToAngleController.setTolerance(limelightThreshold, 0.5);
    profiledTurnToAngleController.setTolerance(0.5);

    // the Field2d class lets us visualize our robot in the simulation GUI.
    SmartDashboard.putData("Field", fieldSim);

    robotDrive.setSafetyEnabled(false);

    // set drivetrain current limit to prevent brown out
    // FL.setSmartCurrentLimit(30, 40);
    // FR.setSmartCurrentLimit(30, 40);
    // BL.setSmartCurrentLimit(30, 40);
    // BR.setSmartCurrentLimit(30, 40);

    if (RobotBase.isSimulation()) {

      limelightAngleX = Drive.shootOffset;
      limelightAngleY = 10; // Reasonable non-zero angle for testing

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
    LEDs.get()
        .setCondition(LEDs.Modes.ANGLE_GOOD, () -> getLimelightAligned() && limelightHasTarget);
  }

  @Override
  public void periodic() {
    leftPosition = wheelDirection * FL_ENC.getPosition();
    rightPosition = wheelDirection * FR_ENC.getPosition();

    leftAccel = accelFilter.calculate((leftVel - lastLeftVel) / 0.02);
    rightAccel = accelFilter.calculate((rightVel - lastRightVel) / 0.02);
    lastLeftVel = leftVel;
    lastRightVel = rightVel;

    odometry.update(new Rotation2d(getHeadingRad()), leftPosition, rightPosition);

    fieldSim.setRobotPose(odometry.getPoseMeters());

    leftVel = getWheelSpeeds().leftMetersPerSecond;
    rightVel = getWheelSpeeds().rightMetersPerSecond;

    xPosition = odometry.getPoseMeters().getX();
    yPosition = odometry.getPoseMeters().getY();

    SmartDashboard.putNumber(
        "TARGET RPM RIGHT HERE LOOK", LerpLLYtoRPM.getRPMFromSupplier(() -> limelightAngleY));

    double[] llpython =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("llpython")
            .getDoubleArray(new double[8]);

    double limelightTX =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    double limelightTY =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    double limelightTV =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (Robot.isReal()) {
      limelightAngleX = -limelightTX; // llpython[0]
      limelightAngleY = limelightTY; // llpython[1]
      limelightHasTarget = limelightTV == 1;
    }

    heading = getHeading();
    headingRad = getHeadingRad();
    angVelRad = getAngVelRad();
    angAccelRad = getAngAccelRad();
    angAccelDeg = Units.radiansToDegrees(angAccelRad);

    profiledTurnToAngleAtGoal = profiledTurnToAngleController.atGoal();
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
    drivetrainSimulator.update(0.020);

    leftEncoderSim.setPosition(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setVelocity(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setPosition(drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setVelocity(drivetrainSimulator.getRightVelocityMetersPerSecond());

    leftVel = getWheelSpeeds().leftMetersPerSecond;
    rightVel = getWheelSpeeds().rightMetersPerSecond;

    gyroSim.set(-drivetrainSimulator.getHeading().getDegrees());

    fieldSim.setRobotPose(getPose());
  }

  // @Config
  public void drive(double speed, double turn) {

    turn = 0.5 * turn + 0.5 * Math.pow(turn, 3);

    WheelSpeeds curveDriveSpeedsProp =
        DifferentialDrive.arcadeDriveIK(accelLimit.calculate(speed), turn, false);
    double leftVolts = curveDriveSpeedsProp.left * 12;
    double rightVolts = curveDriveSpeedsProp.right * 12;

    if (Robot.isReal()) {
      leftVolts += Math.copySign(Constants.Drive.ksVolts, leftVolts);
      rightVolts +=
          Math.copySign(
              Math.abs(rightVolts) > 0.2 ? Constants.Drive.ksVolts + 0.35 : Constants.Drive.ksVolts,
              rightVolts);
    }

    leftVolts = MathUtil.clamp(leftVolts, -12, 12);
    rightVolts = MathUtil.clamp(rightVolts, -12, 12);

    SmartDashboard.putNumber("drive test/leftVolts", leftVolts);
    SmartDashboard.putNumber("drive test/rightVolts", rightVolts);

    tankDriveVolts(leftVolts, rightVolts);

    if (Robot.isSimulation()) {
      leftVoltage = leftVolts;
      rightVoltage = rightVolts;
    }
  }

  public double getDrawnCurrentAmps() {
    return drivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    if (Robot.isReal()) {
      return -gyro.getRotation2d().getDegrees();
    } else {
      return gyro.getRotation2d().getDegrees();
    }
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

  public double getLLDistMeters() {
    double angleToGoalRadians = getLimelightAngleY() * Math.PI / 180;
    double distanceToGoalIN = 71 / Math.tan(angleToGoalRadians);
    return distanceToGoalIN * 0.0254;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        wheelDirection * FL_ENC.getVelocity(), wheelDirection * FR_ENC.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(getHeadingRad()));

    if (RobotBase.isSimulation()) {
      drivetrainSimulator.setPose(pose);
    }
  }

  public void zeroOdometry() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetGyro(double offset) {
    gyro.reset();
    if (Robot.isReal()) {
      gyro.setAngleAdjustment(offset);
    } else {
      gyro.setAngleAdjustment(-offset);
    }
  }

  // @Config
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

  // @Config
  public void setTurnToAnglePID(double P, double I, double D) {
    turnToAngleController.setPID(P, I, D);
  }

  public void turnToLimelight() {
    double PID = turnToAngleController.calculate(getLimelightAngleX(), Drive.shootOffset); // 3.5
    double ks = Math.copySign(Constants.Drive.ksVolts, PID);
    double effort = Robot.isReal() ? PID + ks : PID;
    tankDriveVolts(effort, -effort);
  }

  public Command getTurnToLimelightCommand() {
    return new RunCommand(this::turnToLimelight, this)
    /*.withInterrupt(() -> turnToAngleController.atSetpoint())*/ ;
  }

  @Log
  public Boolean getTurnToAngleAtSetpoint() {
    return turnToAngleController.atSetpoint();
  }

  // Only for LEDs
  public Boolean getLimelightAligned() {
    return Math.abs(limelightAngleX - 2) < limelightThreshold;
  }

  // @Config
  public void setProfiledTurnToAnglePID(double P, double I, double D) {
    profiledTurnToAngleController.setPID(P, I, D);
  }

  public void setProfiledTurnToAngleGoalSource(DoubleSupplier goalSource) {
    double goal = goalSource.getAsDouble();
    double goalRad = Math.toRadians(goal);
    profiledTurnToAngleController.reset(getHeadingRad());
    profiledTurnToAngleController.setGoal(new TrapezoidProfile.State(goalRad, 0));
  }

  public void profiledTurnToAngle() {
    double velSetpoint = profiledTurnToAngleController.getSetpoint().velocity;
    double accelSetpoint = (velSetpoint - turnToAngleLastVel) / 0.02;
    turnToAngleLastVel = velSetpoint;

    double PID = profiledTurnToAngleController.calculate(getHeadingRad());
    double FF = angleFF.calculate(velSetpoint, accelSetpoint);

    profiledTurnToAngleVelSetpoint = velSetpoint;
    profiledTurnToAngleAccelSetpoint = accelSetpoint;
    profiledTurnToAngleGoal = profiledTurnToAngleController.getGoal().position;
    SmartDashboard.putNumber("TurnToAngle/FF", FF);
    SmartDashboard.putNumber("TurnToAngle/PID", PID);

    tankDriveVolts(-(FF + PID), FF + PID);
  }

  public Command profiledTurnToAngleCommand(DoubleSupplier goalSource) {
    return new InstantCommand(() -> setProfiledTurnToAngleGoalSource(goalSource))
        .andThen(
            new RunCommand(() -> profiledTurnToAngle(), this)
                .withInterrupt(() -> profiledTurnToAngleController.atGoal()))
        .andThen(() -> tankDriveVolts(0, 0));
  }

  public void putTrajOnFieldWidget(Trajectory trajectory, String label) {
    fieldSim.getObject(label).setTrajectory(trajectory);
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

  public Trajectory getTrajFromFieldWidget(String traj, boolean reversed) {
    Trajectory trajectory;
    try {
      trajectory =
          TrajectoryGenerator.generateTrajectory(
              fieldSim.getObject(traj).getPoses(), getTrajConfig(reversed));
    } catch (Exception NegativeArraySizeException) {

      trajectory =
          TrajectoryGenerator.generateTrajectory(
              List.of(new Pose2d(0, 0, new Rotation2d(0))), getTrajConfig(false));
    }

    return trajectory;
  }

  public Command getRamseteCommand(Drivetrain robotDrive, Trajectory trajectory) {

    var table = NetworkTableInstance.getDefault().getTable("Ramsete Vals");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    PIDController leftController = new PIDController(Drive.kPVel, 0, 0);
    PIDController rightController = new PIDController(Drive.kPVel, 0, 0);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Drive.ksVolts, Drive.kvVoltSecondsPerMeter, Drive.kaVoltSecondsSquaredPerMeter),
            Drive.kKinematics,
            robotDrive::getWheelSpeeds,
            leftController,
            rightController,
            // RamseteCommand passes volts to the callback
            (leftVolts, rightVolts) -> {
              tankDriveVolts(leftVolts, rightVolts);

              leftMeasurement.setNumber(getWheelSpeeds().leftMetersPerSecond);
              leftReference.setNumber(leftController.getSetpoint());

              rightMeasurement.setNumber(getWheelSpeeds().rightMetersPerSecond);
              rightReference.setNumber(rightController.getSetpoint());
            },
            robotDrive);

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
        // new InstantCommand(() -> resetOdometry(trajectory.getInitialPose())),
        ramseteCommand, new InstantCommand(() -> tankDriveVolts(0, 0)));
  }
}
