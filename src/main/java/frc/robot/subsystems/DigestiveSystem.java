// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Shooter;

public class DigestiveSystem extends SubsystemBase {

  private final CANSparkMax intake = new CANSparkMax(CAN.intake, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer, MotorType.kBrushless);
  private final CANSparkMax flywheel1 = new CANSparkMax(CAN.flywheelL, MotorType.kBrushless);
  private final CANSparkMax flywheel2 = new CANSparkMax(CAN.flywheelR, MotorType.kBrushless);

  private final RelativeEncoder intake_ENC;
  private final RelativeEncoder transfer_ENC;
  private final RelativeEncoder flywheel_ENC;

  private boolean beam1Broken = false;
  private boolean beam2Broken = false;

  // private double flywheelRPM = 0;

  // private double lastPosition = 0;

  // private Timer loopTime = new Timer();

  PIDController flywheelPID = new PIDController(0.1, 0, 0);
  BangBangController flywheelBangBang = new BangBangController();

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Shooter.ksVolts, Shooter.kvVoltSecondsPerRotation);

  /** Creates a new Intake. */
  public DigestiveSystem() {

    intake.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheel1.restoreFactoryDefaults();
    flywheel2.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    transfer.setIdleMode(IdleMode.kCoast);
    flywheel1.setIdleMode(IdleMode.kCoast);
    flywheel2.setIdleMode(IdleMode.kCoast);

    intake_ENC = intake.getEncoder();
    transfer_ENC = transfer.getEncoder();
    flywheel_ENC = flywheel1.getEncoder();

    SmartDashboard.putNumber("flywheel/SpeedSetpoint", 0);
    SmartDashboard.putNumber("flywheel/VoltageInput", 0);
    SmartDashboard.putNumber("flywheel/kP", flywheelPID.getP());
    SmartDashboard.putNumber("transfer/speed", 0);

    flywheel1.setInverted(true);
    flywheel2.follow(flywheel1, true);

    // loopTime.start();
  }

  public void setSpeed(double speed) {
    flywheel1.set(speed);
  }

  public void stop() {
    flywheel1.setVoltage(0);
  }

  // intake methods
  public void takeIn(double pwr) {
    intake.set(pwr);
  }

  @Override
  public void periodic() {
    double targetSpeed = SmartDashboard.getNumber("flywheel/SpeedSetpoint", 0.0);

    // setSpeed(SmartDashboard.getNumber("flywheel/SpeedSetpoint", 0.0));
    SmartDashboard.putNumber("flywheel/Vel", flywheel_ENC.getVelocity());
    SmartDashboard.putNumber("flywheel/Pos", flywheel_ENC.getPosition());

    // flywheelRPM = ((flywheel_ENC.getPosition() - lastPosition) / loopTime.get())
    // * 60;
    // loopTime.reset();
    // lastPosition = flywheel_ENC.getPosition();

    SmartDashboard.putNumber("flywheel/RPM", flywheel_ENC.getVelocity());

    /*
     * // This method will be called once per scheduler run
     * //run until beam1 not broken
     * //don't run if beam 2 broken.
     * if(beam2Broken||!beam1Broken){
     * intake.set(0);
     * }
     * 
     * double targetSpeed = SmartDashboard.getNumber("flywheelSpeed", 0.0);
     * if (flywheel_ENC.getVelocity() < targetSpeed) {
     * flywheel.set(1);
     * } else {
     * flywheel.set(0);
     * }
     */
    // getting values for ff from smart dashboard
    // flywheel.set(bangBang.calculate(flywheel_ENC.getVelocity(), targetSpeed) +
    // 0.9 * feedForward.calculate(targetSpeed));

    // flywheel1.setVoltage(SmartDashboard.getNumber("flywheel/VoltageInput", 0));
    transfer.set(SmartDashboard.getNumber("transfer/speed", 0));

    double targetSpeedRPS = targetSpeed / 60;
    double currentSpeedRPS = flywheel_ENC.getVelocity() / 60;
    SmartDashboard.putNumber("flywheel/velRPS", currentSpeedRPS);
    SmartDashboard.putNumber("flywheel/targetSpeedRPS", targetSpeedRPS);

    flywheelPID.setP(SmartDashboard.getNumber("flywheel/kP", flywheelPID.getP()));

    double ffEffort = feedForward.calculate(targetSpeedRPS);
    double bbEffort = flywheelBangBang.calculate(currentSpeedRPS, targetSpeedRPS);
    double totalEffort = ffEffort + bbEffort * 0.25;
    flywheel1.setVoltage(totalEffort);

    SmartDashboard.putNumber("flywheel/ffEffort", ffEffort);
    SmartDashboard.putNumber("flywheel/bbEffort", bbEffort);
    SmartDashboard.putNumber("flywheel/totalControlEffort", totalEffort);
  }

}
