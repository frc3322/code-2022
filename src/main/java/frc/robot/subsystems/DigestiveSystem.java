// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Shooter;
import frc.robot.RelativeEncoderSim;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

public class DigestiveSystem extends SubsystemBase implements Loggable {

  // Create motors
  private final CANSparkMax intake = new CANSparkMax(CAN.intake, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer, MotorType.kBrushless);
  private final CANSparkMax flywheelL = new CANSparkMax(CAN.flywheelL, MotorType.kBrushless);
  private final CANSparkMax flywheelR = new CANSparkMax(CAN.flywheelR, MotorType.kBrushless);
  private final CANSparkMax intakeExternal =
      new CANSparkMax(CAN.extIntakeTurn, MotorType.kBrushless);
  private final CANSparkMax intakeExternalLift =
      new CANSparkMax(CAN.extIntakeLift, MotorType.kBrushless);

  // Create encoders
  private final RelativeEncoder flywheelMotorEncoder = flywheelL.getEncoder();
  private final Encoder flywheelShaftEncoder =
      new Encoder(4, 3 /*, false, Encoder.EncodingType.k4X*/);

  // Create break beam sensors
  private final DigitalInput breakBeamMouth = new DigitalInput(DIO.breakBeamA);
  private final DigitalInput breakBeamStomach = new DigitalInput(DIO.breakBeamB);

  // Controllers
  PIDController flywheelPID = new PIDController(0.0015, 0, 0); // 0.00009 0.0012

  SimpleMotorFeedforward flywheelFF =
      new SimpleMotorFeedforward(Shooter.ksVolts, Shooter.kvVoltSecondsPerRotation);

  // Flywheel sim
  private FlywheelSim flywheelSimulator;
  private EncoderSim flywheelEncoderSim;

  // Flywheel measurements
  @Log private double flywheelVelRPM;
  private double lastFlywheelVelRPM;
  @Log private double flywheelAccelRPMPerS;
  @Log private double flywheelVelRPMShaftEnc;
  @Log private double flywheelPositionShaftEnc;

  private LinearFilter accelFilter = LinearFilter.movingAverage(40);

  // Flywheel control inputs
  @Log private double flywheelTargetVelRPM;
  @Log private double flywheelFFEffort;
  @Log private double flywheelPIDEffort;
  @Log private double flywheelTotalEffort;
  @Log private double flywheelVoltage;

  // Intake and transfer control inputs
  @Log private double intakeSpeedVolts;
  @Log private double transferSpeedVolts;
  @Log private double intakeExternalSpeedVolts;
  @Log private double intakeExternalLiftSpeedVolts;

  @Log private double intakeExternalLiftCurrent;

  // Track transfer states
  @Log private boolean ballInMouth = false;
  @Log private boolean stomachFull = false;

  private boolean spinFlywheelCustomFreq = false;

  public DigestiveSystem() {

    // Set up motors
    intake.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheelL.restoreFactoryDefaults();
    flywheelR.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    intake.setInverted(true);
    transfer.setIdleMode(IdleMode.kBrake);
    flywheelL.setIdleMode(IdleMode.kCoast);
    flywheelR.setIdleMode(IdleMode.kCoast);
    intakeExternal.follow(intake);
    intakeExternal.setIdleMode(IdleMode.kCoast);
    intakeExternalLift.setIdleMode(IdleMode.kBrake);
    intakeExternalLift.setSmartCurrentLimit(10);

    flywheelShaftEncoder.setDistancePerPulse(1. / 2048.);
    flywheelShaftEncoder.setSamplesToAverage(4);

    // Default to auto transfer balls from intake using beam breaks
    setDefaultCommand(new RunCommand(this::digestBalls, this));

    // Set up sim values
    if (RobotBase.isSimulation()) {

      flywheelSimulator =
          new FlywheelSim(
              Shooter.kFlywheelPlant, Shooter.kFlywheelGearbox, Shooter.kFlywheelGearing);

      flywheelEncoderSim = new EncoderSim(flywheelShaftEncoder);
    }
  }

  // Digestive system commands

  public Command getShootCommand(DoubleSupplier RPM) {
    return new InstantCommand(() -> supplyFlywheelTargetSpeedRPM(RPM))
        // () -> flywheelTargetVelRPM))
        .andThen(new RunCommand(() -> setSpinUpFlywheelCustomFreq(true)));
  }

  public Command spinUpCommand(DoubleSupplier RPM) {
    return new InstantCommand(() -> supplyFlywheelTargetSpeedRPM(RPM))
        .andThen(new InstantCommand(() -> setSpinUpFlywheelCustomFreq(true)));
  }

  public Command getShooterPurgeCommand() {
    return new InstantCommand(() -> supplyFlywheelTargetSpeedRPM(() -> 500))
        .andThen(new RunCommand(() -> spinUpFlywheelToTargetRPM()));
  }

  public Command getIntakeDownCommand() {
    return new StartEndCommand(
            () -> setIntakeExternalLiftSpeedVolts(-5), () -> setIntakeExternalLiftSpeedVolts(-1))
        .alongWith(new InstantCommand(() -> setIntakeSpeedVolts(8)))
        .withTimeout(0.3);
  }

  public Command getIntakeUpCommand() {
    return new StartEndCommand(
            () -> setIntakeExternalLiftSpeedVolts(5), () -> setIntakeExternalLiftSpeedVolts(0.75))
        .alongWith(new InstantCommand(() -> setIntakeSpeedVolts(0)))
        .withTimeout(0.45);
  }

  public Command getIntakeCommand() {
    return new StartEndCommand(
            () -> getIntakeDownCommand().schedule(), () -> getIntakeUpCommand().schedule())
        .withInterrupt(() -> stomachFull);
  }

  // Flywheel control set-up methods

  // @Config
  public void setSpinUpFlywheelCustomFreq(boolean bool) {
    spinFlywheelCustomFreq = bool;
  }

  // @Config
  public void setFlywheelPID(double P, double I, double D) {
    flywheelPID.setPID(P, I, D);
  }

  // @Config
  public void setFlywheelTargetVelRPM(double RPM) {
    flywheelTargetVelRPM = RPM;
  }

  public void supplyFlywheelTargetSpeedRPM(DoubleSupplier RPMsource) {
    flywheelTargetVelRPM = RPMsource.getAsDouble();
  }

  // Methods to calculate control inputs

  public void spinUpFlywheelToTargetRPM() {

    // Calculate control values
    flywheelFFEffort = 0.95 * flywheelFF.calculate(flywheelTargetVelRPM / 60);
    flywheelPIDEffort = flywheelPID.calculate(getFlywheelVelRPM(), flywheelTargetVelRPM);
    flywheelTotalEffort = flywheelFFEffort + flywheelPIDEffort;

    // Use output
    setFlywheelVoltage(flywheelTotalEffort);

    updateSim();
  }

  public void spinUpCustomFreqFunc() {
    if (spinFlywheelCustomFreq) {
      spinUpFlywheelToTargetRPM();
    }
  }

  private void digestBalls() {
    setTransferSpeedProp(ballInMouth && !stomachFull ? 0.5 : 0);
  }

  // Check whether flywheel is within tolerance of setpoint

  @Log
  public boolean flywheelAtTargetVelRPM() {
    if (Math.abs(flywheelVelRPM - flywheelTargetVelRPM) < 100 /* && flywheelAccelRPMPerS < 30*/) {
      return true;
    } else {
      return false;
    }
  }

  // Proportional control methods

  public void setIntakeSpeedProp(double prop) {
    intake.set(prop);
    intakeSpeedVolts = prop * 12;
  }

  public void setIntakeExternalSpeedProp(double prop) {
    intakeExternal.set(prop);
    intakeExternalSpeedVolts = prop * 12;
  }

  public void setIntakeExternalLiftSpeedProp(double prop) {
    intakeExternalLift.set(prop);
    intakeExternalLiftSpeedVolts = prop * 12;
  }

  public void setTransferSpeedProp(double prop) {
    transfer.set(prop);
    transferSpeedVolts = prop * 12;
  }

  public void setFlywheelSpeedProp(double speed) {
    flywheelL.set(speed);
  }

  // Voltage control methods

  public void setIntakeSpeedVolts(double volts) {
    intake.setVoltage(volts);
    intakeSpeedVolts = volts;
  }

  public void setIntakeExternalSpeedVolts(double volts) {
    intakeExternal.setVoltage(volts);
    intakeExternalSpeedVolts = volts;
  }

  public void setIntakeExternalLiftSpeedVolts(double volts) {
    intakeExternalLift.setVoltage(volts);
    intakeExternalLiftSpeedVolts = volts;
  }

  public void setTransferSpeedVolts(double volts) {
    transfer.setVoltage(volts);
    transferSpeedVolts = volts;
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelVoltage = voltage;
    flywheelL.setVoltage(voltage);
    flywheelR.setVoltage(-voltage);
  }

  // Measurement methods

  @Log
  public double getFlywheelVelRPM() {
    return flywheelShaftEncoder.getRate() * 60;
  }

  @Log
  public double getFlywheelPosition() {
    return flywheelShaftEncoder.getDistance();
  }

  private void updateSim() {
    // Set sim inputs
    flywheelSimulator.setInput(flywheelVoltage);
    flywheelSimulator.update(0.020);

    // Calculate sim values
    flywheelEncoderSim.setRate(flywheelSimulator.getAngularVelocityRPM() / 60);

    // Update encoder after sim value is set
    flywheelVelRPMShaftEnc = flywheelShaftEncoder.getRate();
  }

  // Periodic functions

  @Override
  public void periodic() {
    intakeExternalLiftCurrent = intakeExternalLift.getOutputCurrent();

    // Calculate flywheel measurements
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();
    flywheelVelRPM = flywheelMotorEncoder.getVelocity();
    flywheelVelRPMShaftEnc = getFlywheelVelRPM();
    flywheelPositionShaftEnc = getFlywheelPosition();
    flywheelAccelRPMPerS = accelFilter.calculate((flywheelVelRPM - lastFlywheelVelRPM) / 0.02);
    lastFlywheelVelRPM = flywheelVelRPM;
  }

  @Override
  public void simulationPeriodic() {

  }
}
