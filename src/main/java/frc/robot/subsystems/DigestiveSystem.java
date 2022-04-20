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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Shooter;
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
  private final CANSparkMax kicker = new CANSparkMax(CAN.kicker, MotorType.kBrushless);
  private final CANSparkMax intakeExternal =
      new CANSparkMax(CAN.extIntakeTurn, MotorType.kBrushless);
  private final CANSparkMax intakeExternalLift =
      new CANSparkMax(CAN.extIntakeLift, MotorType.kBrushless);

  // Create encoders
  private final RelativeEncoder flywheelMotorEncoder = flywheelL.getEncoder();
  private final Encoder flywheelShaftEncoder =
      new Encoder(4, 3 /*, false, Encoder.EncodingType.k4X*/);

  private final Encoder kickerShaftEncoder = new Encoder(6, 5);

  // Create break beam sensors
  private final DigitalInput breakBeamMouth = new DigitalInput(DIO.breakBeamA);
  private final DigitalInput breakBeamStomach = new DigitalInput(DIO.breakBeamB);

  // Controllers
  PIDController flywheelPID = new PIDController(Shooter.Flywheel.kPVel, 0, 0); // 0.00009 0.0012

  SimpleMotorFeedforward flywheelFF =
      new SimpleMotorFeedforward(
          Shooter.Flywheel.ksVolts, Shooter.Flywheel.kvVoltSecondsPerRotation);

  PIDController kickerPID = new PIDController(Shooter.Kicker.kPVel, 0, 0);

  SimpleMotorFeedforward kickerFF =
      new SimpleMotorFeedforward(Shooter.Kicker.ksVolts, Shooter.Kicker.kvVoltSecondsPerRotation);

  // Flywheel sim
  private FlywheelSim flywheelSimulator;
  private EncoderSim flywheelEncoderSim;

  // Flywheel measurements
  // @Log private double flywheelVelRPM;
  // private double lastFlywheelVelRPM;
  // @Log private double flywheelAccelRPMPerS;
  // @Log private double flywheelVelRPMShaftEnc;
  // @Log private double flywheelPositionShaftEnc;

  private LinearFilter accelFilter = LinearFilter.movingAverage(40);

  // Flywheel control inputs
  @Log private double flywheelTargetVelRPM;
  @Log private double flywheelFFEffort;
  @Log private double flywheelPIDEffort;
  @Log private double flywheelTotalEffort;
  @Log private double flywheelVoltage;

  // Kicker control inputs
  @Log private double kickerTargetVelRPM = Shooter.Kicker.kKickerVelRPM;
  @Log private double kickerFFEffort;
  @Log private double kickerPIDEffort;
  @Log private double kickerTotalEffort;
  @Log private double kickerVoltage;

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
    intakeExternal.restoreFactoryDefaults();
    intakeExternalLift.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheelL.restoreFactoryDefaults();
    flywheelR.restoreFactoryDefaults();
    kicker.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    intake.setInverted(true);
    transfer.setIdleMode(IdleMode.kBrake);
    flywheelL.setIdleMode(IdleMode.kCoast);
    flywheelR.setIdleMode(IdleMode.kCoast);
    kicker.setIdleMode(IdleMode.kCoast);
    intakeExternal.setIdleMode(IdleMode.kCoast);
    intakeExternal.setInverted(true);
    intakeExternalLift.setIdleMode(IdleMode.kBrake);
    intakeExternalLift.setSmartCurrentLimit(15);

    intake.burnFlash();
    transfer.burnFlash();
    flywheelL.burnFlash();
    flywheelR.burnFlash();
    kicker.burnFlash();
    intakeExternal.burnFlash();
    intakeExternalLift.burnFlash();

    // Config encoders
    flywheelShaftEncoder.setDistancePerPulse(1. / 2048.);
    flywheelShaftEncoder.setSamplesToAverage(4);

    kickerShaftEncoder.setDistancePerPulse(1. / 2048.);
    kickerShaftEncoder.setSamplesToAverage(4);

    // Default to auto transfer balls from intake using beam breaks
    setDefaultCommand(new RunCommand(this::digestBalls, this));

    LEDs.get().setCondition(LEDs.Modes.TRANSFER_FULL, () -> getStomachFull());

    // Set up sim values
    if (RobotBase.isSimulation()) {
      flywheelSimulator =
          new FlywheelSim(
              Shooter.Flywheel.kFlywheelPlant,
              Shooter.Flywheel.kGearbox,
              Shooter.Flywheel.kGearing);

      flywheelEncoderSim = new EncoderSim(flywheelShaftEncoder);
    }
  }

  // Digestive system commands

  public Command getShootCommand(DoubleSupplier RPM) {
    return new RunCommand(() -> supplyFlywheelTargetSpeedRPM(RPM))
        // () -> flywheelTargetVelRPM))
        .alongWith(new InstantCommand(() -> setSpinUpShooterCustomFreq(true)));
  }

  public Command getSpinUpCommand(DoubleSupplier RPM) {
    return new InstantCommand(() -> supplyFlywheelTargetSpeedRPM(RPM))
        .andThen(new InstantCommand(() -> setSpinUpShooterCustomFreq(true)));
  }

  public Command feedCommand() {
    return new StartEndCommand(
        () -> setTransferSpeedVolts(6), () -> setTransferSpeedVolts(0), this);
  }

  public Command lowGoalShootCommand() {
    return new StartEndCommand(
            () -> {
              setKickerVoltage(2); // 2
              setFlywheelVoltage(3); // 3.35
            },
            () -> {
              setKickerVoltage(0);
              setFlywheelVoltage(0);
            })
        .alongWith(new WaitCommand(0.5).andThen(feedCommand()));
  }

  // Only use in auton, up and down get reversed when cancelled while running
  public Command getIntakeDownCommand() {
    return new StartEndCommand(
            () -> {
              setIntakeExternalLiftSpeedVolts(-7);
              setIntakeExternalSpeedVolts(8);
            },
            () -> setIntakeExternalLiftSpeedVolts(-2.5))
        .alongWith(new InstantCommand(() -> setIntakeSpeedVolts(10)))
        .withTimeout(0.3);
  }

  public Command getIntakeUpCommand() {
    return new StartEndCommand(
            () -> {
              setIntakeExternalLiftSpeedVolts(4);
              setIntakeExternalSpeedVolts(0);
            },
            () -> setIntakeExternalLiftSpeedVolts(0))
        .alongWith(new InstantCommand(() -> setIntakeSpeedVolts(0)))
        .withTimeout(0.45);
  }

  // Use this for teleop
  public Command getIntakeCommand() {
    return new StartEndCommand(
        () -> {
          setIntakeSpeedVolts(8);
          setIntakeExternalSpeedVolts(8);
          new StartEndCommand(
                  () -> setIntakeExternalLiftSpeedVolts(-7),
                  () -> setIntakeExternalLiftSpeedVolts(-2.5))
              .withTimeout(0.45)
              .schedule();
        },
        () -> {
          setIntakeSpeedVolts(0);
          setIntakeExternalSpeedVolts(0);
          new StartEndCommand(
                  () -> setIntakeExternalLiftSpeedVolts(4),
                  () -> setIntakeExternalLiftSpeedVolts(0))
              .withTimeout(0.4)
              .schedule();
        });
  }

  public boolean getStomachFull() {
    return stomachFull;
  }

  // Flywheel control set-up methods

  public void setSpinUpShooterCustomFreq(boolean bool) {
    spinFlywheelCustomFreq = bool;
  }

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

  // Kicker control set-up methods

  // @Config
  public void setKickerPID(double P, double I, double D) {
    kickerPID.setPID(P, I, D);
  }

  // @Config
  public void setKickerTargetVelRPM(double RPM) {
    kickerTargetVelRPM = RPM;
  }

  // Methods to calculate control inputs

  public void spinUpShooterToTargetRPM() {

    // Calculate control values
    flywheelFFEffort =
        flywheelFF.calculate(flywheelTargetVelRPM / 60) * (Robot.isSimulation() ? 1 : 0.95);
    flywheelPIDEffort = flywheelPID.calculate(getFlywheelVelRPM(), flywheelTargetVelRPM);
    flywheelTotalEffort = flywheelFFEffort + flywheelPIDEffort;

    kickerFFEffort = kickerFF.calculate(kickerTargetVelRPM / 60);
    kickerPIDEffort = kickerPID.calculate(getKickerVelRPM(), kickerTargetVelRPM);
    kickerTotalEffort = kickerFFEffort + kickerPIDEffort;

    // Use output
    setFlywheelVoltage(flywheelTotalEffort);
    setKickerVoltage(kickerTotalEffort);

    if (Robot.isSimulation()) {
      updateSim();
    }
  }

  public void spinUpCustomFreqFunc() {
    if (spinFlywheelCustomFreq) {
      spinUpShooterToTargetRPM();
    }
  }

  private void digestBalls() {
    setTransferSpeedVolts(ballInMouth && !stomachFull ? 6 : 0);
  }

  // Check whether flywheel is within tolerance of setpoint

  @Log
  public boolean flywheelAtTargetVelRPM() {
    if (Math.abs(getFlywheelVelRPM() - flywheelTargetVelRPM)
        < 100 /* && flywheelAccelRPMPerS < 30*/) {
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

  @Config
  public void setFlywheelVoltage(double voltage) {
    flywheelVoltage = voltage;
    flywheelL.setVoltage(voltage);
    flywheelR.setVoltage(-voltage);
  }

  @Config
  public void setKickerVoltage(double voltage) {
    kickerVoltage = voltage;
    kicker.setVoltage(voltage);
  }

  // Measurement methods

  @Log
  public double getFlywheelVelRPM() {
    return flywheelShaftEncoder.getRate() * 60;
  }

  @Log
  public double getKickerVelRPM() {
    return kickerShaftEncoder.getRate() * 60;
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
  }

  // Periodic functions

  @Override
  public void periodic() {
    intakeExternalLiftCurrent = intakeExternalLift.getOutputCurrent();

    // Calculate flywheel measurements
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }
}
