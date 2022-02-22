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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Shooter;
import frc.robot.LerpLLYtoRPM;
import frc.robot.RelativeEncoderSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

public class DigestiveSystem extends SubsystemBase implements Loggable {

  // Create motors
  private final CANSparkMax intake = new CANSparkMax(CAN.intake, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer, MotorType.kBrushless);
  private final CANSparkMax flywheelL = new CANSparkMax(CAN.flywheelL, MotorType.kBrushless);
  private final CANSparkMax flywheelR = new CANSparkMax(CAN.flywheelR, MotorType.kBrushless);

  // Create encoders
  private final RelativeEncoder flywheelEncoder = flywheelL.getEncoder();

  // Create break beam sensors
  private final DigitalInput breakBeamMouth = new DigitalInput(DIO.breakBeamA);
  private final DigitalInput breakBeamStomach = new DigitalInput(DIO.breakBeamB);

  // Transfer states
  @Log private boolean ballInMouth = false;
  @Log private boolean stomachFull = false;

  PIDController flywheelPID = new PIDController(0.00009, 0, 0); // 0.0012

  SimpleMotorFeedforward flywheelFF =
      new SimpleMotorFeedforward(Shooter.ksVolts, Shooter.kvVoltSecondsPerRotation);

  // Flywheel sim
  private FlywheelSim flywheelSimulator;
  private RelativeEncoderSim flywheelEncoderSim;

  // Flywheel measurements
  @Log private double flywheelVelRPM;
  private double lastFlywheelVelRPM;
  @Log private double flywheelAccelRPMPerS;

  private LinearFilter accelFilter = LinearFilter.movingAverage(40);

  // Flywheel control inputs
  @Log private double flywheelTargetVelRPM;
  @Log private double flywheelFFEffort;
  @Log private double flywheelPIDEffort;
  @Log private double flywheelTotalEffort;
  @Log private double flywheelVoltage;

  // Intake and transfer control inputs
  @Log private double intakeSpeedProp;
  @Log private double transferSpeedProp;

  private final CommandXboxController testController = new CommandXboxController(0);

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

    flywheelR.follow(flywheelL, true);

    // Default to auto transfer balls from intake using beam breaks
    setDefaultCommand(new RunCommand(this::digestBalls, this));

    // Set up sim values
    if (RobotBase.isSimulation()) {

      flywheelSimulator =
          new FlywheelSim(
              Shooter.kFlywheelPlant, Shooter.kFlywheelGearbox, Shooter.kFlywheelGearing);

      flywheelEncoderSim = new RelativeEncoderSim(false, CAN.flywheelL);
    }
  }

  // Digestive system commands

  public Command getShootCommand(DoubleSupplier limelightAngleY) {
    return new InstantCommand(
            () ->
                supplyFlywheelTargetSpeedRPM(
                    () -> LerpLLYtoRPM.getRPMFromSupplier(limelightAngleY)))
        // () -> flywheelTargetVelRPM))
        .andThen(new RunCommand(() -> spinUpFlywheelToTargetRPM()));
  }

  public Command getShooterPurgeCommand() {
    return new InstantCommand(() -> supplyFlywheelTargetSpeedRPM(() -> 500))
        .andThen(new RunCommand(() -> spinUpFlywheelToTargetRPM()));
  }

  public Command getIntakeCommand() {
    return new StartEndCommand(() -> setIntakeSpeedProp(0.7), () -> setIntakeSpeedProp(0))
        .withInterrupt(() -> stomachFull);
  }

  // Flywheel control set-up methods

  public void setFlywheelPID(double P, double I, double D) {
    flywheelPID.setPID(P, I, D);
  }

  public void setFlywheelTargetVelRPM(double RPM) {
    flywheelTargetVelRPM = RPM;
  }

  public void supplyFlywheelTargetSpeedRPM(DoubleSupplier RPMsource) {
    flywheelTargetVelRPM = RPMsource.getAsDouble();
  }

  // Methods to calculate control inputs

  public void spinUpFlywheelToTargetRPM() {

    // Calculate control values
    flywheelFFEffort = flywheelFF.calculate(flywheelTargetVelRPM / 60);
    flywheelPIDEffort = flywheelPID.calculate(flywheelEncoder.getVelocity(), flywheelTargetVelRPM);
    flywheelTotalEffort = flywheelFFEffort + flywheelPIDEffort;

    // Use output
    setFlywheelVoltage(flywheelTotalEffort);
  }

  private void digestBalls() {
    setTransferSpeedProp(ballInMouth && !stomachFull ? 0.5 : 0);
  }

  // Check whether flywheel is within tolerance of setpoint

  @Log
  public boolean flywheelAtTargetVelRPM() {
    if (Math.abs(flywheelVelRPM - flywheelTargetVelRPM) < 100 && flywheelAccelRPMPerS < 30) {
      return true;
    } else {
      return false;
    }
  }

  // Proportional control methods

  public void setIntakeSpeedProp(double prop) {
    intake.set(prop);
    intakeSpeedProp = prop;
  }

  public void setTransferSpeedProp(double prop) {
    transfer.set(prop);
    transferSpeedProp = prop;
  }

  public void setFlywheelSpeedProp(double speed) {
    flywheelL.set(speed);
  }

  // Voltage control methods

  public void setFlywheelVoltage(double voltage) {
    flywheelVoltage = voltage;
    flywheelL.setVoltage(voltage);
  }

  // Periodic functions

  @Override
  public void periodic() {

    // Calculate flywheel measurements
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();
    flywheelVelRPM = flywheelEncoder.getVelocity();
    flywheelAccelRPMPerS = accelFilter.calculate((flywheelVelRPM - lastFlywheelVelRPM) / 0.02);
    lastFlywheelVelRPM = flywheelVelRPM;
  }

  @Override
  public void simulationPeriodic() {

    // Set sim inputs
    flywheelSimulator.setInput(flywheelVoltage);
    flywheelSimulator.update(0.020);

    // Calculate sim values
    flywheelEncoderSim.setVelocity(flywheelSimulator.getAngularVelocityRPM());

    // Update encoder after sim value is set
    flywheelVelRPM = flywheelEncoder.getVelocity();
  }
}
