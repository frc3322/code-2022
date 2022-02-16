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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Shooter;
import frc.robot.RelativeEncoderSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

public class DigestiveSystem extends SubsystemBase implements Loggable {

  private final CANSparkMax intake = new CANSparkMax(CAN.intake, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer, MotorType.kBrushless);
  private final CANSparkMax flywheelL = new CANSparkMax(CAN.flywheelL, MotorType.kBrushless);
  private final CANSparkMax flywheelR = new CANSparkMax(CAN.flywheelR, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intake.getEncoder();
  private final RelativeEncoder transferEncoder = transfer.getEncoder();
  private final RelativeEncoder flywheelEncoder = flywheelL.getEncoder();

  private final DigitalInput breakBeamMouth = new DigitalInput(DIO.breakBeamA);
  private final DigitalInput breakBeamStomach = new DigitalInput(DIO.breakBeamB);
  @Log private boolean ballInMouth = false;
  @Log private boolean stomachFull = false;

  PIDController flywheelPID = new PIDController(0.0012, 0, 0);
  BangBangController flywheelBangBang = new BangBangController();

  @Log private double flywheelTargetVelRPM;
  @Log private double flywheelVelRPM;
  private double lastFlywheelVelRPM;
  @Log private double flywheelAccelRPMPerS;
  private double flywheelTargetVelRadPS;
  private double flywheelVelRadPS;
  @Log private double flywheelFFEffort;
  private double flywheelBBEffort;
  @Log private double flywheelPIDEffort;
  @Log private double flywheelTotalEffort;
  private double flywheelFFScalar = 1.0;
  private double flywheelBBScalar = 12;
  @Log private double flywheelVoltage = 0;

  @Log private double intakeSpeedProp;
  @Log private double transferSpeedProp;

  @Log private double limelightAngleRPM;

  SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Shooter.ksVolts, Shooter.kvVoltSecondsPerRadian);

  private final CommandXboxController testController = new CommandXboxController(0);

  private FlywheelSim flywheelSimulator;
  private RelativeEncoderSim flywheelEncoderSim;

  public DigestiveSystem() {

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

    setDefaultCommand(new RunCommand(this::digestBalls, this));

    if (RobotBase.isSimulation()) {

      flywheelSimulator =
          new FlywheelSim(
              Shooter.kFlywheelPlant, Shooter.kFlywheelGearbox, Shooter.kFlywheelGearing);

      flywheelEncoderSim = new RelativeEncoderSim(false, CAN.flywheelL);
    }
  }

  // @Config
  public void setFlywheelPID(double P, double I, double D) {
    flywheelPID.setPID(P, I, D);
  }

  public void setIntakeSpeedProp(double prop) {
    intake.set(prop);
    intakeSpeedProp = prop;
  }

  public void setTransferSpeedProp(double prop) {
    transfer.set(prop);
    transferSpeedProp = prop;
  }

  private void digestBalls() {
    setTransferSpeedProp(ballInMouth && !stomachFull ? 0.5 : 0);
  }

  public double limelightAngleYtoRPM(DoubleSupplier angle) {
    limelightAngleRPM = (-207.25) * Math.sqrt(angle.getAsDouble() - 0.43) + 3698.91;
    return (-207.25) * Math.sqrt(angle.getAsDouble() - 0.43) + 3698.91;
  }

  public void supplyFlywheelTargetSpeedRPM(DoubleSupplier RPMsource) {
    flywheelTargetVelRPM = RPMsource.getAsDouble();
  }

  public void spinUpFlywheelToTargetRPM() {
    flywheelTargetVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTargetVelRPM);
    flywheelVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelVelRPM);

    flywheelFFEffort = feedForward.calculate(flywheelTargetVelRadPS);

    flywheelPIDEffort = 0;

    if (flywheelVelRPM < flywheelTargetVelRPM) {
      flywheelPIDEffort =
          flywheelPID.calculate(flywheelEncoder.getVelocity(), flywheelTargetVelRPM);
    }

    flywheelTotalEffort = flywheelFFEffort + flywheelPIDEffort;
    setFlywheelVoltage(flywheelTotalEffort);
  }

  @Log
  public boolean flywheelAtTargetVelRPM() {
    if (Math.abs(flywheelVelRPM - flywheelTargetVelRPM) < 100 && flywheelAccelRPMPerS < 20) {
      return true;
    } else {
      return false;
    }
  }

  public Command getShootCommand(DoubleSupplier limelightAngleY) {
    return new InstantCommand(
            () -> supplyFlywheelTargetSpeedRPM(() -> limelightAngleYtoRPM(limelightAngleY)))
        .andThen(new RunCommand(() -> spinUpFlywheelToTargetRPM()));
  }

  // @Config
  public void setFlywheelSpeedProp(double speed) {
    flywheelL.set(speed);
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelL.set(voltage);
    flywheelVoltage = voltage;
  }
  public Command getIntakeCommand() {
    return new StartEndCommand(() -> setIntakeSpeedProp(0.7), () -> setIntakeSpeedProp(0))
        .withInterrupt(() -> stomachFull);
  }

  @Override
  public void periodic() {
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();

    flywheelVelRPM = flywheelEncoder.getVelocity();

    flywheelAccelRPMPerS = (flywheelVelRPM - lastFlywheelVelRPM) / 0.02;
    lastFlywheelVelRPM = flywheelVelRPM;

    

    // spinUpFlywheelToTargetRPM();
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimulator.setInput(flywheelVoltage);
    flywheelSimulator.update(0.020);

    flywheelEncoderSim.setVelocity(flywheelSimulator.getAngularVelocityRPM());
  }
}
