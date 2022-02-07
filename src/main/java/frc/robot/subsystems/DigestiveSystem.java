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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Shooter;
import frc.robot.RelativeEncoderSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

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

  PIDController flywheelPID = new PIDController(0.1, 0, 0);
  BangBangController flywheelBangBang = new BangBangController();

  @Config private double flywheelTargetVelRPM;
  @Log private double flywheelVelRPM;
  private double flywheelTargetVelRadPS;
  private double flywheelVelRadPS;
  private double flywheelFFEffort;
  private double flywheelBBEffort;
  @Log private double flywheelTotalEffort;

  @Log private double intakeSpeedProp;
  @Log private double transferSpeedProp;

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

  public void setIntakeSpeedProp(double prop) {
    intake.set(prop);
    intakeSpeedProp = prop;
  }

  @Config
  public void setTransferSpeedProp(double prop) {
    transfer.set(prop);
    transferSpeedProp = prop;
  }

  public void setFlywheelTargetSpeedRPM(double RPM) {
    flywheelTargetVelRPM = RPM;
  }

  // @Config
  // private void setStomachFull(boolean in) {
  //   stomachFull = in;
  // }

  // @Config
  // private void setBallInMouth(boolean in) {
  //   ballInMouth = in;
  // }

  private void digestBalls() {
    setTransferSpeedProp(ballInMouth && !stomachFull ? 0.5 : 0);
  }

  private void shoot() {
    if (Math.abs(flywheelEncoder.getVelocity() - Shooter.targetRPM) < 100) {
      setTransferSpeedProp(0.5);
    }
  }

  @Config
  private void setFlywheelSpeedProp(double speed) {
    flywheelL.set(speed);
  }

  public Command getShootCommand() {
    return new RunCommand(this::shoot, this)
        .beforeStarting(() -> setFlywheelTargetSpeedRPM(Shooter.targetRPM));
  }

  public Command getIntakeCommand() {
    return new StartEndCommand(() -> setIntakeSpeedProp(0.5), () -> setIntakeSpeedProp(0))
        .withInterrupt(() -> stomachFull);
  }

  @Override
  public void periodic() {
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();

    flywheelVelRPM = flywheelEncoder.getVelocity();

    flywheelTargetVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTargetVelRPM);
    flywheelVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelVelRPM);

    flywheelFFEffort = feedForward.calculate(flywheelTargetVelRadPS);
    flywheelBBEffort = 0; // flywheelBangBang.calculate(flywheelVelRadPS, flywheelTargetVelRadPS);
    flywheelTotalEffort = flywheelFFEffort; // + flywheelBBEffort * 0.25;
    flywheelL.setVoltage(flywheelTotalEffort);

    // setIntakeSpeedProp(testController.getLeftTriggerAxis());
    // setTransferSpeedProp(testController.getRightTriggerAxis());
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimulator.setInput(flywheelL.getAppliedOutput());
    flywheelSimulator.update(0.020);

    flywheelEncoderSim.setVelocity(flywheelSimulator.getAngularVelocityRPM());
  }
}
