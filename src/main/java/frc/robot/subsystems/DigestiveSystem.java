// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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




  PIDController flywheelPID = new PIDController(0.001, 0, 0);
  BangBangController flywheelBangBang = new BangBangController();

  private double flywheelTargetVelRPM = 5000;
  @Log private double flywheelVelRPM;
  private double flywheelTargetVelRadPS;
  private double flywheelVelRadPS;
  private double flywheelFFEffort;
  private double flywheelBBEffort;
  @Log private double flywheelPIDEffort;
  @Log private double flywheelTotalEffort;
  private double flywheelFFScalar = 1.0;
  private double flywheelBBScalar = 12;

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

  @Config
  public void setFlywheelTargetSpeedRPM(DoubleSupplier RPMsource) {
    flywheelTargetVelRPM = RPMsource.getAsDouble();
  }

  //@Config
  public void setFlywheelFFScalar(double prop) {
    flywheelFFScalar = prop;
  }

  //@Config
  public void setFlywheelBBScalar(double prop) {
    flywheelBBScalar = prop;
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
    if (Math.abs(flywheelEncoder.getVelocity() - flywheelTargetVelRPM) < 100) {
      setTransferSpeedProp(0.5);
    }

    flywheelTargetVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTargetVelRPM);
    flywheelVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelVelRPM);

    flywheelFFEffort = feedForward.calculate(flywheelTargetVelRadPS);
    flywheelBBEffort = flywheelBangBang.calculate(flywheelVelRadPS, flywheelTargetVelRadPS);

    flywheelPIDEffort = 0;

    if(flywheelEncoder.getVelocity() < flywheelTargetVelRPM) {
      flywheelPIDEffort = flywheelPID.calculate(flywheelEncoder.getVelocity(), flywheelTargetVelRPM);
    }

    flywheelTotalEffort = flywheelFFEffort + flywheelPIDEffort;
    flywheelL.setVoltage(flywheelTotalEffort);
  }

  //@Config
  public void setFlywheelSpeedProp(double speed) {
    flywheelL.set(speed);
  }

  public Command getShootCommand() {
    return new RunCommand(this::shoot, this);
  }

  public Command getSpinUpCommand(DoubleSupplier limelightAngleYSource) {
    return new InstantCommand(
      () -> setFlywheelTargetSpeedRPM(
      () -> ((-207.25) * Math.sqrt(limelightAngleYSource.getAsDouble() - 0.43) + 3698.91)
    )
)
.withInterrupt(() -> limelightAngleYSource.getAsDouble() < 0.43);
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
