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
    transfer.setIdleMode(IdleMode.kCoast);
    flywheelL.setIdleMode(IdleMode.kCoast);
    flywheelR.setIdleMode(IdleMode.kCoast);

    flywheelL.setInverted(true);
    flywheelR.follow(flywheelL, true);

    this.setDefaultCommand(new RunCommand(this::digestBalls, this));

    if (RobotBase.isSimulation()) {

      flywheelSimulator =
          new FlywheelSim(
              Shooter.kFlywheelPlant, Shooter.kFlywheelGearbox, Shooter.kFlywheelGearing);

      flywheelEncoderSim = new RelativeEncoderSim(false, CAN.flywheelL);
    }
  }

  @Config
  public void setIntakeSpeedProp(double prop) {
    intake.set(prop);
  }

  @Config
  public void setTransferSpeedProp(double prop) {
    transfer.set(prop);
  }

  @Config
  public void setFlywheelTargetSpeedRPM(double RPM) {
    flywheelTargetVelRPM = RPM;
  }

  private void digestBalls() {
    ballInMouth = !breakBeamMouth.get();
    stomachFull = !breakBeamStomach.get();
    transfer.set(ballInMouth && !stomachFull ? .2 : 0);
  }

  private void shoot() {
    if (Math.abs(flywheelEncoder.getVelocity() - Shooter.targetRPM) < 10) {
      transfer.set(.2);
    }
  }

  public Command getShootCommand() {
    return new RunCommand(this::shoot)
                          .beforeStarting(() -> setFlywheelTargetSpeedRPM(Shooter.targetRPM))
                          .andThen(() -> setFlywheelTargetSpeedRPM(0));
  }

  public Command getIntakeCommand() {
    return new StartEndCommand(() -> intake.set(0.5), () -> intake.set(0));
  }

  @Override
  public void periodic() {

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

    flywheelVelRPM = flywheelEncoder.getVelocity();

    flywheelTargetVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTargetVelRPM);
    flywheelVelRadPS = Units.rotationsPerMinuteToRadiansPerSecond(flywheelVelRPM);

    flywheelFFEffort = feedForward.calculate(flywheelTargetVelRadPS);
    flywheelBBEffort = flywheelBangBang.calculate(flywheelVelRadPS, flywheelTargetVelRadPS);
    flywheelTotalEffort = flywheelFFEffort + flywheelBBEffort * 0.25;
    flywheelL.setVoltage(flywheelTotalEffort);

    ////////////setIntakeSpeedProp(testController.getLeftTriggerAxis());
    ////////////setTransferSpeedProp(testController.getRightTriggerAxis());
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimulator.setInput(flywheelL.getAppliedOutput());
    flywheelSimulator.update(0.020);

    flywheelEncoderSim.setVelocity(flywheelSimulator.getAngularVelocityRPM());
  }
}
