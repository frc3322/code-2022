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
import frc.robot.CommandXboxController;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Shooter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DigestiveSystem extends SubsystemBase implements Loggable {

  private final CANSparkMax intake = new CANSparkMax(CAN.intake, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer, MotorType.kBrushless);
  private final CANSparkMax flywheel1 = new CANSparkMax(CAN.flywheelL, MotorType.kBrushless);
  private final CANSparkMax flywheel2 = new CANSparkMax(CAN.flywheelR, MotorType.kBrushless);

  private final RelativeEncoder intake_ENC = intake.getEncoder();
  private final RelativeEncoder transfer_ENC = transfer.getEncoder();
  private final RelativeEncoder flywheel_ENC = flywheel1.getEncoder();

  private boolean beam1Broken = false;
  private boolean beam2Broken = false;

  PIDController flywheelPID = new PIDController(0.1, 0, 0);
  BangBangController flywheelBangBang = new BangBangController();

  @Config private double flywheelTargetSpeedRPM;
  private double flywheelTargetSpeedRPS;
  private double flywheelSpeedRPS;
  private double flywheelFFEffort;
  private double flywheelBBEffort;
  @Log private double flywheelTotalEffort;

  SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Shooter.ksVolts, Shooter.kvVoltSecondsPerRotation);

  private final CommandXboxController testController = new CommandXboxController(0);

  public DigestiveSystem() {

    intake.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheel1.restoreFactoryDefaults();
    flywheel2.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    transfer.setIdleMode(IdleMode.kCoast);
    flywheel1.setIdleMode(IdleMode.kCoast);
    flywheel2.setIdleMode(IdleMode.kCoast);

    flywheel1.setInverted(true);
    flywheel2.follow(flywheel1, true);

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
    flywheelTargetSpeedRPM = RPM;
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

    flywheelTargetSpeedRPS = flywheelTargetSpeedRPM / 60;
    flywheelSpeedRPS = flywheel_ENC.getVelocity() / 60;

    flywheelFFEffort = feedForward.calculate(flywheelTargetSpeedRPS);
    flywheelBBEffort = flywheelBangBang.calculate(flywheelSpeedRPS, flywheelTargetSpeedRPS);
    flywheelTotalEffort = flywheelFFEffort + flywheelBBEffort * 0.25;
    flywheel1.setVoltage(flywheelTotalEffort);

    setIntakeSpeedProp(testController.getLeftTriggerAxis());
    setTransferSpeedProp(testController.getRightTriggerAxis());
  }
}
