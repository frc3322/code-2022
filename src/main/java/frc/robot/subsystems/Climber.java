// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase implements Loggable {

  private final CANSparkMax climberR = new CANSparkMax(CAN.climberL, MotorType.kBrushless);
  private final CANSparkMax climberL = new CANSparkMax(CAN.climberR, MotorType.kBrushless);

  private final CANSparkMax traverse = new CANSparkMax(CAN.traverse, MotorType.kBrushless);

  private final RelativeEncoder climberR_ENC;
  private final RelativeEncoder climberL_ENC;

  private final RelativeEncoder traverse_ENC;

  private final PIDController traversePID = new PIDController(0, 0, 0);

  private final double traverseUprightPos = 0.5;

  @Log private double lEncVal;
  @Log private double rEncVal;
  @Log private double tEncVal;
  @Log private double propL;
  @Log private double propR;
  @Log private double propT;

  // private final DigitalInput bottomLimit = new DigitalInput(DIO.breakBeamA);
  // private final DigitalInput topLimit = new DigitalInput(DIO.breakBeamB);

  /** Creates a new Climber. */
  public Climber() {

    climberR.restoreFactoryDefaults();
    climberL.restoreFactoryDefaults();
    traverse.restoreFactoryDefaults();
    climberL.setIdleMode(IdleMode.kBrake);
    climberR.setIdleMode(IdleMode.kBrake);
    traverse.setIdleMode(IdleMode.kBrake);

    climberL.setInverted(true);

    climberL_ENC = climberL.getEncoder();
    climberR_ENC = climberR.getEncoder();
    traverse_ENC = traverse.getEncoder();

    traversePID.disableContinuousInput();
  }

  @Config
  public void setTraversePID(double kp, double ki, double kd) {
    traversePID.setPID(kp, ki, kd);
  }

  public void resetEncoders() {
    climberL_ENC.setPosition(0);
    climberR_ENC.setPosition(0);
    traverse_ENC.setPosition(0);
  }

  public void setPropL(double pwr) {
    climberL.set(pwr);
    propL = pwr;
  }

  public void setPropR(double pwr) {
    climberR.set(pwr);
    propR = pwr;
  }

  public void setPropTraverse(double pwr) {
    traverse.set(pwr);
    propT = pwr;
  }

  public void climb(double pwr) {
    double lPwr = pwr;
    double rPwr = pwr;

    if (lEncVal > rEncVal) {
      rPwr += 0.1;
    } else if (rEncVal > lEncVal) {
      lPwr += 0.1;
    }

    setPropL(lPwr);
    setPropR(rPwr);
  }

  public void putTraverseUpright() {
    setPropTraverse(traversePID.calculate(tEncVal, traverseUprightPos));
  }

  public void pivotTraverse(double pwr) {
    traverse.set(pwr);
    propT = pwr;
  }

  public void supplyClimbInputs(DoubleSupplier pwrL, DoubleSupplier pwrR) {
    setPropL(pwrL.getAsDouble());
    setPropR(pwrR.getAsDouble());
  }

  @Override
  public void periodic() {
    lEncVal = climberL_ENC.getPosition();
    rEncVal = climberR_ENC.getPosition();
    tEncVal = traverse_ENC.getPosition();
    // This method will be called once per scheduler run
  }
}
