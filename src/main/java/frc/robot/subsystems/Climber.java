// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase implements Loggable {

  private final CANSparkMax ClimberR = new CANSparkMax(CAN.climberL, MotorType.kBrushless);
  private final CANSparkMax ClimberL = new CANSparkMax(CAN.climberR, MotorType.kBrushless);

  private final RelativeEncoder ClimberR_ENC;
  private final RelativeEncoder ClimberL_ENC;

  @Log private double lEncVal;
  @Log private double rEncVal;
  @Log private double propL;
  @Log private double propR;

  // private final DigitalInput bottomLimit = new DigitalInput(DIO.breakBeamA);
  // private final DigitalInput topLimit = new DigitalInput(DIO.breakBeamB);

  /** Creates a new Climber. */
  public Climber() {

    ClimberR.restoreFactoryDefaults();
    ClimberL.restoreFactoryDefaults();
    ClimberL.setIdleMode(IdleMode.kBrake);
    ClimberR.setIdleMode(IdleMode.kBrake);

    ClimberL.setInverted(true);
    // ClimberR.follow(ClimberL, true);

    ClimberL_ENC = ClimberL.getEncoder();
    // ClimberL_ENC.setPosition(0);
    ClimberR_ENC = ClimberR.getEncoder();
    // ClimberL_ENC.setPosition(0);

    // ClimberR.setInverted(true);

  }

  public void setPropL(double pwr) {
    ClimberL.set(pwr);
    propL = pwr;
  }

  public void setPropR(double pwr) {
    ClimberR.set(pwr);
    propR = pwr;
  }

  public void climb(double pwr) {
    ClimberL.set(pwr);
    ClimberR.set(pwr);
  }

  public void supplyClimbInputs(DoubleSupplier pwrL, DoubleSupplier pwrR) {
    setPropL(pwrL.getAsDouble());
    setPropR(pwrR.getAsDouble());
  }

  @Override
  public void periodic() {
    lEncVal = ClimberL_ENC.getPosition();
    rEncVal = ClimberR_ENC.getPosition();
    // This method will be called once per scheduler run
  }
}
