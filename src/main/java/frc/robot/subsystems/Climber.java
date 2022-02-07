// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;

public class Climber extends SubsystemBase {

  private final CANSparkMax ClimberR = new CANSparkMax(CAN.climberL, MotorType.kBrushless);
  private final CANSparkMax ClimberL = new CANSparkMax(CAN.climberR, MotorType.kBrushless);

  private final RelativeEncoder ClimberR_ENC;
  private final RelativeEncoder ClimberL_ENC;

  private final DigitalInput bottomLimit = new DigitalInput(DIO.breakBeamA);
  private final DigitalInput topLimit = new DigitalInput(DIO.breakBeamB);

  /** Creates a new Climber. */
  public Climber() {

    ClimberR.restoreFactoryDefaults();
    ClimberL.restoreFactoryDefaults();

    ClimberL_ENC = ClimberL.getEncoder();
    ClimberR_ENC = ClimberR.getEncoder();
    // ClimberR.setInverted(true);

  }

  public void setPropL(double pwr) {
    ClimberL.set(pwr);
  }

  public void setPropR(double pwr) {
    ClimberR.set(pwr);
  }

  public void climb(double pwr) {
    ClimberL.set(pwr);
    ClimberR.set(pwr);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
