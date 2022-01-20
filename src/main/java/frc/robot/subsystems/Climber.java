// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {

  private final CANSparkMax ClimberR = new CANSparkMax(CAN.ClimberR_ID, MotorType.kBrushless);
  private final CANSparkMax ClimberL = new CANSparkMax(CAN.ClimberL_ID, MotorType.kBrushless);

  private final RelativeEncoder ClimberR_ENC;
  private final RelativeEncoder ClimberL_ENC;

  /** Creates a new Climber. */
  public Climber() {

    ClimberR.restoreFactoryDefaults();
    ClimberL.restoreFactoryDefaults();

    ClimberL_ENC = ClimberL.getEncoder();
    ClimberR_ENC  = ClimberR.getEncoder();
    //ClimberR.setInverted(true);

  }


public void climb(double pwr){
 ClimberL.set(pwr);
 ClimberR.set(pwr);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
