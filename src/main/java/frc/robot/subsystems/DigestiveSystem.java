// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DigestiveSystem extends SubsystemBase {

  private final CANSparkMax intake = new CANSparkMax(CAN.intake_ID, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer_ID, MotorType.kBrushless);
  private final CANSparkMax flywheel = new CANSparkMax(CAN.flywheel_ID, MotorType.kBrushless);

  private final RelativeEncoder intake_ENC;
  private final RelativeEncoder transfer_ENC;
  private final RelativeEncoder flywheel_ENC;

  

  /** Creates a new Intake. */
  public DigestiveSystem() {

    intake.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheel.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    transfer.setIdleMode(IdleMode.kCoast);
    flywheel.setIdleMode(IdleMode.kCoast);

    intake_ENC = intake.getEncoder();
    transfer_ENC = transfer.getEncoder();
    flywheel_ENC  = flywheel.getEncoder();

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
