// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import frc.robot.Constants.CAN;
import frc.robot.Constants.shooterFF;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DigestiveSystem extends SubsystemBase {

  private final CANSparkMax intake = new CANSparkMax(CAN.intake_ID, MotorType.kBrushless);
  private final CANSparkMax transfer = new CANSparkMax(CAN.transfer_ID, MotorType.kBrushless);
  private final CANSparkMax flywheel1 = new CANSparkMax(CAN.flywheel1_ID, MotorType.kBrushless);
  private final CANSparkMax flywheel2 = new CANSparkMax(CAN.flywheel2_ID, MotorType.kBrushless);

  private final RelativeEncoder intake_ENC;
  private final RelativeEncoder transfer_ENC;
  private final RelativeEncoder flywheel_ENC;

  private boolean beam1Broken = false;
  private boolean beam2Broken = false;

  SparkMaxPIDController shooterControl;
  BangBangController bangBang = new BangBangController();
  
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(shooterFF.Ks, shooterFF.Kv);

  /** Creates a new Intake. */
  public DigestiveSystem() {

    intake.restoreFactoryDefaults();
    transfer.restoreFactoryDefaults();
    flywheel1.restoreFactoryDefaults();
    flywheel2.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    transfer.setIdleMode(IdleMode.kCoast);
    flywheel1.setIdleMode(IdleMode.kCoast);
    flywheel2.setIdleMode(IdleMode.kCoast);

    intake_ENC = intake.getEncoder();
    transfer_ENC = transfer.getEncoder();
    flywheel_ENC  = flywheel1.getEncoder();

    SmartDashboard.putNumber("flywheel/Speed", 0);

    flywheel1.setInverted(true);
    flywheel2.follow(flywheel1, true);
  }

  //Shooter methods
  public void setupShooterPID(){

    shooterControl.setP(0);
    shooterControl.setI(0);
    shooterControl.setD(0);

  }

  public void setSetpoint(double setpoint) {
  
    shooterControl.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
}


public void setSpeed(double speed) {
  flywheel1.set(speed);
}

public void stop(){
  flywheel1.setVoltage(0);
}

//intake methods
public void takeIn(double pwr){
  intake.set(pwr);
}

  @Override
  public void periodic() {
    double targetSpeed = SmartDashboard.getNumber("flywheel/Speed", 0.0);

   // setSpeed(SmartDashboard.getNumber("flywheel/Speed", 0.0));
    SmartDashboard.putNumber("flywheel/Vel", flywheel_ENC.getVelocity());
    SmartDashboard.putNumber("flywheel/Pos", flywheel_ENC.getPosition());

/*
    // This method will be called once per scheduler run
    //run until beam1 not broken
    //don't run if beam 2 broken.
    if(beam2Broken||!beam1Broken){
      intake.set(0);
    }

    double targetSpeed = SmartDashboard.getNumber("flywheelSpeed", 0.0);
    if (flywheel_ENC.getVelocity() < targetSpeed) {
      flywheel.set(1);
    } else {
      flywheel.set(0);
    }
*/
    //getting values for ff from smart dashboard
    //flywheel.set(bangBang.calculate(flywheel_ENC.getVelocity(), targetSpeed) + 0.9 * feedForward.calculate(targetSpeed));
    flywheel1.set(feedForward.calculate(targetSpeed));

  }

}
