// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class LimelightAlign extends CommandBase {

  Drivetrain drivetrain;
  double currentAngle;
  double limeX;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  boolean aligned = false;

  /** Creates a new LimelightAlign. */
  public LimelightAlign(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setupDrivePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = drivetrain.getHeading();
    limeX  = tx.getDouble(0.0);
      
        double setpoint = (currentAngle - limeX);
  
       double pidOut = drivetrain.drivePID.calculate(drivetrain.getHeading(), setpoint);
        drivetrain.drive(0, -pidOut);
        if(limeX<1.5){
          aligned = true;
        }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aligned;
  }
}
