// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ProfiledPIDAngleCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final DigestiveSystem digestiveSystem = new DigestiveSystem();

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Command driveCommand =
      new RunCommand(
          () -> drivetrain.arcadeDrive(driverController.getLeftY(), driverController.getRightX()),
          drivetrain);

  private final ProfiledPIDAngleCommand turnToAngle = new ProfiledPIDAngleCommand(drivetrain, 90);

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveCommand);

    SmartDashboard.putData("Turn to Angle", turnToAngle);
  }

  private void configureButtonBindings() {
    driverController.x().whenHeld(digestiveSystem.getShootCommand());
    driverController.a().whenHeld(digestiveSystem.getIntakeCommand());
    driverController.y().whenHeld(turnToAngle);
  }

  public Command getAutonomousCommand() {
    return drivetrain.getRamseteCommand(
        drivetrain, drivetrain.getTrajFromFieldWidget("traj1", false));
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
