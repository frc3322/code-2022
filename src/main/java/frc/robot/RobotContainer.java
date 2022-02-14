// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final DigestiveSystem digestiveSystem = new DigestiveSystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(1);

  private final Command driveCommand =
      new RunCommand(
          () -> drivetrain.arcadeDrive(driverController.getLeftY(), -driverController.getRightX()),
          drivetrain);

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveCommand);
    climber.setDefaultCommand(
        new RunCommand(
            () -> climber.setPropL(testController.getRightY()),
            climber)); // very dangerous, horrible decision, pls change, eff u thor
  }

  private void configureButtonBindings() {
    driverController
        .a()
        .whenHeld(
            digestiveSystem
                .getSpinUpLLCommand(() -> drivetrain.getLimelightAngleY())
                .andThen(
                    digestiveSystem
                        .getShootCommand()
                        .alongWith(drivetrain.turnToLimelightCommand())))
        .whenReleased(() -> digestiveSystem.setFlywheelSpeedProp(0));
    driverController.rightBumper().whenHeld(digestiveSystem.getIntakeCommand());
    driverController.y().whenPressed(() -> drivetrain.resetPoseAndSensors());
  }

  public Command getAutonomousCommand() {
    return drivetrain.getRamseteCommand(
        drivetrain, drivetrain.getTrajFromFieldWidget("traj1", false));
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
