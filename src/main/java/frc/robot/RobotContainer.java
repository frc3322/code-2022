// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final DigestiveSystem digestiveSystem = new DigestiveSystem();

  private final Trajectories trajectories =
      new Trajectories(drivetrain.getTrajConfig(false), drivetrain.getTrajConfig(true));

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(1);

  private final Trigger alignedAndSped =
      new Trigger(
          () ->
              (drivetrain.getTurnToAngleAtSetpoint() && digestiveSystem.flywheelAtTargetVelRPM()));

  private final Command driveCommand =
      new RunCommand(
          () -> drivetrain.arcadeDrive(driverController.getLeftY(), -driverController.getRightX()),
          drivetrain);

  private final Command feedCommand =
      new StartEndCommand(
          () -> digestiveSystem.setTransferSpeedProp(0.5),
          () -> digestiveSystem.setTransferSpeedProp(0));

  private final Command waitUntilAlignedAndSpedCommand = new WaitUntilCommand(alignedAndSped);

  private final Command shootCommand =
      digestiveSystem
          .getShootCommand(() -> drivetrain.getLimelightAngleY())
          .alongWith(drivetrain.getTurnToLimelightCommand())
          .alongWith(waitUntilAlignedAndSpedCommand.andThen(feedCommand));

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveCommand);
  }

  private void configureButtonBindings() {

    // driverController
    //     .a()
    //     .whenHeld(
    //         digestiveSystem.getShootCommand().alongWith(drivetrain.getTurnToLimelightCommand()))
    //     .whenReleased(() -> digestiveSystem.setFlywheelSpeedProp(0))
    //     .and(alignedAndSped)
    //     .whileActiveOnce(
    //         new StartEndCommand(
    //             () -> digestiveSystem.setTransferSpeedProp(0.5),
    //             () -> digestiveSystem.setTransferSpeedProp(0),
    //             digestiveSystem));

    driverController
        .a()
        .whenHeld(shootCommand)
        .whenReleased(() -> digestiveSystem.setFlywheelVoltage(0));

    driverController.rightBumper().whenHeld(digestiveSystem.getIntakeCommand());
    driverController.y().whenPressed(() -> drivetrain.zeroPoseAndSensors());
  }

  public Command getAutonomousCommand() {
    return drivetrain.getRamseteCommand(drivetrain, trajectories.example);
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
