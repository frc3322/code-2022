// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final Command driveCommand =
      new RunCommand(
          () -> drivetrain.arcadeDrive(driverController.getLeftY(), -driverController.getRightX()),
          drivetrain);

  private class ShootCommand extends ParallelCommandGroup {
      private final Trigger alignedAndSped =
      new Trigger(
          () ->
              (drivetrain.getTurnToAngleAtSetpoint() && digestiveSystem.flywheelAtTargetVelRPM()));

      private final Command feedCommand =
        new StartEndCommand(
            () -> digestiveSystem.setTransferSpeedProp(0.5),
            () -> digestiveSystem.setTransferSpeedProp(0), digestiveSystem);

      private final Command waitUntilAlignedAndSpedCommand = new WaitUntilCommand(alignedAndSped);

      private ShootCommand() {
        addCommands(
          digestiveSystem.getShootCommand(() -> drivetrain.getLimelightAngleY()),
          drivetrain.getTurnToLimelightCommand(),
          waitUntilAlignedAndSpedCommand.andThen(feedCommand)
        );
      }
  }

  public RobotContainer() {

    SmartDashboard.putData("Shooot", (Sendable) digestiveSystem
    .getShootCommand(() -> drivetrain.getLimelightAngleY()));

    SmartDashboard.putData("TurntoAngleProfiledTestnottheotherone", (Sendable) drivetrain.profiledTurnToAngleCommand(() -> 193));

    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveCommand);
  }

  private void configureButtonBindings() {

    driverController
        .a()
        .whenHeld(new ShootCommand())
        .whenReleased(() -> digestiveSystem.setFlywheelVoltage(0));

    driverController.rightBumper().whenHeld(digestiveSystem.getIntakeCommand());
    driverController.y().whenPressed(() -> drivetrain.zeroOdometry());
  }

  public Command getAutonomousCommand() {

    ShootCommand shootCommand = new ShootCommand();
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.zeroOdometry())
        .alongWith(new InstantCommand(() -> drivetrain.resetGyro())),
      new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
      drivetrain.getRamseteCommand(drivetrain, trajectories.tarmacToBall),
      new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0)),
      drivetrain.profiledTurnToAngleCommand(() -> -167),
      new InstantCommand(() -> shootCommand.schedule()),
      new WaitCommand(5),
      new InstantCommand(() -> shootCommand.cancel())
    );
    // new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)).andThen(new InstantCommand(() -> drivetrain.zeroOdometry())).andThen(drivetrain.getRamseteCommand(drivetrain, trajectories.tarmacToBall)
    // ).andThen(drivetrain.profiledTurnToAngleCommand(() -> -167)).andThen(new ShootCommand().withTimeout(5));
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
