// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  // Create subsystems
  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final DigestiveSystem digestiveSystem = new DigestiveSystem();

  // Create controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(1);

  // Create commands
  private final Command driveCommand =
      new RunCommand(
          () -> drivetrain.arcadeDrive(driverController.getLeftY(), -driverController.getRightX()),
          drivetrain);

  public RobotContainer() {

    // Configuration
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();

    // Default commands
    drivetrain.setDefaultCommand(driveCommand);

    // Trajectories
    drivetrain.putTrajOnFieldWidget(Trajectories.fourBallAuto.tarmacToBall, "Tarmac To Ball");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAuto.ballToHumanPlayer, "Ball To Human Player");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAuto.humanPlayerToShoot, "Human Player To Shoot");
  }

  private void configureButtonBindings() {

    driverController.a().whenHeld(new ShootCommand());
    driverController.rightBumper().whenHeld(digestiveSystem.getIntakeCommand());

    driverController
        .y()
        .whenPressed(
            new InstantCommand(() -> drivetrain.resetGyro())
                .andThen(() -> drivetrain.zeroOdometry()));

    driverController
        .x()
        .whenHeld(
            new StartEndCommand(
                () -> digestiveSystem.setIntakeSpeedProp(-0.7),
                () -> digestiveSystem.setIntakeSpeedProp(0),
                digestiveSystem));

    driverController
        .leftBumper()
        .whenHeld(digestiveSystem.getShooterPurgeCommand())
        .whenReleased(() -> digestiveSystem.setFlywheelVoltage(0));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.resetGyro()),
        new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.fourBallAuto.tarmacToBall.getInitialPose())),
        new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
        drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.tarmacToBall),
        new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0)),
        drivetrain.profiledTurnToAngleCommand(() -> -167),
        getAutoShootCommand(2),
        drivetrain.profiledTurnToAngleCommand(() -> -3.4),
        new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
        drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.ballToHumanPlayer),
        new WaitCommand(1),
        new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0)),
        drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.humanPlayerToShoot),
        drivetrain.profiledTurnToAngleCommand(() -> -180),
        getAutoShootCommand(2));
  }

  // Shoot commands

  private class ShootCommand extends ParallelCommandGroup {
    private final Trigger alignedAndSped =
        new Trigger(
            () ->
                (
                drivetrain.getTurnToAngleAtSetpoint()
                &&  digestiveSystem.flywheelAtTargetVelRPM()));

    private final Command feedCommand =
        new StartEndCommand(
            () -> digestiveSystem.setTransferSpeedProp(0.5),
            () -> digestiveSystem.setTransferSpeedProp(0),
            digestiveSystem);

    private final Command waitUntilAlignedAndSpedCommand = new WaitUntilCommand(alignedAndSped);

    private ShootCommand() {
      addCommands(
          digestiveSystem.getShootCommand(() -> drivetrain.getLimelightAngleY()),
          drivetrain.getTurnToLimelightCommand()/*.withInterrupt(alignedAndSped)*/,
          // new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
          waitUntilAlignedAndSpedCommand.andThen(() -> feedCommand.schedule()));
    }

    @Override
    public void end(boolean interrupted) {
      feedCommand.cancel();
      digestiveSystem.setSpinUpFlywheelCustomFreq(false);
      digestiveSystem.setFlywheelVoltage(0);
    }
  }

  public void spinUpFlywheelCustomFreq() {
    digestiveSystem.spinUpCustomFreqFunc();
  }

  private Command getAutoShootCommand(double duration) {
    ParallelRaceGroup autoShootCommand = new ShootCommand().withTimeout(duration);
    return autoShootCommand;
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
