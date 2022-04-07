// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Trajectories.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  // Create subsystems
  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final DigestiveSystem digestiveSystem = new DigestiveSystem();

  // Create controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Create commands
  private final Command driveCommand =
      new RunCommand(
          () -> {
            double speed = MathUtil.applyDeadband(-driverController.getLeftY(), 0.07);
            double turn = MathUtil.applyDeadband(driverController.getRightX(), 0.07);

            drivetrain.drive(speed, turn);
          },
          drivetrain);

  private final Command traverseCommand =
      new RunCommand(
          () ->
              climber.pivotTraverse(
                  0.5 * secondController.getLeftTriggerAxis()
                      - 0.5 * secondController.getRightTriggerAxis()),
          climber);

  public RobotContainer() {

    // Configuration
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();

    // Default commands
    drivetrain.setDefaultCommand(driveCommand);
    climber.setDefaultCommand(traverseCommand);

    autonChooser.setDefaultOption("Two Ball (Positionable)", new TwoBallPositionableAuto());
    autonChooser.addOption("Four Ball Auto", new FourBallAuto());

    SmartDashboard.putData("Select Autonomous", autonChooser);

    // Trajectories

    // Five ball
    drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.initToWallToShoot, "Init to Shoot");
    drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.ShootToHPS, "Shoot To HPS");
    drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.HPStoFarShoot, "HPS To Far Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.slightForward, "Slight Forward");

    // drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.altInitToWallToShoot, "Alt Init to Wall
    // to Shoot");

    SmartDashboard.putNumber("Shooter Target RPM", 0);
  }

  private void configureButtonBindings() {

    driverController.rightBumper().whenHeld(new ShootCommand(true));
    driverController.a().whenHeld(digestiveSystem.lowGoalShootCommand());
    driverController
        .b()
        .whenHeld(new ShootCommand(() -> SmartDashboard.getNumber("Shooter Target RPM", 0), true));
    driverController
        .leftBumper()
        .and(new Trigger(() -> !digestiveSystem.getStomachFull()))
        .whileActiveOnce(digestiveSystem.getIntakeCommand());

    driverController
        .x()
        .whenHeld(
            new StartEndCommand(
                () -> {
                  digestiveSystem.setIntakeSpeedVolts(-8.5);
                  digestiveSystem.setTransferSpeedVolts(-6);
                },
                () -> {
                  digestiveSystem.setIntakeSpeedVolts(0);
                  digestiveSystem.setTransferSpeedVolts(0);
                },
                digestiveSystem));

    secondController
        .rightBumper()
        .whileActiveOnce(
            new RunCommand(
                () ->
                    climber.supplyClimbInputs(
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.07),
                        () -> MathUtil.applyDeadband(secondController.getLeftY(), 0.07))))
        .whenInactive(() -> climber.climb(0));

    secondController
        .y()
        .whenHeld(new RunCommand(() -> climber.climb(-0.5)))
        .whenReleased(() -> climber.climb(0));

    secondController
        .a()
        .whenHeld(new RunCommand(() -> climber.climb(0.75)))
        .whenReleased(() -> climber.climb(0));

    secondController
        .x()
        .toggleWhenPressed(
            new RunCommand(() -> climber.putTraverseUpright())
                .withInterrupt(
                    () ->
                        secondController.getLeftTriggerAxis() > 0.1
                            || secondController.getRightTriggerAxis() > 0.1));

    secondController
        .leftBumper()
        .whileActiveOnce(
            new RunCommand(
                () ->
                    climber.supplyClimbInputs(
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.07),
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.07))))
        .whenInactive(() -> climber.climb(0));

    driverController
        .y()
        .whenPressed(
            new InstantCommand(
                    () ->
                        drivetrain.resetGyro(
                            Trajectories.FiveBall.initPose.getRotation().getDegrees()))
                .andThen(() -> drivetrain.resetOdometry(Trajectories.FiveBall.initPose)));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  // Shoot commands

  private class ShootCommand extends ParallelCommandGroup {
    private final Trigger alignedAndSped =
        new Trigger(
            () ->
                (drivetrain.getTurnToAngleAtSetpoint()
                    && digestiveSystem.flywheelAtTargetVelRPM()));

    private final Trigger sped = new Trigger(() -> digestiveSystem.flywheelAtTargetVelRPM());

    private final Command waitUntilAlignedAndSpedCommand = new WaitUntilCommand(alignedAndSped);

    private final Command waitUntilSpedCommand = new WaitUntilCommand(sped);

    private final Command feedCommand = digestiveSystem.feedCommand();

    private Boolean stopShooterAtEnd;

    private ShootCommand(Boolean stopShooterAtEnd) {
      this.stopShooterAtEnd = stopShooterAtEnd;

      addCommands(
          digestiveSystem.getShootCommand(
              () ->
                  ShooterParams.getRPMFromDistanceMetersSupplier(
                      () -> drivetrain.getDistanceToGoalMeters())),
          drivetrain.getTurnToLimelightCommand(),
          waitUntilAlignedAndSpedCommand.andThen(() -> feedCommand.schedule()));
    }

    private ShootCommand(double RPM, Boolean stopShooterAtEnd) {
      this.stopShooterAtEnd = stopShooterAtEnd;

      addCommands(
          digestiveSystem.getShootCommand(() -> RPM),
          waitUntilSpedCommand.andThen(() -> feedCommand.schedule()));
    }

    private ShootCommand(DoubleSupplier RPMTargetSource, Boolean stopShooterAtEnd) {
      this.stopShooterAtEnd = stopShooterAtEnd;

      addCommands(
          digestiveSystem.getShootCommand(RPMTargetSource),
          waitUntilSpedCommand.andThen(() -> feedCommand.schedule()));
    }

    @Override
    public void end(boolean interrupted) {
      feedCommand.cancel();

      if (stopShooterAtEnd) {
        digestiveSystem.setSpinUpShooterCustomFreq(false);
        digestiveSystem.setFlywheelVoltage(0);
        digestiveSystem.setKickerVoltage(0);
      }
    }
  }

  public void spinUpFlywheelCustomFreq() {
    digestiveSystem.spinUpCustomFreqFunc();
  }

  private Command getAutoShootCommand(double duration, boolean stopShooterAtEnd) {

    return new ShootCommand(stopShooterAtEnd).withTimeout(duration);
  }

  // Auton commands

  private class TwoBallPositionableAuto extends SequentialCommandGroup {
    private TwoBallPositionableAuto() {
      addCommands(
          // new InstantCommand(() -> digestiveSystem.setIntakeSpeedVolts(8)),
          digestiveSystem.getIntakeDownCommand(),
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.straightForward.getInitialPose().getRotation().getDegrees())),
          new InstantCommand(
              () -> drivetrain.resetOdometry(Trajectories.straightForward.getInitialPose())),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.straightForward),
          digestiveSystem.getIntakeUpCommand(),
          drivetrain.profiledTurnToAngleCommand(() -> 170).withTimeout(1),
          getAutoShootCommand(8, true));
      // new InstantCommand(() -> digestiveSystem.setIntakeSpeedVolts(0)));

    }
  }

  /*TODO:
  Adjust 1st arc
  Straighten path from shoot to HPS and back
  drive faster, DONE, test on ground
  get real coords from field asap
  visit thors moms house immediately */

  private class FourBallAuto extends SequentialCommandGroup {
    private final Command feedCommand = digestiveSystem.feedCommand();

    private FourBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(Trajectories.FiveBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.FiveBall.initPose)),
          digestiveSystem.getSpinUpCommand(() -> 1900.0),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.initToWallToShoot)
              .alongWith(
                  digestiveSystem
                      .getIntakeDownCommand()
                      .andThen(new WaitCommand(3))
                      .andThen(digestiveSystem.getIntakeUpCommand())),
          drivetrain.profiledTurnToAngleCommand(() -> -148),
          getAutoShootCommand(1.5, false),
          // drivetrain
          //     .getRamseteCommand(drivetrain, Trajectories.FiveBall.slightForward)
          //     .alongWith(digestiveSystem.getIntakeDownCommand()),
          // getAutoShootCommand(1, true),
          //   .alongWith(
          //       new SequentialCommandGroup(
          //           new WaitCommand(0.15), digestiveSystem.getIntakeDownCommand())),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.ShootToHPS)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          // new WaitCommand(1),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.HPStoFarShoot)
              .alongWith(digestiveSystem.getSpinUpCommand(() -> 2400.0)),
          drivetrain.profiledTurnToAngleCommand(() -> 195).withTimeout(2),
          getAutoShootCommand(2, true).alongWith(digestiveSystem.getIntakeUpCommand()));
    }
  }

  // Logger

  public void updateLogger() {
    Logger.updateEntries();
  }

  // Reset climb encoders

  public void resetClimbEncoders() {
    climber.resetEncoders();
  }
}
