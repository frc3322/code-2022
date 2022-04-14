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
            double speed = MathUtil.applyDeadband(-driverController.getLeftY(), 0.09);
            double turn = MathUtil.applyDeadband(driverController.getRightX(), 0.08);

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
    autonChooser.addOption("Four Ball", new AltFourBallAuto());
    autonChooser.addOption("One Ball (Positionable)", new OneBallPositionableAuto());

    SmartDashboard.putData("Select Autonomous", autonChooser);

    // Trajectories

    // Four ball
    // drivetrain.putTrajOnFieldWidget(Trajectories.FourBall.initToWallToShoot, "Init to Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.FourBall.ShootToHPS, "Shoot To HPS");
    // drivetrain.putTrajOnFieldWidget(Trajectories.FourBall.HPStoFarShoot, "HPS To Far Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.FiveBall.slightForward, "Slight Forward");

    // Alt Four ball
    drivetrain.putTrajOnFieldWidget(Trajectories.altFourBall.initToFirstBall, "init to first ball");
    drivetrain.putTrajOnFieldWidget(Trajectories.altFourBall.firstBallToHPS, "first ball to HPS");
    drivetrain.putTrajOnFieldWidget(Trajectories.altFourBall.HPStoShoot, "hps to shoot");
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
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.09),
                        () -> MathUtil.applyDeadband(secondController.getLeftY(), 0.09))))
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
                            Trajectories.FourBall.initPose.getRotation().getDegrees()))
                .andThen(() -> drivetrain.resetOdometry(Trajectories.FourBall.initPose)));

    secondController.upperPOV().whenPressed(() -> ShooterParams.offsetShootTunings(50));

    secondController.lowerPOV().whenPressed(() -> ShooterParams.offsetShootTunings(-50));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.2)
        .whileActiveOnce(new InstantCommand(() -> digestiveSystem.setIntakeSpeedVolts(8)))
        .whenInactive(new InstantCommand(() -> digestiveSystem.setIntakeSpeedVolts(0)));

    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2)
        .whileActiveOnce(new ShootCommand(ShooterParams.getRPMFromDistanceMeters(3), true));
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
                    && digestiveSystem.flywheelAtTargetVelRPM()
                    && drivetrain.getLimelightHasTarget()));

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
          digestiveSystem.getIntakeDownCommand(),
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.straightForward.getInitialPose().getRotation().getDegrees())),
          new InstantCommand(
              () -> drivetrain.resetOdometry(Trajectories.straightForward.getInitialPose())),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.straightForward),
          digestiveSystem.getIntakeUpCommand(),
          drivetrain.profiledTurnToAngleCommand(() -> 180).withTimeout(1.5),
          getAutoShootCommand(2, true));
    }
  }

  private class OneBallPositionableAuto extends SequentialCommandGroup {
    private OneBallPositionableAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.straightBackward.getInitialPose().getRotation().getDegrees())),
          new InstantCommand(
              () -> drivetrain.resetOdometry(Trajectories.straightBackward.getInitialPose())),
          getAutoShootCommand(2, true),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.straightBackward));
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
                  drivetrain.resetGyro(Trajectories.FourBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.FourBall.initPose)),
          digestiveSystem.getSpinUpCommand(() -> 1900.0),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FourBall.initToWallToShoot)
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
              .getRamseteCommand(drivetrain, Trajectories.FourBall.ShootToHPS)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          // new WaitCommand(1),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FourBall.HPStoFarShoot)
              .alongWith(digestiveSystem.getSpinUpCommand(() -> 2400.0)),
          drivetrain.profiledTurnToAngleCommand(() -> 195).withTimeout(2),
          getAutoShootCommand(2, true).alongWith(digestiveSystem.getIntakeUpCommand()));
    }
  }

  private class AltFourBallAuto extends SequentialCommandGroup {

    private AltFourBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.altFourBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.altFourBall.initPose)),
          digestiveSystem.getSpinUpCommand(() -> 1900.0),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.altFourBall.initToFirstBall)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          drivetrain.profiledTurnToAngleCommand(() -> 204),
          getAutoShootCommand(1.5, false).alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain.profiledTurnToAngleCommand(() -> 24),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.altFourBall.firstBallToHPS)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.altFourBall.slightBackward),
          // new WaitCommand(1),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.altFourBall.HPStoShoot)
              .alongWith(digestiveSystem.getSpinUpCommand(() -> 2400.0)),
          drivetrain.profiledTurnToAngleCommand(() -> 208).withTimeout(2),
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
