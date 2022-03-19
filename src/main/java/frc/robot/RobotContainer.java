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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

import java.time.Instant;
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
        () -> climber.pivotTraverse(secondController.getRightTriggerAxis() - secondController.getLeftTriggerAxis()), climber);

  public RobotContainer() {

    // Configuration
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();

    // Default commands
    drivetrain.setDefaultCommand(driveCommand);
    climber.setDefaultCommand(traverseCommand);

    autonChooser.setDefaultOption("Two Ball (Positionable)", new TwoBallPositionableAuto());
    autonChooser.addOption("Three Ball", new ThreeBallAuto());
    autonChooser.addOption("Four Ball", new FourBallAuto());
    autonChooser.addOption("Five Ball Auto", new FiveBallAuto());
    autonChooser.addOption("Alt Four Ball Auto", new AltFourBallAuto());

    SmartDashboard.putData("Select Autonomous", autonChooser);

    // Trajectories

    drivetrain.putTrajOnFieldWidget(
        Trajectories.FiveBall.ShootToHPS, "Shoot To HPS");

    SmartDashboard.putNumber("Shooter Target RPM", 0);
  }

  private void configureButtonBindings() {

    driverController.rightBumper().whenHeld(new ShootCommand(true));
    driverController.a().whenHeld(new ShootCommand(1250, true));
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
        .leftBumper()
        .and(secondController.rightBumper())
        .whileActiveOnce(
            new RunCommand(
                () ->
                    climber.supplyClimbInputs(
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.07),
                        () -> MathUtil.applyDeadband(secondController.getLeftY(), 0.07))))
        .whenInactive(() -> climber.climb(0));

    secondController
        .y()
        .whenPressed(() -> climber.climb(-0.5))
        .whenReleased(() -> climber.climb(0));

    secondController
        .a()
        .whenPressed(() -> climber.climb(0.75))
        .whenReleased(() -> climber.climb(0));

    testController
        .y()
        .whenPressed(
            new InstantCommand(
                    () ->
                        drivetrain.resetGyro(
                            Trajectories.RightSide.initPose.getRotation().getDegrees()))
                .andThen(() -> drivetrain.resetOdometry(Trajectories.RightSide.initPose)));

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
              () -> LerpLLYtoRPM.getRPMFromSupplier(() -> drivetrain.getLimelightAngleY())),
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

  private Command getAutoShootCommand(double duration, boolean useLimelight) {
    ParallelRaceGroup autoShootCommand;

    if (useLimelight) {
      autoShootCommand = new ShootCommand(true).withTimeout(duration);
    } else {
      autoShootCommand = new ShootCommand(2900, true).withTimeout(duration);
    }

    return autoShootCommand;
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

  private class ThreeBallAuto extends SequentialCommandGroup {
    private ThreeBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(Trajectories.RightSide.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.RightSide.initPose)),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.tarmacToShoot)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          getAutoShootCommand(2, true).alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.ThreeBallAuto.shootToWallBall)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.ThreeBallAuto.wallBallToShoot)
              .alongWith(digestiveSystem.getIntakeUpCommand()),
          getAutoShootCommand(2, true));
    }
  }

  private class FourBallAuto extends SequentialCommandGroup {
    private FourBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(Trajectories.RightSide.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.RightSide.initPose)),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.tarmacToShoot)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          getAutoShootCommand(2, true).alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.FourBallAuto.shootToHumanPlayer)
              .alongWith(new WaitCommand(4).andThen(digestiveSystem.getIntakeDownCommand())),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.RightSide.FourBallAuto.humanPlayerToShoot)
              .alongWith(digestiveSystem.getIntakeUpCommand()),
          getAutoShootCommand(2, true));
    }
  }

  private class AltFourBallAuto extends SequentialCommandGroup {
    private AltFourBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.AltFourBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.AltFourBall.initPose)),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.AltFourBall.initToWallToShoot)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          getAutoShootCommand(2, true),
          drivetrain.profiledTurnToAngleCommand(() -> 10.6),
          drivetrain.getRamseteCommand(
              drivetrain, Trajectories.AltFourBall.shootToOutsideTarmacToHPS),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.AltFourBall.HPSToFinalShoot)
              .alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain.profiledTurnToAngleCommand(() -> -145).withTimeout(1),
          getAutoShootCommand(2, true));
    }
  }

  /*TODO:
  Adjust 1st arc
  Straighten path from shoot to HPS and back
  drive faster, DONE, test on ground
  get real coords from field asap
  visit thors moms house immediately */

  private class FiveBallAuto extends SequentialCommandGroup {
    private final Command feedCommand = digestiveSystem.feedCommand();

    private FiveBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(Trajectories.FiveBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.FiveBall.initPose)),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.initToWallToShoot)
              .alongWith(
                  digestiveSystem
                      .getIntakeDownCommand()
                      .andThen(new WaitCommand(3))
                      .andThen(digestiveSystem.getIntakeUpCommand())),
          drivetrain.profiledTurnToAngleCommand(() -> -152.3),
          getAutoShootCommand(3, true)
            .alongWith(
              new SequentialCommandGroup(
                new WaitCommand(1),
                digestiveSystem.getIntakeDownCommand()//,
                // new InstantCommand(() -> feedCommand.schedule()),
                // new WaitCommand(1),
                // new InstantCommand(() -> feedCommand.cancel())
              )
            ),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.ShootToHPS)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          new WaitCommand(1),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.FiveBall.HPStoShoot)
              .alongWith(digestiveSystem.getIntakeUpCommand()),
          getAutoShootCommand(2, true));
    }
  }

  // Logger

  public void updateLogger() {
    Logger.updateEntries();
  }
}
