// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Logger;
import frc.robot.Trajectories.*;

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

  public RobotContainer() {

    // Configuration
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();

    // Default commands
    drivetrain.setDefaultCommand(driveCommand);

    autonChooser.setDefaultOption("Two Ball (Positionable)", new TwoBallPositionableAuto());
    autonChooser.addOption("Three Ball", new ThreeBallAuto());
    autonChooser.addOption("Four Ball", new FourBallAuto());
    autonChooser.addOption("Alt Four Ball", new AltFourBallAuto());

    SmartDashboard.putData("Select Autonomous", autonChooser);

    // Trajectories

    // drivetrain.putTrajOnFieldWidget(Trajectories.FourBallAuto.tarmacToBall, "Tarmac To Ball");
    // drivetrain.putTrajOnFieldWidget(
    //     Trajectories.FourBallAuto.shootToHumanPlayer, "Shoot To Human Player");
    // drivetrain.putTrajOnFieldWidget(
    //     Trajectories.FourBallAuto.humanPlayerToShoot, "Human Player To Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.RightSide.tarmacToShoot, "Tarmac To Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.RightSide.FourBallAuto.shootToHumanPlayer, "Shoot To Human Player");
    // drivetrain.putTrajOnFieldWidget(Trajectories.RightSide.FourBallAuto.humanPlayerToShoot, "Human Player To Shoot");
    // drivetrain.putTrajOnFieldWidget(Trajectories.FourBallAuto.shootToWallBall, "Shoot To Wall Ball");
    // drivetrain.putTrajOnFieldWidget(Trajectories.AltFourBall.goToWallBall, "Go to wall ball");

    drivetrain.putTrajOnFieldWidget(Trajectories.AltFourBall.initToWallToShoot, "InitToWallBallToShoot");
    drivetrain.putTrajOnFieldWidget(Trajectories.AltFourBall.shootToOutsideTarmacToHPS, "shootToOutsideTarmacToHPS");
    drivetrain.putTrajOnFieldWidget(Trajectories.AltFourBall.HPSToFinalShoot, "HPSToFinalShoot");

    SmartDashboard.putNumber("Shooter Target RPM", 0);
    
  }

  private void configureButtonBindings() {

    driverController.rightBumper().whenHeld(new ShootCommand(true));
    driverController.a().whenHeld(new ShootCommand(1250, true));
    driverController.b().whenHeld(new ShootCommand(() -> SmartDashboard.getNumber("Shooter Target RPM", 0), true));
    driverController
        .leftBumper()
        .and(new Trigger(() -> !digestiveSystem.getStomachFull()))
        .whileActiveOnce(digestiveSystem.getIntakeCommand());

    driverController
        .x()
        .whenHeld(
            new StartEndCommand(
                () -> {
                  digestiveSystem.setIntakeSpeedProp(-0.7);
                  digestiveSystem.setTransferSpeedProp(-0.5);
                },
                () -> {
                  digestiveSystem.setIntakeSpeedProp(0);
                  digestiveSystem.setTransferSpeedProp(0);
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
                        Trajectories.AltFourBall.initPose.getRotation().getDegrees()))
            .andThen(() -> drivetrain.resetOdometry(Trajectories.AltFourBall.initPose)));

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

    private final Command feedCommand =
        new StartEndCommand(
            () -> digestiveSystem.setTransferSpeedProp(0.5),
            () -> digestiveSystem.setTransferSpeedProp(0),
            digestiveSystem);

    private final Command waitUntilAlignedAndSpedCommand = new WaitUntilCommand(alignedAndSped);

    private final Command waitUntilSpedCommand = new WaitUntilCommand(sped);

    private Boolean stopShooterAtEnd;

    private ShootCommand(Boolean stopShooterAtEnd) {
      this.stopShooterAtEnd = stopShooterAtEnd;

      addCommands(
          digestiveSystem.getShootCommand(
              () -> LerpLLYtoRPM.getRPMFromSupplier(() -> drivetrain.getLimelightAngleY())),
          drivetrain.getTurnToLimelightCommand() /*.withInterrupt(alignedAndSped)*/,
          // new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
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
          drivetrain.getRamseteCommand(drivetrain, Trajectories.RightSide.FourBallAuto.shootToHumanPlayer)
            .alongWith(new WaitCommand(4).andThen(digestiveSystem.getIntakeDownCommand())),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.RightSide.FourBallAuto.humanPlayerToShoot)
            .alongWith(digestiveSystem.getIntakeUpCommand()),
          getAutoShootCommand(2, true));
    }
  }

  /*TODO:
  Adjust 1st arc
  Straighten path from shoot to HPS and back
  drive faster, DONE, test on ground
  get real coords from field asap
  visit thors moms house immediately */

  private class AltFourBallAuto extends SequentialCommandGroup {

    private AltFourBallAuto() {
      addCommands(
        new InstantCommand(
          () ->
          drivetrain.resetGyro(Trajectories.AltFourBall.initPose.getRotation().getDegrees())),
          new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.AltFourBall.initPose)),
          
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.AltFourBall.initToWallToShoot)
              .alongWith(digestiveSystem.getIntakeDownCommand()),
          getAutoShootCommand(2, true),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.AltFourBall.shootToOutsideTarmacToHPS),
          drivetrain
              .getRamseteCommand(drivetrain, Trajectories.AltFourBall.HPSToFinalShoot)
              .alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain.profiledTurnToAngleCommand(() -> -140).withTimeout(1),
          getAutoShootCommand(2, true)
              
        );
    }
  }

  // Logger

  public void updateLogger() {
    Logger.updateEntries();
  }
}
