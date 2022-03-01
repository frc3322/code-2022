// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
          () -> {
            double speed = MathUtil.applyDeadband(-driverController.getLeftY(), 0.07);
            double turn = MathUtil.applyDeadband(driverController.getRightX(), 0.07);

            drivetrain.drive(speed, turn, driverController.leftStick().get());
          },
          drivetrain);

  public RobotContainer() {

    // Configuration
    Logger.configureLoggingAndConfig(this, false);
    configureButtonBindings();

    // Default commands
    drivetrain.setDefaultCommand(driveCommand);

    // Trajectories
    // drivetrain.putTrajOnFieldWidget(Trajectories.fourBallAuto.tarmacToBall, "Tarmac To Ball");
    // drivetrain.putTrajOnFieldWidget(
    //     Trajectories.fourBallAuto.ballToHumanPlayer, "Ball To Human Player");
    // drivetrain.putTrajOnFieldWidget(
    //     Trajectories.fourBallAuto.humanPlayerToShoot, "Human Player To Shoot");

    // drivetrain.putTrajOnFieldWidget(
    //   Trajectories.fourBallAutoTransformed.tarmacToBall, "Tarmac To Ball Absolute");
    // drivetrain.putTrajOnFieldWidget(
    //   Trajectories.fourBallAutoTransformed.ballToHumanPlayer, "Ball To Human Player Absolute");
    // drivetrain.putTrajOnFieldWidget(
    //   Trajectories.fourBallAutoTransformed.humanPlayerToShoot, "Human Player To Shoot Absolute");

    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAutoAdjusted.tarmacToBall, "Tarmac To Ball Adjusted");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAutoAdjusted.ballToHumanPlayer, "Ball To Human Player Adjusted");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAutoAdjusted.humanPlayerToShoot, "Human Player To Shoot Adjusted");
  }

  private void configureButtonBindings() {

    driverController.a().whenHeld(new ShootCommand());
    driverController.leftBumper().whenHeld(new ShootCommand(1500));
    driverController.rightBumper().whenHeld(digestiveSystem.getIntakeCommand());

    driverController
        .y()
        .whenPressed(
            new InstantCommand(
                    () -> drivetrain.resetGyro(Trajectories.initPose.getRotation().getDegrees()))
                .andThen(
                    () ->
                        drivetrain.resetOdometry(
                            Trajectories
                                .initPose))); // Trajectories.fourBallAutoAdjusted.ballToHumanPlayer.getStates().get(Trajectories.fourBallAutoAdjusted.ballToHumanPlayer.getStates().size() - 1).poseMeters)));

    driverController
        .x()
        .whenHeld(
            new StartEndCommand(
                () -> digestiveSystem.setIntakeSpeedProp(-0.7),
                () -> digestiveSystem.setIntakeSpeedProp(0),
                digestiveSystem));

    driverController
        .start()
        .whenHeld(digestiveSystem.getShooterPurgeCommand())
        .whenReleased(() -> digestiveSystem.setFlywheelVoltage(0));

    testController
        .leftBumper()
        .and(testController.rightBumper())
        .whileActiveOnce(
            new RunCommand(
                () ->
                    climber.supplyClimbInputs(
                        () -> testController.getLeftY(), () -> testController.getRightY())))
        .whenInactive(() -> climber.climb(0));
  }

  public Command getAutonomousCommand() {
    return new FourBallAuto();
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

    private ShootCommand() {
      addCommands(
          digestiveSystem.getShootCommand(
              () -> LerpLLYtoRPM.getRPMFromSupplier(() -> drivetrain.getLimelightAngleY())),
          drivetrain.getTurnToLimelightCommand() /*.withInterrupt(alignedAndSped)*/,
          // new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
          waitUntilAlignedAndSpedCommand.andThen(() -> feedCommand.schedule()));
    }

    private ShootCommand(double RPM) {
      addCommands(
          digestiveSystem.getShootCommand(() -> RPM),
          waitUntilSpedCommand.andThen(() -> feedCommand.schedule()));
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

  private class FourBallAuto extends SequentialCommandGroup {
    private FourBallAuto() {
      addCommands(
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.fourBallAutoAdjusted
                          .tarmacToBall
                          .getInitialPose()
                          .getRotation()
                          .getDegrees())),
          new InstantCommand(
              () ->
                  drivetrain.resetOdometry(
                      Trajectories.fourBallAutoAdjusted.tarmacToBall.getInitialPose())),
          new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAutoAdjusted.tarmacToBall),
          new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0)),
          drivetrain.profiledTurnToAngleCommand(() -> -143),
          getAutoShootCommand(2, false),
          drivetrain.profiledTurnToAngleCommand(
              () ->
                  Trajectories.fourBallAutoAdjusted
                      .ballToHumanPlayer
                      .getInitialPose()
                      .getRotation()
                      .getDegrees()),
          new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
          drivetrain.getRamseteCommand(
              drivetrain, Trajectories.fourBallAutoAdjusted.ballToHumanPlayer),
          new WaitCommand(2),
          new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0)),
          drivetrain.getRamseteCommand(
              drivetrain, Trajectories.fourBallAutoAdjusted.humanPlayerToShoot),
          drivetrain.profiledTurnToAngleCommand(() -> -135),
          getAutoShootCommand(2, false));
    }
  }

  private Command getAutoShootCommand(double duration, boolean useLimelight) {
    ParallelRaceGroup autoShootCommand;

    if(useLimelight){
        autoShootCommand = new ShootCommand().withTimeout(duration);
    } else {
        autoShootCommand = new ShootCommand(3300).withTimeout(duration);
    }

    return autoShootCommand;
  }

  public void updateLogger() {
    Logger.updateEntries();
  }
}
