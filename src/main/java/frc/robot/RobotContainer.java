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
import frc.robot.Constants.AutoConstants;
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
  private final CommandXboxController secondController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);

  SendableChooser<Command> autonChooser = new SendableChooser<>();

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

    autonChooser.setDefaultOption("Two Ball (Positionable)", new TwoBallPositionableAuto());
    autonChooser.addOption("Four Ball", new FourBallAuto());

    SmartDashboard.putData("Select Autonomous", autonChooser);

    // Trajectories

    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAuto.tarmacToBall, "Tarmac To Ball Adjusted");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAuto.ballToHumanPlayer, "Ball To Human Player Adjusted");
    drivetrain.putTrajOnFieldWidget(
        Trajectories.fourBallAuto.humanPlayerToShoot, "Human Player To Shoot Adjusted");
  }

  private void configureButtonBindings() {

    driverController.rightBumper().whenHeld(new ShootCommand(true));
    driverController.a().whenHeld(new ShootCommand(1500, true));
    driverController.leftBumper().whenHeld(digestiveSystem.getIntakeCommand());

    testController
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

    secondController
        .leftBumper()
        .and(secondController.rightBumper())
        .whileActiveOnce(
            new RunCommand(
                () ->
                    climber.supplyClimbInputs(
                        () -> MathUtil.applyDeadband(secondController.getLeftY(), 0.7),
                        () -> MathUtil.applyDeadband(secondController.getRightY(), 0.7))))
        .whenInactive(() -> climber.climb(0));
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

    @Override
    public void end(boolean interrupted) {
      feedCommand.cancel();

      if(stopShooterAtEnd){
        digestiveSystem.setSpinUpFlywheelCustomFreq(false);
        digestiveSystem.setFlywheelVoltage(0);
      }
    }
  }

  public void spinUpFlywheelCustomFreq() {
    digestiveSystem.spinUpCustomFreqFunc();
  }

  private Command getAutoShootCommand(double duration, boolean useLimelight) {
    ParallelRaceGroup autoShootCommand;

    if (useLimelight) {
      autoShootCommand = new ShootCommand(false).withTimeout(duration);
    } else {
      autoShootCommand = new ShootCommand(3300, false).withTimeout(duration);
    }

    return autoShootCommand;
  }

  // Auton commands

  private class TwoBallPositionableAuto extends SequentialCommandGroup {
    private TwoBallPositionableAuto() {
      addCommands(
          new InstantCommand(() -> digestiveSystem.setIntakeSpeedProp(0.75)),
          new InstantCommand(
              () ->
                  drivetrain.resetGyro(
                      Trajectories.oneMeterForward.getInitialPose().getRotation().getDegrees())),
          new InstantCommand(
              () -> drivetrain.resetOdometry(Trajectories.oneMeterForward.getInitialPose())),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.oneMeterForward),
          drivetrain.profiledTurnToAngleCommand(() -> 180),
          getAutoShootCommand(2, true));
    }
  }

  private class FourBallAuto extends SequentialCommandGroup {
    private FourBallAuto() {
      addCommands(
          new InstantCommand(
              () -> drivetrain.resetGyro(Trajectories.initPose.getRotation().getDegrees())),
          new InstantCommand(
              () ->
                  drivetrain.resetOdometry(
                      Trajectories.initPose)),
          digestiveSystem.spinUpCommand(() -> AutoConstants.flywheelIdleRPM),
          drivetrain.profiledTurnToAngleCommand(
              () ->
                  Trajectories.fourBallAuto
                      .tarmacToBall
                      .getInitialPose()
                      .getRotation()
                      .getDegrees()),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.tarmacToBall)
            .alongWith(digestiveSystem.getIntakeDownCommand()),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.ballToShoot)
            .alongWith(digestiveSystem.getIntakeUpCommand()),
          getAutoShootCommand(2, false),
          drivetrain.profiledTurnToAngleCommand(
              () ->
                  360 + Trajectories.fourBallAuto
                      .ballToHumanPlayer
                      .getInitialPose()
                      .getRotation()
                      .getDegrees()),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.ballToHumanPlayer)
            .alongWith(digestiveSystem.getIntakeDownCommand()),
          new WaitCommand(2),
          drivetrain.getRamseteCommand(drivetrain, Trajectories.fourBallAuto.humanPlayerToShoot)
          .alongWith(digestiveSystem.getIntakeUpCommand()),
          drivetrain.profiledTurnToAngleCommand(() -> 240),
          getAutoShootCommand(2, false),
          new InstantCommand(() -> {
            digestiveSystem.setSpinUpFlywheelCustomFreq(false);
            digestiveSystem.setFlywheelVoltage(0);
          }));
    }
  }

  // Logger

  public void updateLogger() {
    Logger.updateEntries();
  }
}
