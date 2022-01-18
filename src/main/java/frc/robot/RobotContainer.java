// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DigestiveSystem;
import frc.robot.subsystems.Climber;
import frc.robot.DPadButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Climber m_climber = new Climber();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final DigestiveSystem m_digestiveSystem = new DigestiveSystem();
  
  private final Joystick upperChassis = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton bumper_left_upper = new JoystickButton(upperChassis, 5);
    DPadButton dpad_down = new DPadButton(upperChassis, DPadButton.Direction.DOWN);
    DPadButton dpad_up = new DPadButton(upperChassis, DPadButton.Direction.DOWN);


    
    bumper_left_upper.whenPressed(new InstantCommand(() -> m_digestiveSystem.takeIn(0.5)))
    .whenReleased(new InstantCommand(() -> m_digestiveSystem.takeIn(0)));

    dpad_down.whenPressed(new InstantCommand(() -> m_climber.climb(-0.5)))
    .whenReleased(new InstantCommand(() -> m_climber.climb(0)));
    dpad_up.whenPressed(new InstantCommand(() -> m_climber.climb(0.5)))
    .whenReleased(new InstantCommand(() -> m_climber.climb(0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_drivetrain.getRamseteCommand(m_drivetrain, m_drivetrain.getTrajFromFieldWidget("traj1", false));
  }
}
