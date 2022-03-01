// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.climber.ClimbToMiddle;
import frc.robot.commands.climber.ManualClimb;
import frc.robot.commands.climber.RetractClimber;
import frc.robot.commands.climber.RetractComplete;
import frc.robot.commands.climber.ReturnTrueWhenPressed;
import frc.robot.commands.drivetrain.FlipOrientation;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.ToggleSlowMode;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick ps4_controller;
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private SendableChooser<Command> autonChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    this.ps4_controller = new Joystick(Constants.OI.PS4_CONTROLLER_PORT);
    this.drivetrain = Drivetrain.getInstance();
    configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        drivetrain.setDefaultCommand(
          new JoystickDrive(
                  () -> ps4_controller.getRawAxis(1),
                  () -> ps4_controller.getRawAxis(2)));

    SmartDashboard.putData("Auton chooser", autonChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    final Button buttonName = new JoystickButton(ps4_controller, Constants.OI.PortNumber);
    buttonName.whenPressed(new commandName());
    */

    final Button flipOrientation = new JoystickButton(ps4_controller, Constants.OI.L3_BUTTON_PORT);
    flipOrientation.whenPressed(new FlipOrientation());

    final Button slowMode = new JoystickButton(ps4_controller, Constants.OI.L2_BUTTON_PORT);
    slowMode.whenHeld(new ToggleSlowMode());

    final Button climbToMid = new JoystickButton(ps4_controller, Constants.OI.TRIANGLE_BUTTON_PORT);
    climbToMid.whenPressed(new ClimbToMiddle());

    final Button climbHold = new JoystickButton(ps4_controller, Constants.OI.CIRCLE_BUTTON_PORT);
    climbHold.whenHeld(new ManualClimb(Constants.Climber.globalClimbSpeed));

    final Button retractComplete = new JoystickButton(ps4_controller, Constants.OI.SQUARE_BUTTON_PORT);
    retractComplete.whenPressed(new RetractComplete());

    final POVButton upPov = new POVButton(ps4_controller, 0);
    upPov.whenHeld(new ManualClimb(Constants.Climber.globalClimbSpeed));

    final POVButton downPov = new POVButton(ps4_controller, 180);
    downPov.whenHeld(new ManualClimb(-Constants.Climber.globalClimbSpeed));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
