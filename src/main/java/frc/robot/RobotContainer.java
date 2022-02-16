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
import frc.robot.commands.auton.DriveBackForTime;
import frc.robot.commands.auton.DriveForDistance2;
import frc.robot.commands.auton.DriveForDistanceSingleController;
import frc.robot.commands.auton.TurnForAngle;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.SetDrivetrainBrakeMode;
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
    Paths.generate();
    configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        drivetrain.setDefaultCommand(
          new JoystickDrive(
                  () -> ps4_controller.getRawAxis(1),
                  () -> ps4_controller.getRawAxis(2)));

    SmartDashboard.putData("Auton chooser", autonChooser);
    autonChooser.setDefaultOption("Drive Back for Time", new DriveBackForTime(-0.5, 2));
    autonChooser.addOption("Drive Forward for Time", new DriveBackForTime(0.7, 5));
    autonChooser.addOption("Drive for Distance Test 2", new DriveForDistance2(1));
    autonChooser.addOption("Drive for Distance Test Single Controller", new DriveForDistanceSingleController(1));
    autonChooser.addOption("Motion Profiling Test", RamseteGenerator.generateCommandForPath(Paths.test));
    autonChooser.addOption("Turn Set Angle", new TurnForAngle(180));

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
    // final Button driveForDist = new JoystickButton(ps4_controller, Constants.OI.X_BUTTON_PORT);
    // driveForDist.whenPressed(new DriveForDistance2(2.13));
    
    // final Button driveForDist1 = new JoystickButton(ps4_controller, Constants.OI.TRIANGLE_BUTTON_PORT);
    // driveForDist1.whenPressed(new DriveForDistance(drivetrain, 2.13));

    final Button motionProfile = new JoystickButton(ps4_controller, Constants.OI.X_BUTTON_PORT);
    motionProfile.whenPressed(RamseteGenerator.generateCommandForPath(Paths.rotationTest));

    final Button brake = new JoystickButton(ps4_controller, Constants.OI.CIRCLE_BUTTON_PORT);
    brake.whenPressed(new SetDrivetrainBrakeMode());

    


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
