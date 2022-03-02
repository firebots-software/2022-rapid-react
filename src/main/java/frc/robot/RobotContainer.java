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
import frc.robot.commandGroups.MoveToBallAndShootingDistance;
import frc.robot.commandGroups.TaxiAndIntake;
import frc.robot.commandGroups.TimeDriveAndShoot;
import frc.robot.commands.auton.*;
import frc.robot.commands.limelight.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.shooter.*;
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
    this.drivetrain = Drivetrain.getInstance();
    configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        drivetrain.setDefaultCommand(
          new CurvatureDrive(
                  () -> ps4_controller.getRawAxis(1),
                  () -> ps4_controller.getRawAxis(2)));

    autonChooser.setDefaultOption("limelightAim", new AlignToTarget());

    SmartDashboard.putData("Auton chooser", autonChooser);
    autonChooser.setDefaultOption("Drive Back for Time", new DriveForTime(-0.5, 4));
    autonChooser.addOption("DRIVE TURN SHOOT", new TimeDriveAndShoot());
    autonChooser.addOption("dummy auton with turn 180", new DriveForTime(-0.5, 4).andThen(new TurnForAngle(180)));
    autonChooser.addOption("only taxi", RamseteGenerator.generateCommandForPath(Paths.moveToBall));
    autonChooser.addOption("taxi and move to shooting distance", new MoveToBallAndShootingDistance());
    autonChooser.addOption("taxi and intake ball", new TaxiAndIntake());
    autonChooser.addOption("drive back 1m", RamseteGenerator.generateCommandForPath(Paths.moveToShootingDistanceFromBall));
    autonChooser.addOption("drive forward 1m", RamseteGenerator.generateCommandForPath(Paths.moveToBall));
    autonChooser.addOption("turn 180", new TurnForAngle(180));
    // autonChooser.addOption("taxi and shoot ball", taxiAndShoot);
    // autonChooser.addOption("taxi and intake and shoot one ball", taxiIntakeShootOne);
    // autonChooser.addOption("taxi and intake and shoot two balls", taxiIntakeShootTwo);



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

    final Button loadBall = new JoystickButton(ps4_controller, Constants.OI.X_BUTTON_PORT); //TODO: change button accordingly
    loadBall.whenHeld(new StartRoller());

    final Button spinUpShooter = new JoystickButton(ps4_controller, Constants.OI.CIRCLE_BUTTON_PORT);
    spinUpShooter.toggleWhenPressed(new SpinUpShooter());

    final Button turretClockwise = new JoystickButton(ps4_controller, Constants.OI.R1_BUTTON_PORT);
    turretClockwise.whenHeld(new ManualTurretTurn(0.3));

    final Button turretCounterclockwise = new JoystickButton(ps4_controller, Constants.OI.L1_BUTTON_PORT);
    turretCounterclockwise.whenHeld(new ManualTurretTurn(-0.3));

    final Button limelightAim = new JoystickButton(ps4_controller, Constants.OI.TRIANGLE_BUTTON_PORT);
    limelightAim.whenPressed(new AlignToTarget());

    final Button toggleAdjustRPM = new JoystickButton(ps4_controller, Constants.OI.OPTIONS_BUTTON_PORT);
    toggleAdjustRPM.whenPressed(new ToggleAdjustableShooter());

    final Button toggleCamOrientation = new JoystickButton(ps4_controller, Constants.OI.PS_SHARE_BUTTON_PORT);
    toggleCamOrientation.whenPressed(new ToggleCameraOrientation());

    


  


    // double rpmInterval = 100;
    // final Button increaseTopWheel = new JoystickButton(ps4_controller, Constants.OI.L1_BUTTON_PORT);
    // increaseTopWheel.whenPressed(new ChangeShooterTargetRPM(true, rpmInterval));

    // final Button decreaseTopWheel = new JoystickButton(ps4_controller, Constants.OI.L2_BUTTON_PORT);
    // decreaseTopWheel.whenPressed(new ChangeShooterTargetRPM(true, -rpmInterval));

    // final Button increaseBottomWheel = new JoystickButton(ps4_controller, Constants.OI.R1_BUTTON_PORT);
    // increaseBottomWheel.whenPressed(new ChangeShooterTargetRPM(false, rpmInterval));

    // final Button decreaseBottomWheel = new JoystickButton(ps4_controller, Constants.OI.R2_BUTTON_PORT);
    // decreaseBottomWheel.whenPressed(new ChangeShooterTargetRPM(false, -rpmInterval));


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
