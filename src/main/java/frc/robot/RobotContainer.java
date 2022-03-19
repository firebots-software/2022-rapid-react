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
import frc.robot.commandGroups.AimAndShoot2Balls;
import frc.robot.commandGroups.AimAndShoot;
import frc.robot.commandGroups.DummyAutonAndShoot;
import frc.robot.commandGroups.RunIntakeAndSpaghetti;
import frc.robot.commandGroups.RunSpaghetAndRoll;
import frc.robot.commandGroups.StopEverything;
import frc.robot.commandGroups.TaxiIntakeShoot;
import frc.robot.commandGroups.TaxiTurnShoot;
import frc.robot.commands.auton.*;
import frc.robot.commands.limelight.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.climber.ManualClimb;
import frc.robot.commands.intake.FlapIntake;
import frc.robot.commands.intake.ToggleIntakePiston;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick ps4_controller1;
  private Joystick ps4_controller2; 
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Turret turret = Turret.getInstance(); 
  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    this.ps4_controller1 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_1);
    this.ps4_controller2 = new Joystick(Constants.OI.PS4_CONTROLLER_PORT_2); 
    
    Paths.generate();
    this.drivetrain = Drivetrain.getInstance();
    this.turret = Turret.getInstance(); 
    configureButtonBindings();

    // Configure default commands
    // Set the default driv e command to split-stick arcade drive
    drivetrain.setDefaultCommand(
        new JoystickArcadeDrive(
            () -> ps4_controller1.getRawAxis(1),
            () -> ps4_controller1.getRawAxis(2)));
    
    turret.setDefaultCommand(
        new AlignToTargetFeedForward()
    ); 

    autonChooser.setDefaultOption("limelightAim", new AlignToTargetFeedForward());

    SmartDashboard.putData("Auton chooser", autonChooser);
    autonChooser.setDefaultOption("DUMMY AUTON", new DriveForTime(-0.5, 1));
    autonChooser.setDefaultOption("dummy auton backwards & shoot", new DummyAutonAndShoot());
    autonChooser.addOption("taxi, turn, shoot", new TaxiTurnShoot());
    autonChooser.addOption("intake and shoot 2", new TaxiIntakeShoot());
    autonChooser.addOption("test only -- turn for time", new TurnForTime(0.35, 3));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * final Button buttonName = new JoystickButton(ps4_controller,
     * Constants.OI.PortNumber);
     * buttonName.whenPressed(new commandName());
     */

    final Button flipOrientation = new JoystickButton(ps4_controller1, Constants.OI.L3_BUTTON_PORT);
    flipOrientation.whenPressed(new FlipOrientation());

    final Button slowMode = new JoystickButton(ps4_controller1, Constants.OI.L1_BUTTON_PORT);
    slowMode.whenHeld(new ToggleSlowMode());

    final Button loadBall = new JoystickButton(ps4_controller1, Constants.OI.X_BUTTON_PORT); // TODO: change button
                                                                                            // accordingly
    loadBall.whenHeld(new RunSpaghetAndRoll());

    final Button spinUpShooter = new JoystickButton(ps4_controller1, Constants.OI.CIRCLE_BUTTON_PORT);
    spinUpShooter.toggleWhenPressed(new SpinUpShooter());

    final Button limelightAim = new JoystickButton(ps4_controller1, Constants.OI.TRIANGLE_BUTTON_PORT);
    limelightAim.whenHeld(new LimelightAimPosControl());

    final Button toggleAdjustRPM = new JoystickButton(ps4_controller1, Constants.OI.OPTIONS_BUTTON_PORT);
    toggleAdjustRPM.whenPressed(new ToggleAdjustableShooter());

    final Button toggleCamOrientation = new JoystickButton(ps4_controller1, Constants.OI.PS_SHARE_BUTTON_PORT);
    toggleCamOrientation.whenPressed(new ToggleCameraOrientation());

    final Button toggleIntake = new JoystickButton(ps4_controller1, Constants.OI.PS_BUTTON_PORT);
    toggleIntake.whenPressed(new ToggleIntakePiston());

    final Button runIntake = new JoystickButton(ps4_controller1, Constants.OI.R1_BUTTON_PORT);
    runIntake.whenHeld(new RunIntakeAndSpaghetti(false));

    final Button stopEverything = new JoystickButton(ps4_controller1, Constants.OI.BIG_BUTTON_PORT);
    stopEverything.whenPressed(new StopEverything());

    final POVButton upPov = new POVButton(ps4_controller1, 0);
    upPov.whenHeld(new ManualClimb(Constants.Climber.climbSpeedUp));

    final POVButton downPov = new POVButton(ps4_controller1, 180);
    downPov.whenHeld(new ManualClimb(Constants.Climber.climbSpeedDown));

    // only if using second controller
    double manualTurretSpeed = 0.5;
    final POVButton leftPov = new POVButton(ps4_controller2, 270);
    leftPov.whenHeld(new ManualTurretTurn(-manualTurretSpeed));

    final POVButton rightPov = new POVButton(ps4_controller2, 90);
    rightPov.whenHeld(new ManualTurretTurn(manualTurretSpeed));

    final Button pjAiming = new JoystickButton(ps4_controller1, Constants.OI.SQUARE_BUTTON_PORT);
    pjAiming.whenHeld(new AlignToTargetFeedForward());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return null;
    return autonChooser.getSelected();
  }
}
