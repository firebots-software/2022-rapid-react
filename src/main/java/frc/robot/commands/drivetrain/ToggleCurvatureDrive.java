package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleCurvatureDrive extends CommandBase {
  private final Drivetrain drivetrain;

  private Joystick ps4_controller;

  /**
   * Toggle which default driving command is used for teleop: arcade drive or
   * curvature drive.
   * 
   * @param ps4 = ps4 controller
   */
  public ToggleCurvatureDrive(Joystick ps4) {
    this.drivetrain = Drivetrain.getInstance();
    ps4_controller = ps4;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.toggleDefaultCommand(ps4_controller);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
