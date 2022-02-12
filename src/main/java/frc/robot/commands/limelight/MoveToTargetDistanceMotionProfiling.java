// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class MoveToTargetDistanceMotionProfiling extends CommandBase {
  private Limelight limelight; 
  private Drivetrain drivetrain; 
  private RamseteGenerator ramseteGenerator; 

  /** Creates a new MoveToTargetDistanceMotionProfiling. */
  public MoveToTargetDistanceMotionProfiling() {
    limelight = Limelight.getInstance(); 
    drivetrain = Drivetrain.getInstance(); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    limelight.refreshValues();
    Trajectory moveToTargetDistance = generatePaths(limelight.getDistanceToTarget()); 
  }

  public static Trajectory generatePaths(double distanceNeeded) {
    Trajectory moveDistance = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(0.5, 0)), 
      new Pose2d(distanceNeeded, 0, new Rotation2d(0)), 
      Constants.Drivetrain.MotionProfilingConfig); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
