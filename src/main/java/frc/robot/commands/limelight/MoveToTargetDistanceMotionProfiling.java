// ALSO NOT USING THIS ANYMORE

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.limelight;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants;
// import frc.robot.Paths;
// import frc.robot.RamseteGenerator;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;

// public class MoveToTargetDistanceMotionProfiling extends CommandBase {
//   private Limelight limelight; 
//   private Drivetrain drivetrain; 
//   private RamseteGenerator ramseteGenerator; 

//   /** Creates a new MoveToTargetDistanceMotionProfiling. */
//   public MoveToTargetDistanceMotionProfiling() {
//     limelight = Limelight.getInstance(); 
//     drivetrain = Drivetrain.getInstance(); 
//     ramseteGenerator = new RamseteGenerator(); 
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     drivetrain.resetEncoders();
//     limelight.refreshValues();
//     RamseteCommand moveToTargetDistanceCommand = RamseteGenerator.generateCommandForPath(Paths.moveToTargetDistance); 
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
