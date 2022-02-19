// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class Paths {
    public static Trajectory test, test2, turnForAngle;

    public static void generate(){
        test = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(5, 0, new Rotation2d(0))),
            Constants.Drivetrain.MotionProfilingConfig
        );

        turnForAngle = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(0.10, 0.10, new Rotation2d(90))),
            Constants.Drivetrain.MotionProfilingConfig
        );

        test2 = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(3, -1, new Rotation2d(-90)),
                    new Pose2d(0, -2, new Rotation2d(-180))),
            Constants.Drivetrain.MotionProfilingConfig
        );


    }
}
