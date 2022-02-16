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
    public static Trajectory test, rotationTest, test2;

    public static void generate(){
        test = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(5, 0, new Rotation2d(0))),
            Constants.Drivetrain.MotionProfilingConfig
        );

        rotationTest = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(2, 0, new Rotation2d(0)),
                    new Pose2d(2, -1, new Rotation2d(0)),
                    new Pose2d(4, -1, new Rotation2d(0))),
            Constants.Drivetrain.MotionProfilingConfig
        );

        test2 = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(5, -3, new Rotation2d(0))),
            Constants.Drivetrain.MotionProfilingConfig
        );


    }
}
