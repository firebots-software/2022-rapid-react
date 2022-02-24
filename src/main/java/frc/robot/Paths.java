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
    public static Trajectory moveToBall, moveToShootingDistanceFromBall;

    public static void generate(){
        moveToBall = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(1, 0, new Rotation2d(0))),
            Constants.Drivetrain.MotionProfilingConfig
        );

        moveToShootingDistanceFromBall = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(-1, 0, new Rotation2d(0))), // Negative distance because robot facing hoop and moving backwards
            Constants.Drivetrain.MotionProfilingConfigReversed
        );

    }
}
