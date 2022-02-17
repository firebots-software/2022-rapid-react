package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class RamseteGenerator {
    private static Drivetrain drivetrain = Drivetrain.getInstance();

    /**
     * Generator method to create a RamseteCommand that will follow a given path.
     *
     * @param path = trajectory object describing path
     * @return RamseteCommand to drive given path
     */
    public static RamseteCommand generateCommandForPath(Trajectory path) {

        PIDController leftController = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0);
        PIDController rightController = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0);

        RamseteCommand ramseteCommand = new RamseteCommand(
                path,
                drivetrain::getPose,
                Constants.Ramsete.ramseteController,
                Constants.Ramsete.feedforward,
                Constants.Drivetrain.kinematics,
                drivetrain::getWheelSpeeds,
                leftController,
                rightController,
                // RamseteCommand passes volts to the callback
                (leftVolts, rightVolts) -> {
                    drivetrain.tankDriveVolts(leftVolts, rightVolts);

                    SmartDashboard.putNumber("left measurement", drivetrain.getWheelSpeeds().leftMetersPerSecond);
                    SmartDashboard.putNumber("right measurement", drivetrain.getWheelSpeeds().rightMetersPerSecond);

                    SmartDashboard.putNumber("left target", leftController.getSetpoint());
                    SmartDashboard.putNumber("right target", rightController.getSetpoint());
                },
                drivetrain
        );

    
        return ramseteCommand;
    }


}