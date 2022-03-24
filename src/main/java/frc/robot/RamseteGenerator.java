package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.subsystems.Drivetrain;

public class RamseteGenerator {
    private static Drivetrain drivetrain = Drivetrain.getInstance();

    /**
     * Generator method to create a RamseteCommand that will follow a given path.
     *
     * @param path = trajectory object describing path
     * @return RamseteCommand to drive given path
     */
    public static Command generateCommandForPath(Trajectory path) {

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

                    // SmartDashboard.putNumber("motprof left target", leftController.getSetpoint());
                    // SmartDashboard.putNumber("motprof right target", rightController.getSetpoint());

                    // SmartDashboard.putNumber("motprof left speed", drivetrain.getLeftEncoderVelocityMetersPerSec());
                    // SmartDashboard.putNumber("motprof right speed", drivetrain.getRightEncoderVelocityMetersPerSec());

                    // System.out.println("executing mot prof");
                },
                drivetrain
        );

    
        return new ResetOdometry().andThen(ramseteCommand);
    }


}