package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class TurnForAngle extends PIDCommand {

    private Drivetrain drivetrain; //todo: figure out how to use drivetrain without having it as constructor param

    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drivetrain         The drive subsystem to use
     */
    public TurnForAngle(Drivetrain drivetrain, double targetAngleDegrees) {
        super(
                //controller
                new PIDController(Constants.Drivetrain.smallTurnP, Constants.Drivetrain.smallTurnI, Constants.Drivetrain.smallTurnD),
                // Close loop on heading
                drivetrain::getHeading,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                output -> {
                    drivetrain.PIDarcadeDrive(0, output);
                    SmartDashboard.putNumber("pid output", output);
                    SmartDashboard.putNumber("robot heading", drivetrain.getHeading());
                },
                // Require the drive
                drivetrain);

        drivetrain.resetGyro();
        this.drivetrain = drivetrain;

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
                .setTolerance(Constants.Drivetrain.turnToleranceDeg, Constants.Drivetrain.turnRateToleranceDegPerS);

        SmartDashboard.putNumber("turn p", getController().getP());
        SmartDashboard.putNumber("turn i", getController().getI());
        SmartDashboard.putNumber("turn d", getController().getD());

    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.resetGyro();


    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        SmartDashboard.putBoolean("IsFinished", getController().atSetpoint());
        return getController().atSetpoint();
    }

    //    public void updateVals() {
//        getController().setP(SmartDashboard.getNumber("turn p", Constants.smallTurnP));
//        getController().setI(SmartDashboard.getNumber("turn i", Constants.smallTurnI));
//        getController().setD(SmartDashboard.getNumber("turn d", Constants.smallTurnD));
//
//    }

    public String getVals() {
        return getController().getP() + "\t" +
                getController().getI() + "\t\t" +
                getController().getD();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
