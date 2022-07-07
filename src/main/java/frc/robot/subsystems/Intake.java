// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Declaring necessary fields
  private static Intake instance;
  public WPI_TalonSRX rollerMotor, spaghettiMotor;

  // solenoid is the piston controller
  private Solenoid piston;

  

  /** Creates a new Intake. */
  private Intake() {
    // defining all necessary fields
    piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.PISTON_PORT);
    rollerMotor = new WPI_TalonSRX(Constants.Intake.INTAKE_MOTOR_PORT);
    spaghettiMotor = new WPI_TalonSRX(Constants.Intake.SPAGHETTI_MOTOR_PORT);

    // starts in the retracted stage
    retractIntake();

  }

  /**
   * Returns the Singleton instance of this Intake. This static method
   * should be used -- {@code Intake.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  // pistons only have 2 states 
  // true = fully extended
  public void extendIntake() {
    piston.set(true);
  }

  // false = fully retracted
  public void retractIntake() {
    piston.set(false);
  }

  // flipping a boolean value to extend/retract intake
  public void togglePiston() {
    piston.set(!piston.get());
  }

  // returns state of the piston
  public boolean pistonExtended() {
    return piston.get();
  }

  // need the motor to spin in order to get the balls into the robot
  // don't want motor to spin if closed
  // check if opened, then spin motor at predefined constant speed
  public void runRollerMotor(double speed) {
    if (pistonExtended()) { // only if extended
      rollerMotor.set(ControlMode.PercentOutput, speed);
    } 
  }

  // run spaghetti motors at given speed
  public void runSpaghettiMotor(double speed) {
    spaghettiMotor.set(ControlMode.PercentOutput, speed);
  }

  // stop all motors --> set all speed to 0
  public void stopMotors() {
    runRollerMotor(0);
    runSpaghettiMotor(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

// CLEANED