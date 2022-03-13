// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.validation.SchemaFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;
  public WPI_TalonSRX rollerMotor, spaghettiMotor;
  private Solenoid piston;

  

  /** Creates a new Intake. */
  private Intake() {
    piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.PISTON_PORT);
    rollerMotor = new WPI_TalonSRX(Constants.Intake.INTAKE_MOTOR_PORT);
    spaghettiMotor = new WPI_TalonSRX(Constants.Intake.SPAGHETTI_MOTOR_PORT);

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

  public void extendIntake() {
    piston.set(true);
  }

  public void retractIntake() {
    piston.set(false);
  }

  public void togglePiston() {
    piston.set(!piston.get());
  }

  public boolean pistonExtended() {
    return piston.get();
  }


  public void runRollerMotor(double speed) {
    if (pistonExtended()) { // only if extended
      rollerMotor.set(ControlMode.PercentOutput, speed);
    } 
    // else {
    //   rollerMotor.set(ControlMode.PercentOutput, 0);
    // }
  }

  public void runSpaghettiMotor(double speed) {
    spaghettiMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotors() {
    runRollerMotor(0);
    runSpaghettiMotor(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
