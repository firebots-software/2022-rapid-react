// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  // private Solenoid piston;
  // TODO: changing to roller motor
  private TalonSRX rollerMotor;
  private TalonFX topMotor, bottomMotor; 
  private boolean atTargetSpeed;
  private double targetSpeed;

  /** Creates a new Shooter. */
  private Shooter() {
    // this.piston = new Solenoid(PneumaticsModuleType.CTREPCM,
    // Constants.Shooter.shooterPistonPort);
    this.topMotor = new TalonFX(Constants.Shooter.shooterTopMotorPort);
    this.bottomMotor = new TalonFX(Constants.Shooter.shooterBottomMotorPort);
    this.rollerMotor = new TalonSRX(Constants.Shooter.rollerMotorPort);
    atTargetSpeed = false;
    this.targetSpeed = 0;

  }

  /**
   * Returns the Singleton instance of this Shooter. This static method
   * should be used -- {@code Shooter.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 /* 
  * Returns motor RPM from selected sensor velocity.
  */
  public double getTopShooterRPM() {
    return (topMotor.getSelectedSensorVelocity() * 600.0) / Constants.Shooter.shooterEncoderTicksPerRev; // per 100ms * 600                                                                                         // = per min
  }
  public double getBottomShooterRPM() {
    return (bottomMotor.getSelectedSensorVelocity() * 600.0) / Constants.Shooter.shooterEncoderTicksPerRev; // per 100ms * 600                                                                                         // = per min
  }

  public void setRollerMotorSpeed(double speed){
   rollerMotor.set(ControlMode.PercentOutput, speed);
  }

 /* 
  * Sets speed of the motor as a setpoint value.
  * @param speed the specified speed to set the motor to
  */
  public void setTopMotorSpeed(double speed) {
    topMotor.set(ControlMode.PercentOutput, -speed);
    // value to move to aimed point
  }

  public void setBottomMotorSpeed(double speed) {
    bottomMotor.set(ControlMode.PercentOutput, -speed);
    // value to move to aimed point
  }

 /* 
  * Stops motor. Sets speed of the motor to zero.
  */
  public void stopTopMotor() {
    setTopMotorSpeed(0);
  }

  public void stopBottomMotor() {
    setTopMotorSpeed(0);
  }


  public void stopRollerMotor(){
    setRollerMotorSpeed(0.0);
  }

  // ask command or method

  /*
   * public void setAtTargetSpeed(boolean atTarget) {
   * this.atTargetSpeed = atTarget;
   * }
   */
  // add threshold for target speed

 /* 
  * Accessor for targetSpeed member variable.
  */
  public double getTargetSpeed() {
    return this.targetSpeed;
  }

 /* 
  * Mutates target speed to new specified target speed.
  * @param targetSpeed new specified target speed
  */
  public void setTargetSpeed(double targetSpeed) {
    this.targetSpeed = targetSpeed;
  }

 /* 
  * Checks whether motor RPM is within a specified MoE of target speed.
  * @param marginOfError bounded MoE between motor RPM and target speed
  */
  public boolean isAtTargetSpeed(double marginOfError) {
    double topError = getTopShooterRPM() - getTargetSpeed();
    double bottomError = getBottomShooterRPM() - getTargetSpeed();
    return (Math.abs(topError) <= marginOfError) && (Math.abs(bottomError) <= marginOfError); //both motors 
  }

  // extend/ retract piston --> binds to button

}
