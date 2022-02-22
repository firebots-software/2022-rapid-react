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
  private double topTargetRPM, bottomTargetRPM;
  private final double MAX_SPEED = 1;
  private final double RAMPING_CONSTANT = 0.25;

  /** Creates a new Shooter. */
  private Shooter() {
    // this.piston = new Solenoid(PneumaticsModuleType.CTREPCM,
    // Constants.Shooter.shooterPistonPort);
    this.topMotor = new TalonFX(Constants.Shooter.shooterTopMotorPort);
    topMotor.configOpenloopRamp(RAMPING_CONSTANT);
    this.bottomMotor = new TalonFX(Constants.Shooter.shooterBottomMotorPort);
    bottomMotor.configOpenloopRamp(RAMPING_CONSTANT);


    this.rollerMotor = new TalonSRX(Constants.Shooter.rollerMotorPort);
    atTargetSpeed = false;
    this.topTargetRPM = 3000;
    this.bottomTargetRPM = 3000;

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
    return (-topMotor.getSelectedSensorVelocity() * 600.0) / Constants.Shooter.shooterEncoderTicksPerRev; // per 100ms * 600                                                                                         // = per min
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
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    topMotor.set(ControlMode.PercentOutput, -speed);
    // value to move to aimed point
  }

  public void setBottomMotorSpeed(double speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    bottomMotor.set(ControlMode.PercentOutput, speed);
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

  public void stopBothMotors() {
    stopTopMotor();
    stopBottomMotor();
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


  public void setTopTargetRPM(double rpm) {
    this.topTargetRPM = rpm;
  }  
  
  public void setBottomTargetRPM(double rpm) {
    this.bottomTargetRPM = rpm;
  }

 /* 
  * Checks whether motor RPM is within a specified MoE of target speed.
  * @param marginOfError bounded MoE between motor RPM and target speed
  */
  public boolean isAtTargetSpeed(double marginOfError) {
    return bottomAtTargetRPM(marginOfError) && topAtTargetRPM(marginOfError); //both motors 
  }

  public boolean topAtTargetRPM(double marginOfError) {
    double error = getTopShooterRPM() - getTopTargetRPM();
    return Math.abs(error) <= marginOfError; //both motors 
  }

  public boolean bottomAtTargetRPM(double marginOfError) {
    double error = getBottomShooterRPM() - getBottomTargetRPM();
    return Math.abs(error) <= marginOfError; //both motors 
  }

  public double getTopMotorOutput() {
    return topMotor.getMotorOutputPercent();
  }

  public double getBottomMotorOutput() {
    return bottomMotor.getMotorOutputPercent();
  }

  public double getTopTargetRPM() {
    return topTargetRPM;
  }

  public double getBottomTargetRPM() {
    return bottomTargetRPM;
  }

  public double getTopVoltage() {
    return topMotor.getBusVoltage();
  }

  public double getBottomVoltage() {
    return bottomMotor.getBusVoltage();
  }
}
