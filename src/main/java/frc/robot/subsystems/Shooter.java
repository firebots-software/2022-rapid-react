// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // fields
  private static Shooter instance;
  private Limelight limelight;
  private TalonSRX rollerMotor;
  private TalonFX topMotor, bottomMotor; 
  private double topTestTargetRPM, bottomTestTargetRPM;
  private final double MAX_SPEED = 1;
  private final double RAMPING_CONSTANT = 0.25;
  private final double TOP_FLYWHEEL_CONST = 0.8;
  private final double RPM_MOE = 120;
  private double currentDist; 
  private boolean isAdjustingRPM; 
  private SimpleMotorFeedforward topMotorFF, bottomMotorFF;

  /** Creates a new Shooter */
  private Shooter() {
    // top motor configs
    this.topMotor = new TalonFX(Constants.Shooter.shooterTopMotorPort);
    topMotor.configFactoryDefault();
    topMotor.configOpenloopRamp(RAMPING_CONSTANT); // slowls the rate at which it speeds up so motor isn't damaged
    topMotor.setInverted(true); // motor spins backward
    topMotor.configVoltageCompSaturation(12); // max voltage that can be applied
    topMotor.enableVoltageCompensation(true); // TODO: figure out what this does

    // bottom motor configs
    this.bottomMotor = new TalonFX(Constants.Shooter.shooterBottomMotorPort);
    bottomMotor.configFactoryDefault();
    bottomMotor.configOpenloopRamp(RAMPING_CONSTANT);
    bottomMotor.setInverted(true);
    bottomMotor.configVoltageCompSaturation(12);
    bottomMotor.enableVoltageCompensation(true);


    this.rollerMotor = new TalonSRX(Constants.Shooter.rollerMotorPort);
    rollerMotor.setInverted(true);

    this.topTestTargetRPM = Constants.Shooter.FIXED_RPM;
    this.bottomTestTargetRPM = Constants.Shooter.FIXED_RPM;

    // feed forward controllers for both motors
    topMotorFF = new SimpleMotorFeedforward(Constants.Shooter.ksTopFlywheel, Constants.Shooter.kvTopFlywheel, Constants.Shooter.kaTopFlywheel);
    bottomMotorFF = new SimpleMotorFeedforward(Constants.Shooter.ksBottomFlywheel, Constants.Shooter.kvBottomFlywheel, Constants.Shooter.kaBottomFlywheel);

    isAdjustingRPM = true; 

    limelight = Limelight.getInstance();
    currentDist = 0; 

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
    return (topMotor.getSelectedSensorVelocity() * 600.0) / Constants.Shooter.shooterEncoderTicksPerRev; // rev per 100ms * 600 = rpm                                                                                 // = per min
  }
  public double getBottomShooterRPM() {
    return (bottomMotor.getSelectedSensorVelocity() * 600.0) / Constants.Shooter.shooterEncoderTicksPerRev; // rev per 100ms * 600 = rpm                                                                                       // = per min
  }

  // run roller motor at specified speed
  public void setRollerMotorSpeed(double speed){
   rollerMotor.set(ControlMode.PercentOutput, speed); // mode = PercentOutput, where 1 corresponds to 100% and -1 corresponds to -100%
  }

 /* 
  * Sets speed of the motor as a setpoint value.
  * @param speed the specified speed to set the motor to
  */
  public void setTopMotorSpeed(double speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    topMotor.set(ControlMode.PercentOutput, speed);
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
    setBottomMotorSpeed(0);
  }

  public void stopBothMotors() {
    stopTopMotor();
    stopBottomMotor();
  }

  // feed forward control to get flywheel spinning at desired rpm
  public void setTopClosedLoopVelocity(double rpm) {
    rpm -= 225;
    topMotor.set(
        ControlMode.Velocity,
        (rpm / 60.0) * 2048 * 0.1,
        DemandType.ArbitraryFeedForward,
        topMotorFF.calculate(rpm / 60.0) / 12.0
    );
  }

  public void setBottomClosedLoopVelocity(double rpm) {
    rpm -= 225;
    bottomMotor.set(
        ControlMode.Velocity,
        (rpm / 60.0) * 2048 * 0.1,
        DemandType.ArbitraryFeedForward,
        bottomMotorFF.calculate(rpm / 60.0) / 12.0
    );
  }

  public void stopRollerMotor(){
    setRollerMotorSpeed(0.0);
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

  // getters
  public double getTopMotorOutput() {
    return topMotor.getMotorOutputPercent();
  }

  public double getBottomMotorOutput() {
    return bottomMotor.getMotorOutputPercent();
  }


  public double getTopTargetRPM() {
    if (!isRPMAdjusting()) return Constants.Shooter.FIXED_RPM * TOP_FLYWHEEL_CONST; // constant speed if not variable RPM
    return getRPMForDistanceInches(limelight.getAverageDistance()) * TOP_FLYWHEEL_CONST; // if variable RPM calculate using limelight distance and function
  }

  public void setBottomTestTargetRPM(double rpm) {
    bottomTestTargetRPM = rpm;
  }

  public double getTopTestTargetRPM() {
    return getBottomTestTargetRPM() * TOP_FLYWHEEL_CONST;
  }

  public double getBottomTestTargetRPM() {
    return bottomTestTargetRPM;
  }

  public double getBottomTargetRPM() {
    if (!isRPMAdjusting()) return Constants.Shooter.FIXED_RPM; // constant speed if not variable RPM
    return getRPMForDistanceInches(limelight.getAverageDistance()); // if variable RPM calculate using limelight distance and function
  }

  public double getBottomTargetRPM(double distance) {
    if (!isRPMAdjusting()) return Constants.Shooter.FIXED_RPM * TOP_FLYWHEEL_CONST;
    double average = (distance + currentDist)/2; // average out distance to get a better estimate of the target RPM
    return getRPMForDistanceInches(average);
  }

  // getters and setters
  public void setDistance(double newDist) {
    currentDist = newDist; 
  }

  public double getTopVoltage() {
    return topMotor.getBusVoltage();
  }

  public double getBottomVoltage() {
    return bottomMotor.getBusVoltage();
  }

  public boolean isRPMAdjusting() {
    return isAdjustingRPM; 
  }

  public void setAdjustingRPM(boolean value) {
    isAdjustingRPM = value; 
  }

  // function which maps distance to desired RPM
  public double getRPMForDistanceInches(double distance) {
    double rpm = (0.131271 * distance * distance) + (-19.4747 * distance) + 3779.39;
    if (rpm < 0) rpm = 0;
    if (rpm > Constants.Shooter.MAX_RPM) rpm = Constants.Shooter.MAX_RPM;
    return rpm; 
  }

  // sets ramping constant for both motors
  public void setRampingConstant(double ramp) {
    topMotor.configOpenloopRamp(ramp);
    bottomMotor.configOpenloopRamp(ramp);
  }

  // at target RPM for both top and bottom
  public boolean atTargetRPM(double marginOfError) {
    boolean topAtTarget = Math.abs(getTopShooterRPM() - getTopTargetRPM()) < marginOfError;
    boolean bottomAtTarget = Math.abs(getBottomShooterRPM() - getBottomTargetRPM()) < marginOfError;

    return topAtTarget && bottomAtTarget;
    
  }

  public boolean atTargetRPM() {
    return atTargetRPM(RPM_MOE);
  }

}

// CLEANED