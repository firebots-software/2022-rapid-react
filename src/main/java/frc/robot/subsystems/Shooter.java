// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private Limelight limelight;
  private TalonSRX rollerMotor;
  private TalonFX topMotor, bottomMotor; 
  private double topTestTargetRPM, bottomTestTargetRPM;
  private final double MAX_SPEED = 1;
  private final double RAMPING_CONSTANT = 0.25;
  private final double TOP_FLYWHEEL_CONST = 0.8;
  private boolean isAdjustingRPM; 
  private SimpleMotorFeedforward topMotorFF, bottomMotorFF;

  /** Creates a new Shooter. */
  private Shooter() {
    this.topMotor = new TalonFX(Constants.Shooter.shooterTopMotorPort);
    topMotor.configFactoryDefault();
    topMotor.configOpenloopRamp(RAMPING_CONSTANT);
    topMotor.setInverted(true);
    topMotor.configVoltageCompSaturation(12);
    topMotor.enableVoltageCompensation(true);

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

    topMotorFF = new SimpleMotorFeedforward(Constants.Shooter.ksTopFlywheel, Constants.Shooter.kvTopFlywheel, Constants.Shooter.kaTopFlywheel);
    bottomMotorFF = new SimpleMotorFeedforward(Constants.Shooter.ksBottomFlywheel, Constants.Shooter.kvBottomFlywheel, Constants.Shooter.kaBottomFlywheel);

    isAdjustingRPM = false; 

    limelight = Limelight.getInstance();

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

    topMotor.set(ControlMode.PercentOutput, speed);
    // value to move to aimed point
  }

  public void setTopMotorVoltage(double volts) {
    topMotor.set(ControlMode.Current, volts);
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

  public void setTopClosedLoopVelocity(double rpm) {
    rpm -= 250;
    topMotor.set(
        ControlMode.Velocity,
        (rpm / 60.0) * 2048 * 0.1,
        DemandType.ArbitraryFeedForward,
        topMotorFF.calculate(rpm / 60.0) / 12.0
    );
  }

  public void setBottomClosedLoopVelocity(double rpm) {
    rpm -= 250;
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

  // ask command or method

  /*
   * public void setAtTargetSpeed(boolean atTarget) {
   * this.atTargetSpeed = atTarget;
   * }
   */
  // add threshold for target speed


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
    if (!isRPMAdjusting()) return Constants.Shooter.FIXED_RPM;
    return getRPMForDistanceInches(limelight.getDistanceToTarget());
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
    if (!isRPMAdjusting()) return Constants.Shooter.FIXED_RPM;
    return getRPMForDistanceInches(limelight.getDistanceToTarget());
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

  public double getRPMForDistanceInches(double distance) {
    double rpm = (0.0501976 * (distance + 25.8149) * (distance + 25.8149)) + 2142.69; 
    if (rpm > Constants.Shooter.MAX_RPM) rpm = Constants.Shooter.MAX_RPM;
    return rpm; 
  }

  public void setRampingConstant(double ramp) {
    topMotor.configOpenloopRamp(ramp);
    bottomMotor.configOpenloopRamp(ramp);
  }
}
