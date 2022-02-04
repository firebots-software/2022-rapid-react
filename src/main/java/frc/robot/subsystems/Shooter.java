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
  private Solenoid piston; 
  private TalonSRX motor;
  private boolean atTargetSpeed;

  
  /** Creates a new Shooter.  */
  private Shooter() {
    this.piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Shooter.shooterPistonPort);
    this.motor = new TalonSRX(Constants.Shooter.shooterMotorPort);
    atTargetSpeed = false;

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

public double getRPM() {
	return (motor.getSelectedSensorVelocity() * 600.0) /Constants.Shooter.shooterEncoderTicksPerRev;  //per 100ms * 600 = per min
 }

public void setSpeed(double speed) {
  motor.set(ControlMode.PercentOutput, speed);
  // value to move to aimed point
}

public void stopMotor() {
  setSpeed(0);
}


//ask command or method


public void extendPiston(){
  piston.set(true);
}

public void retractPiston(){
  piston.set(false);
}

public void togglePiston(){
  piston.toggle();
}

public void setAtTargetSpeed(boolean atTarget) {
  this.atTargetSpeed = atTarget;
}

public boolean isAtTargetSpeed() {
  return atTargetSpeed;
}

//extend/ retract piston --> binds to button 




}
