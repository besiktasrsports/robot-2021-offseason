// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final WPI_VictorSPX climbMotor1 = new WPI_VictorSPX(60);
  private final WPI_VictorSPX climbMotor2 = new WPI_VictorSPX(61);

  private final DoubleSolenoid climbLock = new DoubleSolenoid(4,5);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    //climbMotor2.follow(climbMotor1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed){
    climbMotor1.set(speed);
  }

  public void lockClimber(){
    climbLock.set(Value.kForward);
  }

  public void releaseClimber(){
    climbLock.set(Value.kReverse);
  }
}
