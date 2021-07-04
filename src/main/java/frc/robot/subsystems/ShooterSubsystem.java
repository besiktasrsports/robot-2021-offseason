// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotor1Port);
  private WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotor2Port);

  public ShooterSubsystem() {
    shooterMotor1.setInverted(ShooterConstants.kShooterInvertedMode1);
    shooterMotor2.setInverted(ShooterConstants.kShooterInvertedMode2);
    shooterMotor2.follow(shooterMotor1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed);
  }
}
