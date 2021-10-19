// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAtSetpoint = false;
    private int i;
    private double rpmSum;
    private WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotor1Port);
    private WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotor2Port);

    public final Encoder shooterEncoder =
            new Encoder(
                    ShooterConstants.kShooterEncoderA,
                    ShooterConstants.kShooterEncoderB,
                    ShooterConstants.kShooterEncoderIsReversed);

    public ShooterSubsystem() {
        shooterEncoder.setDistancePerPulse(1.0 / (ShooterConstants.kShooterEncoderPPR));
        shooterMotor1.setInverted(ShooterConstants.kShooterInvertedMode1);
        shooterMotor2.setInverted(ShooterConstants.kShooterInvertedMode2);
        shooterMotor2.follow(shooterMotor1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // System.out.println("RPM : " + getRPM());
        /*
        if(i == 50){
        System.out.println("Avg RPM : " + rpmSum/50);
        rpmSum = 0;
        i = 0;
        }
        rpmSum += getRPM();
        i++;

        */

        SmartDashboard.putNumber("shooter/rpm", getRPM());
    }

    public void runShooter(double speed) {
        shooterMotor1.set(speed);
    }

    public void runShooterVoltage(double voltage) {
        shooterMotor1.setVoltage(voltage);
    }

    public double getRPM() {
        return shooterEncoder.getRate() * 60;
    }
}
