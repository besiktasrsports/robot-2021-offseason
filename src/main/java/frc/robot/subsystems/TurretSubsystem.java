// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase implements AutoCloseable {

  private final WPI_VictorSPX turretMotor = new WPI_VictorSPX(TurretConstants.kTurretMotorPort);
  public final PWMVictorSPX testMotorUT = new PWMVictorSPX(0);
  public final Encoder turretEncoder =
      new Encoder(
          TurretConstants.kTurretEncoderA,
          TurretConstants.kTurretEncoderB,
          TurretConstants.kIsEncoderReversed);

  public TurretSubsystem() {

    turretMotor.setInverted(TurretConstants.kIsMotorReversed);
    turretMotor.setNeutralMode(TurretConstants.kTurretMotorMode);

    turretEncoder.setDistancePerPulse(1.0 / (TurretConstants.kTurretEncoderPPR));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runTurret(double speed) {

    turretMotor.set(speed);
  }

  public void turretTestMethod(double speed){

    testMotorUT.set(speed);
  }
  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub

  }
}
