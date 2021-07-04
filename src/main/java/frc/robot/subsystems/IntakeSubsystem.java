// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);
    private final Compressor compressor = new Compressor(IntakeConstants.kCompressorPort);
    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(IntakeConstants.kIntakeDoubleSolenoidPort1,IntakeConstants.kIntakeDoubleSolenoidPort2);
    public boolean compressorState = false;
    public boolean intakeState = false;

  public IntakeSubsystem() {
    compressor.setClosedLoopControl(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double m_speed) {
    intakeMotor.set(m_speed);
  }

  public void openCompressor() {
    compressor.setClosedLoopControl(true);
  }

  public void closeCompressor() {
    compressor.setClosedLoopControl(false);
  }
  public void intakeUp() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeDown() {
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeOff() {
    intakeSolenoid.set(Value.kOff);
  }


}
