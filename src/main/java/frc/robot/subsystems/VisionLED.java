// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;

public class VisionLED extends SubsystemBase {
  /** Creates a new VisionLED. */
  private boolean ledState;
  public final DigitalOutput m_relay = new DigitalOutput(MiscConstants.kLedRelayPort);
  public VisionLED() {
    toggleRelay(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledState = !m_relay.get();
    SmartDashboard.putBoolean("led/state", ledState);
  }

  public void toggleRelay(boolean _status) {
    m_relay.set(_status);
  }
}
