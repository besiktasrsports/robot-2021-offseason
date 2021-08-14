// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionLed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionLED;

public class CloseLED extends CommandBase {
  private final VisionLED m_led;
  /** Creates a new CloseLED. */
  public CloseLED(VisionLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.m_relay.set(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.m_relay.set(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.m_relay.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
