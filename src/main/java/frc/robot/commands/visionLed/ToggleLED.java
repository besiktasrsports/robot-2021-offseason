// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionLed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionLED;

public class ToggleLED extends CommandBase {
  private final VisionLED m_led;
  /** Creates a new ToggleLED. */
  public ToggleLED(VisionLED led) {
    m_led = led;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.toggleRelay(!m_led.m_relay.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
