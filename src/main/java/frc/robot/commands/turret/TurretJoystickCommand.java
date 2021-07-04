// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretJoystickCommand extends CommandBase {
  /** Creates a new TurretJoystickCommand. */
  private final double speed;

  private final TurretSubsystem m_turret;

  public TurretJoystickCommand(TurretSubsystem _turret, double _speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = _speed;
    this.m_turret = _turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_turret.runTurret(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
