// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends CommandBase {
  /** Creates a new AutoIntake. */
  private final IntakeSubsystem m_intake;
  private final double m_speed;
  public AutoIntake(IntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = intake;
    this.m_speed = speed;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //This empty because it's using Autonomous
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
