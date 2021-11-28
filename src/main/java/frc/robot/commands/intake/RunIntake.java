// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
    private final double m_speed;
    private final IntakeSubsystem m_intake;
    private boolean m_toFeed;
    private boolean shooterAtSetpoint;

    public RunIntake(IntakeSubsystem intake, double speed, boolean toFeed) {
        this.m_speed = speed;
        this.m_intake = intake;
        m_toFeed = toFeed;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterAtSetpoint = Robot.m_robotContainer.m_shooter.isAtSetpoint;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterAtSetpoint = Robot.m_robotContainer.m_shooter.isAtSetpoint;
        if (m_toFeed) {
            if (shooterAtSetpoint && Robot.m_robotContainer.m_turret.isAtSetpoint) {
                m_intake.runIntake(m_speed);
            } else {
                m_intake.runIntake(0);
            }
        } else {
            m_intake.runIntake(m_speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.runIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
