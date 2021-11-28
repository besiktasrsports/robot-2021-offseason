// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
    /** Creates a new AccelaratorCommand. */
    private final FeederSubsystem m_accelarator;

    private final double m_speed;
    private boolean m_toFeed;
    private boolean shooterAtSetpoint;

    public FeederCommand(FeederSubsystem accelarator, double speed, boolean toFeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_accelarator = accelarator;
        m_speed = speed;
        m_toFeed = toFeed;
        addRequirements(m_accelarator);
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
                m_accelarator.runFeeder(m_speed);
            } else {
                m_accelarator.runFeeder(0);
            }
        } else {
            m_accelarator.runFeeder(m_speed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_accelarator.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!m_toFeed) {
            return m_accelarator.getSensorStatus();
        }
        return false;
    }
}
