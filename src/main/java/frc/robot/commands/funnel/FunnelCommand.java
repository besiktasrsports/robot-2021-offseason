// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FunnelSubsystem;

public class FunnelCommand extends CommandBase {
    /** Creates a new FunnelCommand. */
    private final FunnelSubsystem m_funnel;

    private final Double m_speed;
    private final Double m__speed;

    public FunnelCommand(FunnelSubsystem funnel, double speed, double _speed) {
        this.m_speed = speed;
        this.m_funnel = funnel;
        this.m__speed = _speed;
        addRequirements(m_funnel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_funnel.runFunnel(m_speed, m__speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_funnel.stopFunnel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
