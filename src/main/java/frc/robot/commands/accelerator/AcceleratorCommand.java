// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.accelerator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AcceleratorSubsystem;

public class AcceleratorCommand extends CommandBase {
    /** Creates a new AccelaratorCommand. */
    private final AcceleratorSubsystem m_accelarator;

    private final double m_speed;

    public AcceleratorCommand(AcceleratorSubsystem accelarator, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_accelarator = accelarator;
        m_speed = speed;
        addRequirements(m_accelarator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_accelarator.runAccelerator(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_accelarator.stopAccelerator();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
