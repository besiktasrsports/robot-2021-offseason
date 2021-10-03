// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class JoystickDriveCommand extends CommandBase {
    /** Creates a new JoystickDriveCommand. */
    private final DriveSubsystem m_drive;

    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    public JoystickDriveCommand(DriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        m_left = left;
        m_right = right;
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(m_left.getAsDouble(), m_right.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
