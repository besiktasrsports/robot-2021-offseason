// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsytem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionTurnPF extends PIDCommand {
    /** Creates a new VisionTurnPF. */
    private DriveSubsytem m_drive;

    public VisionTurnPF(DriveSubsytem drive) {
        super(
                // The controller that the command will use
                new PIDController(DriveConstants.kVisionTurnP, 0, 0),
                // This should return the measurement
                () -> Robot.getVisonYawAngle(),
                // This should return the setpoint (can also be a constant)
                () -> 0,
                // This uses the output
                output -> {
                    drive.arcadeDrive(
                            0,
                            (Robot.isValidAngle())
                                    ? ((output > 0)
                                            ? -DriveConstants.kVisionMinCommand - output
                                            : DriveConstants.kVisionMinCommand - output)
                                    : 0);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        m_drive = drive;
        addRequirements(m_drive);
        getController().setTolerance(DriveConstants.kTurnToleranceDeg);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Robot.isValidAngle() && getController().atSetpoint());
    }
}
