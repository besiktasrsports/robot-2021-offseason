// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.sneakylib.auto.AdaptivePurePursuitController;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class APPCPathFollowerLeft extends PIDCommand {
  /** Creates a new APPCPathFollower. */
  private DriveSubsystem m_drive;
  private AdaptivePurePursuitController m_appc;
  private static double m_leftMotorOutput;
  public APPCPathFollowerLeft(DriveSubsystem drive, AdaptivePurePursuitController appc, Trajectory trajectory, boolean reversed) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kPathFollowP, DriveConstants.kPathFollowI, DriveConstants.kPathFollowD),
        // This should return the measurement
        drive::getLeftWheelVelocity,
        // This should return the setpoint (can also be a constant)
        appc.update(trajectory, drive.getPose(), Math.toRadians(drive.getHeading()), reversed)[0],
        // This uses the output
        output -> {
            m_leftMotorOutput = output;
            System.out.println(m_leftMotorOutput);
          // Use the output here
        });
        getController().setTolerance(0.1);
        m_drive = drive;
        m_appc = appc;

        addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public void initialize() {
    
    super.initialize();
    m_leftMotorOutput = 0;
  }
  @Override
  public void execute() {
    
    super.execute();
  }
  @Override
  public boolean isFinished() {
    if(getController().atSetpoint()){
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_leftMotorOutput = 0;
  }
}
