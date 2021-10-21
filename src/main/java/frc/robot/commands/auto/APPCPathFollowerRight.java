package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.sneakylib.auto.AdaptivePurePursuitController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class APPCPathFollowerRight extends PIDCommand {
    /** Creates a new APPCPathFollower. */
    private DriveSubsystem m_drive;

    private AdaptivePurePursuitController m_appc;
    private static double m_rightMotorOutput;

    public APPCPathFollowerRight(
            DriveSubsystem drive,
            AdaptivePurePursuitController appc,
            Trajectory trajectory,
            boolean reversed) {
        super(
                // The controller that the command will use
                new PIDController(
                        DriveConstants.kPathFollowP, DriveConstants.kPathFollowI, DriveConstants.kPathFollowD),
                // This should return the measurement
                drive::getRightWheelVelocity,
                // This should return the setpoint (can also be a constant)
                1.0,
                // appc.update(trajectory, drive.getPose(), Math.toRadians(drive.getHeading()),
                // reversed)[1],

                // This uses the output
                output -> {
                    m_rightMotorOutput = output;
                    // System.out.print("Right : " + m_rightMotorOutput);
                    drive.leftRearMotor.setVoltage(12 * m_rightMotorOutput);
                    // Use the output here
                });
        getController().setTolerance(0);
        m_drive = drive;
        m_appc = appc;

        addRequirements();
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Returns true when the command should end.
    @Override
    public void initialize() {

        super.initialize();
        m_rightMotorOutput = 0;
    }

    @Override
    public void execute() {

        super.execute();
    }

    @Override
    public boolean isFinished() {
        if (m_appc.isAtSetpoint) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_rightMotorOutput = 0;
    }
}
