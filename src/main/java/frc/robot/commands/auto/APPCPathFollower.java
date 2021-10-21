// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.sneakylib.auto.AdaptivePurePursuitController;
import frc.sneakylib.math.Conversions;

public class APPCPathFollower extends CommandBase {
    /** Creates a new APPCPathFollower. */
    private DriveSubsystem m_drive;

    private AdaptivePurePursuitController m_appc;
    private double leftCumulativeError, rightCumulativeError;
    private final double kTargetVelP = 1;
    private final double kMinOutput = 3.0;
    private final double kMaxOutput = 8.0;

    public APPCPathFollower(DriveSubsystem drive, Trajectory trajectory) {
        m_drive = drive;
        m_appc = new AdaptivePurePursuitController(drive);
        m_appc.init(trajectory);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftCumulativeError = 0;
        rightCumulativeError = 0;
        m_appc.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double[] target_velocities = m_appc.update();
        double leftTargetVelocity = target_velocities[1] * kTargetVelP;
        double rightTargetVelocity = target_velocities[0] * kTargetVelP;
        double leftError = -m_drive.getLeftWheelVelocity() + leftTargetVelocity;
        double rightError = -m_drive.getRightWheelVelocity() + rightTargetVelocity;
        leftCumulativeError += leftError;
        rightCumulativeError += rightError;
        double leftOutput =
                DriveConstants.kPathFollowP * leftError + DriveConstants.kPathFollowI * leftCumulativeError;
        double rightOutput =
                DriveConstants.kPathFollowP * rightError
                        + DriveConstants.kPathFollowI * rightCumulativeError;
        // System.out.println("Left Error: " + leftError + " Left Output: " + leftOutput);
        // System.out.println("Right Error: " + rightError + " Right Output: " + -rightOutput);
        if (leftTargetVelocity > 0) {
            leftOutput = Conversions.clamp(leftOutput, kMinOutput, kMaxOutput);
        } else if (leftTargetVelocity < 0) {
            leftOutput = Conversions.clamp(leftOutput, -kMaxOutput, -kMinOutput);
        }
        if (rightTargetVelocity > 0) {
            rightOutput = Conversions.clamp(rightOutput, kMinOutput, kMaxOutput);
        } else if (rightTargetVelocity < 0) {
            rightOutput = Conversions.clamp(rightOutput, -kMaxOutput, -kMinOutput);
        }
        System.out.println("Left Output : " + leftOutput);
        System.out.println("Right Output : " + rightOutput);
        // System.out.println("Left Target : " + leftTargetVelocity);
        // System.out.println("Right Target : " + rightTargetVelocity);
        System.out.println("Goal:" + m_appc.getPointPose());
        System.out.println("Current:" + m_drive.getPose());
        m_drive.rightRearMotor.setVoltage(-1 * leftOutput);
        m_drive.leftRearMotor.setVoltage(rightOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return m_appc.isAtSetpoint;
    }
}
