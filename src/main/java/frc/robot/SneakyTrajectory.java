/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/** Add your docs here. */
public class SneakyTrajectory {

    private DriveSubsystem m_drive;
    public Trajectory[] testAuto = new Trajectory[2];
    public Trajectory[] steal7Balls = new Trajectory [4];

    public SneakyTrajectory(DriveSubsystem drive) {

        m_drive = drive;
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                        DriveConstants.kDriveKinematics,
                        DriveConstants.kMaxAutoVoltage);
        TrajectoryConfig configForward =
                new TrajectoryConfig(
                                DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        configForward.setReversed(false);
        TrajectoryConfig configBackward =
                new TrajectoryConfig(
                                DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        configBackward.setReversed(true);

        testAuto[0] =
                TrajectoryGenerator.generateTrajectory(
                        List.of(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                new Pose2d(5, 1, new Rotation2d(Math.toRadians(0)))),
                        configForward);
        testAuto[1] =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(-4, 0, new Rotation2d(0)),
                        List.of(new Translation2d(-2, -0.5)),
                        new Pose2d(-0.5, -1, new Rotation2d(Math.toRadians(0))),
                        configForward);

        steal7Balls[0] =  TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(3.2, 2.55, new Rotation2d(3.142)),
                    new Pose2d(6.155, 0.76, new Rotation2d(2.555))
                ),configBackward);

        steal7Balls[1] = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(6.155, 0.76, new Rotation2d(2.555)),
                        new Pose2d(4.172, 2.205, new Rotation2d(2.22)),
                        new Pose2d(2.6, 4.659, new Rotation2d(2.51))
                ),configForward);

        steal7Balls[2] = TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(2.6, 4.659, new Rotation2d(2.548)),
                new Pose2d(6.912, 4.184, new Rotation2d(-2.822))
                ),configBackward);

        steal7Balls[3] = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(6.912, 4.184, new Rotation2d(-2.81)),
                        new Pose2d(2.6, 4.659, new Rotation2d(2.548))
                ),configForward);
                        
                        
                

        
    }

    public RamseteCommand getRamsete(Trajectory trajectory) {

        return new RamseteCommand(
                trajectory,
                m_drive::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_drive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_drive::tankDriveVolts,
                m_drive);
    }
}
