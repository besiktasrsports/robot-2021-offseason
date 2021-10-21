// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.sneakylib.auto.AdaptivePurePursuitController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class APPCCG extends ParallelCommandGroup {
    /** Creates a new APPCCG. */
    DriveSubsystem m_drive;

    AdaptivePurePursuitController m_appc;
    Trajectory m_trajectory;
    boolean m_reversed;

    public APPCCG(
            DriveSubsystem drive,
            AdaptivePurePursuitController appc,
            Trajectory trajectory,
            boolean reversed) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_drive = drive;
        m_appc = appc;
        m_trajectory = trajectory;
        m_reversed = reversed;

        addCommands(
                new APPCPathFollowerLeft(m_drive, m_appc, m_trajectory, m_reversed)
                        .alongWith(new APPCPathFollowerRight(m_drive, m_appc, m_trajectory, m_reversed)));
    }
}
