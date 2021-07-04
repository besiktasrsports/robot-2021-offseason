// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.subsystems.DriveSubsytem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BarrelRacingPath extends SequentialCommandGroup {
  /** Creates a new PathBTogether. */
  public BarrelRacingPath(SneakyTrajectory s_trajectory, DriveSubsytem robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[0]), s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[1]) ,
    s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[2]), s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[3]), s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[4]),
    s_trajectory.getRamsete(s_trajectory.BarrelRacingPath[5]).andThen(() -> robotDrive.tankDriveVolts(0, 0)));
  }
}
