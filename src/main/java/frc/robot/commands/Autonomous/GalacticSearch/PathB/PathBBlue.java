// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathBBlue extends SequentialCommandGroup {
  /** Creates a new PathBBlue. */
  public PathBBlue(SneakyTrajectory s_trajectory, DriveSubsytem robotDrive, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new AutoIntake(intake, 0.5).raceWith(s_trajectory.getRamsete(s_trajectory.PathBBlue[0])
    .andThen(() -> robotDrive.tankDriveVolts(0, 0))), new AutoIntake(intake, 0)
    );
  }
}
