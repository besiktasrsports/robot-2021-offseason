// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.commands.intake.ActivateIntakeCG;
import frc.robot.commands.shooter.ShootCG;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto7BallSteal extends SequentialCommandGroup {
  /** Creates a new Auto5BallSteal. */
  public Auto7BallSteal(SneakyTrajectory s_trajectory, IntakeSubsystem intake, FeederSubsystem feeder, DriveSubsystem drive, ShooterSubsystem shooter, FunnelSubsystem funnel, TurretSubsystem turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(s_trajectory.getRamsete(s_trajectory.steal7Balls[0]).raceWith(new ActivateIntakeCG(intake, feeder, 0.7)),
    s_trajectory.getRamsete(s_trajectory.steal7Balls[1]).andThen(() -> drive.tankDriveVolts(0, 0)),
    new ShootCG(shooter, turret, feeder, funnel, intake).withTimeout(4), s_trajectory.getRamsete(s_trajectory.steal7Balls[2]).raceWith(new ActivateIntakeCG(intake,feeder,0.7)),
    s_trajectory.getRamsete(s_trajectory.steal7Balls[3]).andThen(() -> drive.tankDriveVolts(0, 0)),
    new ShootCG(shooter, turret, feeder, funnel, intake)
    );
  }
}
