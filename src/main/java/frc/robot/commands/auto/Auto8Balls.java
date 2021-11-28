// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.commands.feeder.FeedCG;
import frc.robot.commands.intake.ActivateIntakeCG;
import frc.robot.commands.intake.DropIntake;
import frc.robot.commands.shooter.SetShooterRPMPF;
import frc.robot.commands.turret.TurretPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto8Balls extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public Auto8Balls(SneakyTrajectory s_trajectory, DriveSubsystem m_drivetrain, IntakeSubsystem m_intake, TurretSubsystem m_turret, ShooterSubsystem m_shooter, FeederSubsystem m_feeder, FunnelSubsystem m_funnel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new TurretPIDCommand(m_turret,true).withTimeout(1).alongWith(new SetShooterRPMPF(2900, m_shooter, true)).raceWith(new DropIntake(m_intake))
    , new FeedCG(m_shooter, m_feeder, m_intake, m_funnel).withTimeout(2).raceWith(new SetShooterRPMPF(2900, m_shooter, false))
    , new DriveStraightMeters(m_drivetrain, -5.8, 0,1).raceWith(new ActivateIntakeCG(m_intake, m_feeder,0.7)).andThen(new DriveStraightMeters(m_drivetrain, 5.4, 0,5))
    , new TurretPIDCommand(m_turret,true).withTimeout(1).alongWith(new SetShooterRPMPF(2900, m_shooter, true)),new FeedCG(m_shooter,m_feeder,m_intake,m_funnel).withTimeout(5).raceWith(new SetShooterRPMPF(2900, m_shooter, false)));
  }
    /*

    super(s_trajectory.getRamsete(s_trajectory.testAuto[0]).raceWith(new RunIntake(m_intake, 0.7))
    );
  }
  */
  // .andThen(s_trajectory.getRamsete(s_trajectory.testAuto[1]).raceWith(new RunIntake(m_intake, 0)).andThen(()-> m_drivetrain.tankDriveVolts(0, 0)))
}
