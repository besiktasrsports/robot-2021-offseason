// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeedCG;
import frc.robot.commands.shooter.SetShooterRPMPF;
import frc.robot.commands.turret.TurretPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DefaultAuto extends SequentialCommandGroup {
    public DefaultAuto(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            IntakeSubsystem intake,
            FunnelSubsystem funnel,
            TurretSubsystem turret) {
        // TODO: Change this to a reasonable auto
        super(
                new TurretPIDCommand(turret)
                        .withTimeout(1)
                        .alongWith(new SetShooterRPMPF(2900, shooter, true)),
                new SetShooterRPMPF(2900, shooter, false)
                        .raceWith(new FeedCG(shooter, feeder, intake, funnel))
                        .withTimeout(3)
                        .andThen(
                                () -> {
                                    drive.arcadeDrive(0.7, 0);
                                })
                        .withTimeout(2)
                        .andThen(
                                () -> {
                                    drive.arcadeDrive(0, 0);
                                }));
    }
}
