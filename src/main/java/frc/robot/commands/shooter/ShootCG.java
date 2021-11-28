// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.commands.feeder.FeedCG;
import frc.robot.commands.turret.TurretPIDCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCG extends SequentialCommandGroup {
    private ShooterSubsystem m_shooter;
    private TurretSubsystem m_turret;
    private FeederSubsystem m_feeder;
    private IntakeSubsystem m_intake;
    private FunnelSubsystem m_funnel;

    // Use with "whileHeld" !!

    /** Creates a new ShootCG. */
    public ShootCG(
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            FeederSubsystem feeder,
            FunnelSubsystem funnel,
            IntakeSubsystem intake) {
        Robot.robotState = RobotState.SHOOT;

        m_shooter = shooter;
        m_turret = turret;
        m_feeder = feeder;
        m_intake = intake;
        m_funnel = funnel;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new TurretPIDCommand(m_turret, false)
                        .alongWith(
                                new SetShooterRPMPF(2900, shooter, false)
                                        .alongWith(new FeedCG(m_shooter, m_feeder, m_intake, m_funnel))));
    }
}
