// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.funnel.FunnelCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedCG extends SequentialCommandGroup {
    /** Creates a new FeedCG. */
    private ShooterSubsystem m_shooter;
    private FeederSubsystem m_feeder;
    private IntakeSubsystem m_intake;
    private FunnelSubsystem m_funnel;


    public FeedCG(ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake, FunnelSubsystem funnel) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_intake = intake;
        m_feeder = feeder;
        m_shooter = shooter;
        m_funnel = funnel;

            addCommands(
                    new FeederCommand(m_feeder, -0.8, false)
                            .raceWith(new RunIntake(m_intake, 0))
                            .withTimeout(0.4), new FeederCommand(m_feeder, -0.8, false)
                            .alongWith(new RunIntake(m_intake, 0.7)).alongWith(new FunnelCommand(m_funnel, -0.5, -0.5))
                            
                            );
        
        
    }
}
