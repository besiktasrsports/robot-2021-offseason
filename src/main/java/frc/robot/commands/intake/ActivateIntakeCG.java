// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeederCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActivateIntakeCG extends SequentialCommandGroup {
    /** Creates a new ActivateIntakeCG. */
    private final IntakeSubsystem m_intake;

    private final FeederSubsystem m_feeder;

    public ActivateIntakeCG(IntakeSubsystem intake, FeederSubsystem feeder, double speed) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_intake = intake;
        m_feeder = feeder;
        addCommands(
                new DropIntake(m_intake)
                        .withTimeout(.2)
                        .andThen(new OffIntake(m_intake).withTimeout(.1))
                        .andThen(new RunIntake(m_intake, speed))
                        .alongWith(new FeederCommand(m_feeder, -0.6, true)));
    }

    @Override
    public void end(boolean interrupted) {

        super.end(interrupted);
    }
}
