// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActivateIntakeCG extends ParallelCommandGroup {
  /** Creates a new ActivateIntakeCG. */
  private final IntakeSubsystem m_intake;
  public ActivateIntakeCG(IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intake = intake;
    addCommands(new DropIntake(m_intake).withTimeout(.2).andThen(new OffIntake(m_intake).withTimeout(.2)).andThen(new RunIntake(m_intake, 0.7)));
  }

  @Override
  public void end(boolean interrupted) {
    
    super.end(interrupted);
  }
}
