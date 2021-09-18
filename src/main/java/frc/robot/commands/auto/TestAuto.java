// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto(SneakyTrajectory s_trajectory, DriveSubsytem m_drivetrain, IntakeSubsystem m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(s_trajectory.getRamsete(s_trajectory.testAuto[0]).raceWith(new RunIntake(m_intake, 0.7))
    );
  }
  // .andThen(s_trajectory.getRamsete(s_trajectory.testAuto[1]).raceWith(new RunIntake(m_intake, 0)).andThen(()-> m_drivetrain.tankDriveVolts(0, 0)))
}
