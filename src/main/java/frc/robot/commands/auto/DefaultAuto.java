// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultAuto extends SequentialCommandGroup {
  public DefaultAuto(DriveSubsytem drive, ShooterSubsystem shooter) {
    // TODO: Change this to a reasonable auto
    super(new RunShooter(shooter, 0.0).withTimeout(3).andThen(() -> drive.tankDriveVolts(0.0, 0.0)));
  }
}
