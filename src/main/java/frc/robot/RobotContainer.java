// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.drivetrain.JoystickDriveCommand;
import frc.robot.commands.feeder.FeederCommand;
import frc.robot.commands.funnel.FunnelCommand;
import frc.robot.commands.intake.ActivateIntakeCG;
import frc.robot.commands.intake.ToggleCompressor;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.SetShooterRPMPF;
import frc.robot.commands.turret.TurretBangBangControl;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionLED;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
    public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);
    public final FunnelSubsystem m_funnel = new FunnelSubsystem();
    public final TurretSubsystem m_turret = new TurretSubsystem();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();
    public final ShooterSubsystem m_shooter = new ShooterSubsystem();
    public final FeederSubsystem m_Feeder = new FeederSubsystem();
    public final DriveSubsytem m_robotDrive = new DriveSubsytem();
    public final VisionLED m_VisionLED = new VisionLED();
    public final SneakyTrajectory s_trajectory = new SneakyTrajectory(m_robotDrive);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        m_robotDrive.setDefaultCommand(
                new JoystickDriveCommand(
                        m_robotDrive,
                        () -> -m_driverController.getRawAxis(1),
                        () -> -m_driverController.getRawAxis(0)));
    }

    private void configureButtonBindings() {

        // Turret Commands
        new JoystickButton(m_driverController, 2).whileHeld(new TurretBangBangControl(m_turret));
        new JoystickButton(m_driverController, 10).whileHeld(new TurretJoystickCommand(m_turret, -0.2));
        // Funnel Commands
        new JoystickButton(m_driverController, 4).whileHeld(new FunnelCommand(m_funnel, -0.5, -0.5));
        new JoystickButton(m_driverController, 9).whileHeld(new FunnelCommand(m_funnel, 0.5, -0.5));
        // new JoystickButton(m_driverController, 6).whileHeld(new FunnelCommand(m_funnel, -0.5, 0.5));

        // Intake Commands
        new JoystickButton(m_driverController, 1).toggleWhenPressed(new ActivateIntakeCG(m_intake));
        // new JoystickButton(m_operatorController, 1).whileHeld(new ToggleDropIntake(m_intake));

        // Shooter Commands
        new JoystickButton(m_driverController, 3).whileHeld(new RunShooter(m_shooter, -0.65));
        new JoystickButton(m_driverController, 6)
                .toggleWhenPressed(new SetShooterRPMPF(2550, m_shooter, false)); // 2450 2750

        // Feeder Commands
        new JoystickButton(m_driverController, 5).whileHeld(new FeederCommand(m_Feeder, -0.8));
        // new JoystickButton(m_driverController, 6).whenPressed(new FeederCommand(m_Feeder,
        // -0.8).withTimeout(0.2));
        // Misc Commands
        new JoystickButton(m_driverController, 8).whileHeld(new ToggleCompressor(m_intake));
        // new JoystickButton(m_driverController, 7).whenPressed(new DisableIntakeCG(m_intake));
        // new JoystickButton(m_driverController, 10).whenPressed(new ToggleLED(m_VisionLED));

        // Vision Drive
        // new JoystickButton(m_driverController, 3).whileHeld(new CloseLED(m_VisionLED));
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new TestAuto(s_trajectory, m_robotDrive, m_intake);
    }
}
