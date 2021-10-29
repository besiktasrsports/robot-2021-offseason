// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.auto.Auto8Balls;
import frc.robot.commands.auto.DefaultAuto;
import frc.robot.commands.climb.LockClimber;
import frc.robot.commands.climb.ReleaseClimber;
import frc.robot.commands.climb.RunClimber;
import frc.robot.commands.drivetrain.JoystickDriveCommand;
import frc.robot.commands.feeder.FeedCG;
import frc.robot.commands.intake.ActivateIntakeCG;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleCompressor;
import frc.robot.commands.shooter.SetShooterRPMPF;
import frc.robot.commands.turret.TurretInterruptor;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.commands.turret.TurretPIDCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.sneakylib.drivers.WS2812LEDDriver;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
    public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);
    public final FunnelSubsystem m_funnel = new FunnelSubsystem();
    public final ClimbSubsystem m_climb = new ClimbSubsystem();
    public final TurretSubsystem m_turret = new TurretSubsystem();
    // public final AdaptivePurePursuitController m_appc = new AdaptivePurePursuitController();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();
    public final ShooterSubsystem m_shooter = new ShooterSubsystem();
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final FeederSubsystem m_Feeder = new FeederSubsystem();
    public final WS2812LEDDriver m_ledDriver = new WS2812LEDDriver(1, 21);
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

        new JoystickButton(m_driverController, 4).whileHeld(new RunIntake(m_intake, -0.7));
        // Turret Commands
        new JoystickButton(m_driverController, 3).whileHeld(new TurretPIDCommand(m_turret));
        new JoystickButton(m_driverController, 2).whileHeld(new TurretInterruptor(m_turret));

        new JoystickButton(m_operatorController, 9).whileHeld(new TurretJoystickCommand(m_turret, 0.3));
        new JoystickButton(m_operatorController, 10)
                .whileHeld(new TurretJoystickCommand(m_turret, -0.3));
        // Intake Commands
        new JoystickButton(m_driverController, 1)
                .toggleWhenPressed(new ActivateIntakeCG(m_intake, m_Feeder, 1));
        // Shooter Commands
        new JoystickButton(m_driverController, 6)
                .toggleWhenPressed(new SetShooterRPMPF(2900, m_shooter, false)); // 2900
        // Feeder Commands
        new JoystickButton(m_driverController, 5)
                .whileHeld(new FeedCG(m_shooter, m_Feeder, m_intake, m_funnel));
        // Misc Commands
        new JoystickButton(m_driverController, 8).whileHeld(new ToggleCompressor(m_intake));

        // Climb Commands
        new JoystickButton(m_operatorController, 11).whileHeld(new RunClimber(m_climb, 1));
        new JoystickButton(m_operatorController, 12).whileHeld(new RunClimber(m_climb, -1));

        new JoystickButton(m_operatorController, 1).whileHeld(new LockClimber(m_climb));
        new JoystickButton(m_operatorController, 7).whileHeld(new ReleaseClimber(m_climb));

        /**
        * Use this to pass the autonomous command to the main {@link Robot} class.
        *
        * @return the command to run in autonomous
        */
    }

    public Command getAutonomousCommand(Integer auto) {
        // An ExampleCommand will run in autonomous
        switch (auto) {
            case 1:
                return new DefaultAuto(m_robotDrive, m_shooter, m_Feeder, m_intake, m_funnel, m_turret);
            default:
                return new Auto8Balls(
                        s_trajectory, m_robotDrive, m_intake, m_turret, m_shooter, m_Feeder, m_funnel);
        }
    }
}
