// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    NetworkTableInstance photon = NetworkTableInstance.create();
    public static RobotState robotState;
    public static NetworkTableEntry angle;
    public static NetworkTableEntry validAngle;
    public static NetworkTableInstance inst;
    NetworkTable table = photon.getTable("photonvision").getSubTable("microsoftlifecam");
    private Command m_autonomousCommand;
    public static RobotContainer m_robotContainer;
    public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
    // public static PhotonPipelineResult result;

    public static boolean ledCanStart = false;

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        photon.startClient("10.72.85.12");
        angle = table.getEntry("targetYaw");
        validAngle = table.getEntry("hasTarget");

        autoChooser.setDefaultOption("8 Balls Right Side", 0);
        autoChooser.addOption("3 Balls", 1);
        SmartDashboard.putData("Autonomous Selector", autoChooser);
        robotState = RobotState.IDLE;
        m_robotContainer.m_robotDrive.zeroHeading();
    }

    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        ledCanStart = true;
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {

        m_robotContainer.m_robotDrive.resetEncoders();
        m_robotContainer.m_robotDrive.zeroHeading();
        m_robotContainer.m_robotDrive.m_odometry.resetPosition(
                m_robotContainer.s_trajectory.testAuto[0].getInitialPose(), new Rotation2d(0));

        m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running whesn
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        CommandScheduler.getInstance().cancelAll();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.m_robotDrive.resetEncoders();
        m_robotContainer.m_robotDrive.zeroHeading();
        m_robotContainer.m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    public static double getVisionYawAngle() {
        return angle.getDouble(0);
    }

    public static boolean isValidAngle() {

        return validAngle.getBoolean(false);
    }
}
