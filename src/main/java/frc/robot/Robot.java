// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sneakylib.auto.AdaptivePurePursuitController;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    NetworkTableInstance photoncam = NetworkTableInstance.create();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
    public static PhotonPipelineResult result;
    public static PhotonTrackedTarget target;


    PhotonCamera camera = new PhotonCamera("Lifecam");
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        autoChooser.setDefaultOption("Default Auto", 0);
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
        result = camera.getLatestResult();
        if(isValidAngle()){
        target = result.getBestTarget();
        }
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
        m_robotContainer.m_robotDrive.m_odometry
          .resetPosition(m_robotContainer.s_trajectory.testAuto[0].getInitialPose(), new Rotation2d(0));

          m_autonomousCommand = m_robotContainer.getAutonomousCommand();
          if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
          }
        /*
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        */
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
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.m_robotDrive.resetEncoders();
        m_robotContainer.m_robotDrive.zeroHeading();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //System.out.println(m_appc.update(m_robotContainer.s_trajectory.testAuto[0],m_robotContainer.m_robotDrive.getPose(),Math.toRadians(m_robotContainer.m_robotDrive.getHeading()) ,false)[0]);
        //System.out.println("Left Velocity : "+ m_robotContainer.m_robotDrive.getLeftWheelVelocity() + " Right Velocity : "+m_robotContainer.m_robotDrive.getRightWheelVelocity());
        //System.out.println("hello");
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    public static double getVisionYawAngle() {
        return target.getYaw();
    }

    public static boolean isValidAngle() {
        return result.hasTargets();
    }
}
