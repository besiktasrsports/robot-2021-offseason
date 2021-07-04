// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
 
  

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
    autoChooser.addOption("Path A Red", 1);
    autoChooser.addOption("Path A Blue", 2);
    autoChooser.addOption("Path A Blue and Red together ", 3);
    autoChooser.addOption("Path B Red", 4);
    autoChooser.addOption("Path B Blue", 5);
    autoChooser.addOption("Path B Blue and Red together", 6);
    autoChooser.addOption("BarrelRacingPath", 7);
    autoChooser.addOption("BouncePath", 8);
    
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());

    if(autoChooser.getSelected() == 1 ){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathARed[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 2){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathABlue[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 3){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathATogether[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 4){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathBRed[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 5){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathBBlue[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 6){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.PathBTogether[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 7){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.BarrelRacingPath[0].getInitialPose(), new Rotation2d(0));
    }
    if(autoChooser.getSelected() == 8){
      m_robotContainer.m_robotDrive.m_odometry.resetPosition(m_robotContainer.s_trajectory.BouncePath[0].getInitialPose(), new Rotation2d(0));
    }


    //m_robotContainer.m_robotDrive.m_odometry.resetPosition(new Pose2d(0,0, null), new Rotation2d(0));
    

    // schedule the autonomous command (example)
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
}
