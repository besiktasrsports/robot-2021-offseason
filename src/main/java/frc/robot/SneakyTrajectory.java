/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autonomous.GalacticSearch.*;
import frc.robot.subsystems.DriveSubsytem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
  public Trajectory[] PathARed = new Trajectory[2];
  public Trajectory[] PathABlue = new Trajectory[2];
  public Trajectory[] PathATogether = new Trajectory[3];
  public Trajectory[] PathBRed = new Trajectory[2];
  public Trajectory[] PathBBlue = new Trajectory[1];
  public Trajectory[] PathBTogether = new Trajectory[3];
  public Trajectory[] ZeroTrajectory = new Trajectory[1];
  public Trajectory[] BarrelRacingPath = new Trajectory[6];
  public Trajectory[] BouncePath = new Trajectory[4];
    
    private DriveSubsytem m_drive;

 
    public SneakyTrajectory(DriveSubsytem drive){
                
        m_drive = drive;
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            DriveConstants.kMaxAutoVoltage);
            TrajectoryConfig configForward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);

     TrajectoryConfig configBackward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);
     configBackward.setReversed(true);

     
     

      //#region ZeroTrajectory
      ZeroTrajectory[0] = TrajectoryGenerator.generateTrajectory( 
      List.of(
          new Pose2d(0, 0, new Rotation2d(0))),
          configBackward);
      //#endregion    

      //#region PathARed
      PathARed[0] = TrajectoryGenerator.generateTrajectory( 
     List.of(
         new Pose2d(0.74, 4.775, new Rotation2d(-2.9671)), 
         new Pose2d(4, 4, new Rotation2d(2.4435)),
         new Pose2d(6.57, 2.58, new Rotation2d(-2.1293))),
         configBackward);
      
         PathARed[1] = TrajectoryGenerator.generateTrajectory( 
         List.of(
             new Pose2d(6.57, 2.58, new Rotation2d(-2.1293)), 
             new Pose2d(15.3, 5.7, new Rotation2d(0))),
             configForward);


      //#endregion
      
      //#region PathABlue
      PathABlue[0] = TrajectoryGenerator.generateTrajectory( 
        List.of(
            new Pose2d(0.75, 1.38, new Rotation2d(-3.1416)), 
            new Pose2d(8, 1.33, new Rotation2d(2.618)),
            new Pose2d(9.34, 5.43, new Rotation2d(-1.9199))),
            configBackward);
                  
            PathABlue[1] = TrajectoryGenerator.generateTrajectory( 
            List.of(
                new Pose2d(9.34, 5.43, new Rotation2d(2.7925)), 
                new Pose2d(12, 4, new Rotation2d(2.618)),
                new Pose2d(15, 3, new Rotation2d(2.9671))),
                configBackward);

      //#endregion

      //#region PathATogether
      
      PathATogether[0] = TrajectoryGenerator.generateTrajectory( 
      List.of(
         new Pose2d(0.7, 5, new Rotation2d(-3.1416)), 
         new Pose2d(4, 4, new Rotation2d(2.618)),
         new Pose2d(6.7, 2.7, new Rotation2d(2.4435)),
         new Pose2d(8, 1.4, new Rotation2d(2.2689))),
         configBackward);
                        
         PathATogether[1] = TrajectoryGenerator.generateTrajectory( 
         List.of(
             new Pose2d(8, 1.4, new Rotation2d(2.2689)),
             new Pose2d(7, 6.8, new Rotation2d(-3.14169))),
             configForward);
             
             PathATogether[2] = TrajectoryGenerator.generateTrajectory( 
             List.of(
               new Pose2d(7, 6.8, new Rotation2d(-3.14169)),
               new Pose2d(8, 6.8, new Rotation2d(2.618)), 
               new Pose2d(9.3, 5.5, new Rotation2d(2.3213)),
               new Pose2d(12, 4, new Rotation2d(2.9671)),
               new Pose2d(15.2, 3.4, new Rotation2d(2.7925))),
               configBackward);
      //#endregion

      //#region PathBRed
      
      PathBRed[0] = TrajectoryGenerator.generateTrajectory( 
      List.of(
               new Pose2d(0.7, 1.9, new Rotation2d(-2.8798)), 
               new Pose2d(4, 5.5, new Rotation2d(2.5307)),
               new Pose2d(6.7, 2.8, new Rotation2d(-2.3038)),
               new Pose2d(9.3, 5.5, new Rotation2d(-2.2166))),
               configBackward);
                          
             PathBRed[1] = TrajectoryGenerator.generateTrajectory( 
             List.of( 
               new Pose2d(9.3, 5.5, new Rotation2d(0)),
               new Pose2d(15.2, 5.5, new Rotation2d(0))),
               configForward);
      //#endregion

      //#region PathBBlue
      
      PathBBlue[0] = TrajectoryGenerator.generateTrajectory( 
      List.of(
               new Pose2d(0.7, 1.9, new Rotation2d(3.0543)), 
               new Pose2d(8, 2.8, new Rotation2d(-2.618)),
               new Pose2d(10.7, 5.5, new Rotation2d(2.9671)),
               new Pose2d(13.3, 2.8, new Rotation2d(2.0944)),
               new Pose2d(15.3, 2.3, new Rotation2d(-2.9671))),
               configBackward);
      //#endregion

      //#region PathBTogether

      PathBTogether[0] = TrajectoryGenerator.generateTrajectory( 
      List.of(
               new Pose2d(0.7, 6, new Rotation2d(-3.1416)), 
               new Pose2d(4, 5.5, new Rotation2d(2.618)),
               new Pose2d(6.7, 2.7, new Rotation2d(2.3213)),
               new Pose2d(8, 2.7, new Rotation2d(-3.1416)),
               new Pose2d(13.3, 2.7, new Rotation2d(3.1416))),
               configBackward);
                                   
               PathBTogether[1] = TrajectoryGenerator.generateTrajectory( 
               List.of(
               new Pose2d(13.3, 2.7, new Rotation2d(-1.0647)),
               new Pose2d(10.7, 5.5, new Rotation2d(0.017453)),
               new Pose2d(9.2, 5.5, new Rotation2d(0))),
               configBackward);
                        
               PathBTogether[2] = TrajectoryGenerator.generateTrajectory( 
               List.of(
               new Pose2d(9.2, 5.5, new Rotation2d(0)),
               new Pose2d(15.2, 5.5, new Rotation2d(0))),
               configForward);

      //#endregion

      //#region BarrelRacingPath

      BarrelRacingPath[0] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(2.0, 3.7, new Rotation2d(Math.toRadians(21))), 
            new Pose2d(7.0, 4.0, new Rotation2d(Math.toRadians(-31)))),
            configBackward);
            
      BarrelRacingPath[1] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(7.0, 4.0, new Rotation2d(Math.toRadians(-31))), 
            new Pose2d(6.8, 1.7, new Rotation2d(Math.toRadians(160))), 
            new Pose2d(6.5, 3.5, new Rotation2d(Math.toRadians(26)))),
            configBackward);   

      BarrelRacingPath[2] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(6.5, 3.5, new Rotation2d(Math.toRadians(26))), 
            new Pose2d(11.7, 5.3, new Rotation2d(Math.toRadians(62.16)))),
            configBackward);
                  
      BarrelRacingPath[3] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(11.7, 5.3, new Rotation2d(Math.toRadians(62.16))), 
            new Pose2d(10.3, 6.3, new Rotation2d(Math.toRadians(-150))), 
            new Pose2d(10.3, 3.7, new Rotation2d(Math.toRadians(-50)))),
            configBackward);   
      
      BarrelRacingPath[4] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(10.3, 3.7, new Rotation2d(Math.toRadians(-50))), 
            new Pose2d(14.0, 2.4, new Rotation2d(Math.toRadians(45))), 
            new Pose2d(13.5, 4.0, new Rotation2d(Math.toRadians(170)))),
            configBackward);
                      
      BarrelRacingPath[5] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(13.5, 4.0, new Rotation2d(Math.toRadians(-175))), 
            new Pose2d(7.0, 4.8, new Rotation2d(Math.toRadians(175))), 
            new Pose2d(2.0, 5.0, new Rotation2d(Math.toRadians(-175)))),
            configBackward);

      //#endregion

      //#region BouncePath
      
      BouncePath[0] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(2.0, 4.0, new Rotation2d(Math.toRadians(0))), 
            new Pose2d(4.0, 7.0, new Rotation2d(Math.toRadians(90)))),
            configForward);
            
      BouncePath[1] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(4.0, 7.0, new Rotation2d(Math.toRadians(90))), 
            new Pose2d(6.8, 1.5, new Rotation2d(Math.toRadians(175))), 
            new Pose2d(8.0, 7.0, new Rotation2d(Math.toRadians(-100)))),
            configBackward);   

      BouncePath[2] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(8.0, 7.0, new Rotation2d(Math.toRadians(-100))), 
            new Pose2d(10.0, 1.7, new Rotation2d(Math.toRadians(10))), 
            new Pose2d(12.0, 7.0, new Rotation2d(Math.toRadians(90)))),
            configForward);
                  
      BouncePath[3] = TrajectoryGenerator.generateTrajectory( 
          List.of(
            new Pose2d(12.0, 7.0, new Rotation2d(Math.toRadians(90))), 
            new Pose2d(15.0, 4.0, new Rotation2d(Math.toRadians(180)))),
            configBackward);   

      //#endregion



    }
    public RamseteCommand getRamsete(Trajectory trajectory ){

        return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
    );
    }


}
