// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsytem;

public class DriveStraightMeters extends CommandBase {
  /** Creates a new DriveStraightMeters. */
  private double m_meters;
  private DriveSubsytem m_drive;
  private double poseXError;
  private double lastPoseXError;
  private double poseYError;
  private double lastPoseYError;
  private DifferentialDriveOdometry m_odometry;
  public DriveStraightMeters(DriveSubsytem drive, double meters) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_meters = meters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_drive.getHeading()));
    m_drive.resetEncoders();
    m_drive.zeroHeading();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double posePower;
    double turnPower;

    m_odometry.update(
      Rotation2d.fromDegrees(m_drive.getHeading()), m_drive.getLeftEncoderDistance(), m_drive.getRightEncoderDistance());

      poseXError = m_meters - m_odometry.getPoseMeters().getX();
      poseYError = -m_odometry.getPoseMeters().getY();

      if(poseXError >= 0){
         posePower = (Constants.DriveConstants.kStraightDriveP*poseXError + (Constants.DriveConstants.kStraightDriveD*(lastPoseXError-poseXError)));

      }
      else{
        posePower = -((Constants.DriveConstants.kStraightDriveP*poseXError + (Constants.DriveConstants.kStraightDriveD*(lastPoseXError-poseXError))));

      }

      if(poseYError >= 0){
        turnPower = (Constants.DriveConstants.kStraightDriveTurnP*poseYError) + (Constants.DriveConstants.kStraightDriveTurnD*(lastPoseYError-poseYError));
        m_drive.tankDriveVolts(posePower-turnPower, -posePower);
      }
      else{
        turnPower = -((Constants.DriveConstants.kStraightDriveTurnP*poseYError) + (Constants.DriveConstants.kStraightDriveTurnD*(lastPoseYError-poseYError)));
        m_drive.tankDriveVolts(posePower, -posePower-turnPower);
      }

      
      lastPoseXError = poseXError;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command s  hould end.
  @Override
  public boolean isFinished() {
    if(poseXError >= -Constants.DriveConstants.kStraightDriveAccuracy && poseXError <= Constants.DriveConstants.kStraightDriveAccuracy &&
    poseYError >= -Constants.DriveConstants.kStraightDriveTurnAccuracy && poseYError <= Constants.DriveConstants.kStraightDriveTurnAccuracy){
      return true;
    }
    return false;
  }
}
