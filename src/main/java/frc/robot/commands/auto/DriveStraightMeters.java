// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveStraightMeters extends CommandBase {
  /** Creates a new DriveStraightMeters. */
  private double m_x_meters;
  private double m_goalAngle;
  private DriveSubsytem m_drive;
  private double poseXError;
  private double m_kP;
  private double lastPoseXError;
  private double angularError;
  private double lastAngularError;
  private double leftPower;
  private double rightPower;
  private DifferentialDriveOdometry m_odometry;
  public DriveStraightMeters(DriveSubsytem drive, double x_meters, double goalAngle, double kP) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x_meters = x_meters;
    m_goalAngle = goalAngle;
    m_kP = kP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drive.resetEncoders();
    m_drive.zeroHeading();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_drive.getHeading()));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double posePower;
    double turnPower;

    m_odometry.update(
      Rotation2d.fromDegrees(m_drive.getHeading()), m_drive.getLeftEncoderDistance(), m_drive.getRightEncoderDistance());

      poseXError = m_x_meters - m_odometry.getPoseMeters().getX();
      double current_heading = m_drive.getHeading();
      if(current_heading > 180)
      {
        current_heading = current_heading - 360;
      }
      angularError = m_goalAngle - current_heading;

      posePower = 0;
      turnPower = 0;
      posePower = (m_kP*poseXError + (Constants.DriveConstants.kStraightDriveD*(poseXError-lastPoseXError)));
      turnPower = (Constants.DriveConstants.kStraightDriveTurnP*angularError) + (Constants.DriveConstants.kStraightDriveTurnD*(angularError-lastAngularError));
      leftPower = 0;
      rightPower = 0;
      /*
      if (turnPower > posePower)
      {
        turnPower = posePower;
      }
      */
      leftPower = posePower - turnPower; 
      rightPower = posePower;
      if(leftPower >=  0){
        leftPower += Constants.DriveConstants.kStraightDriveMinVolts;
      }
      else{
        leftPower -= Constants.DriveConstants.kStraightDriveMinVolts;
      }

      if(rightPower >=  0){
        rightPower += Constants.DriveConstants.kStraightDriveMinVolts;
      }
      else{
        rightPower -= Constants.DriveConstants.kStraightDriveMinVolts;
      }

      if (leftPower > Constants.DriveConstants.kStraightDriveMaxVolts)
      {
        leftPower = Constants.DriveConstants.kStraightDriveMaxVolts;
      }
      else if (leftPower < -Constants.DriveConstants.kStraightDriveMaxVolts)
      {
        leftPower = -Constants.DriveConstants.kStraightDriveMaxVolts;
      }
      if (rightPower > Constants.DriveConstants.kStraightDriveMaxVolts)
      {
        rightPower = Constants.DriveConstants.kStraightDriveMaxVolts;
      }
      else if (rightPower < -Constants.DriveConstants.kStraightDriveMaxVolts)
      {
        rightPower = -Constants.DriveConstants.kStraightDriveMaxVolts;
      }
      m_drive.tankDriveVolts(leftPower, rightPower);
      
      System.out.println(m_odometry.getPoseMeters());
      System.out.print("PXE : " + poseXError);
      System.out.print(" PYE : " + angularError);
      System.out.print(" PoseP : " + posePower);
      System.out.print(" TurnP : " + turnPower);
      System.out.print(" RightP : " + rightPower);
      System.out.println(" LeftP : " + leftPower);
      lastPoseXError = poseXError;
      lastAngularError = angularError;
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
    angularError >= -Constants.DriveConstants.kStraightDriveTurnAccuracy && angularError <= Constants.DriveConstants.kStraightDriveTurnAccuracy){
      return true;
    }
    return false;
  }
}
