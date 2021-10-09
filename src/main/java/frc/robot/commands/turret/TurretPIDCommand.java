// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPIDCommand extends CommandBase {
  /** Creates a new TurretPIDCommand. */
  private TurretSubsystem m_turret;
  double error;
  double output;
  double outputSum;
  boolean shouldTurnRight = false;
  char lookingSide;
  public TurretPIDCommand(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    output = 0;
    m_turret.isFollowingTarget = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.isValidAngle()){
    error = Robot.getVisionYawAngle();
    output = error*TurretConstants.kP;
    shouldTurnRight = error>0 ? true : false;
    
    if(output > 6){
      output = 6;
    }
    else if (output < -6){
      output = -6;
    }
    if(!m_turret.turretHallEffect1.get() == true && shouldTurnRight == true){
      output = 0;
    }

    if(!m_turret.turretHallEffect2.get() == true && shouldTurnRight == false){
      output = 0;
    }

    if(error >= -2 && error <= 2){
      m_turret.isAtSetpoint = true;
      m_turret.isFollowingTarget = false; // might have to remove this
      output = 0;
      }
      outputSum += output;
      if(outputSum > 0){
        lookingSide = 'r';
      }
      else{
        lookingSide = 'l';
      }

    m_turret.setTurretVolts(output);
    
    }

    else{

      if(lookingSide == 'r'){
        m_turret.scanToLeft();
      }
      else if(lookingSide == 'l'){
        m_turret.scanToRight();
      }
      else{
        System.err.println("lookingSide is invalid! not scanning!");
        m_turret.setTurretVolts(0);
      }
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
