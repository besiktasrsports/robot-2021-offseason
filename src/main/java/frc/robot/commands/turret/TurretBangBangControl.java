// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.TurretSubsystem;

public class TurretBangBangControl extends CommandBase {

  private final TurretSubsystem m_turret;
  private double error;
  private double yaw;
  private double goal;
  private int accuracy;
  private double x;
  /** Creates a new TurretBangBangControl. */
  public TurretBangBangControl(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.isValidAngle()){
    yaw = Robot.getVisionYawAngle();
    error = goal - yaw;

    if(error<0){
      m_turret.runTurret(0.3);
      if(error>-10){
        m_turret.runTurret(0.2);
      }
    }
    else if(error>0){
      m_turret.runTurret(-0.3);
      if(error<10){
        m_turret.runTurret(-0.2);
      }
    }
    else{
      m_turret.runTurret(0);
    }
    
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Robot.isValidAngle()==true && error>=-2 && error<=2){
      return true;
    }
    return false;
  }
}
