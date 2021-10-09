// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.turret.TurretBangBangControl;

public class TurretSubsystem extends SubsystemBase {

    private final VictorSP turretMotor = new VictorSP(TurretConstants.kTurretMotorPort);
    public final DigitalInput turretHallEffect1 = new DigitalInput(TurretConstants.kTurretHallEffect1Port); // right 90
    public final DigitalInput turretHallEffect2 = new DigitalInput(TurretConstants.kTurretHallEffect2Port); // left 90
    public boolean isFollowingTarget = false;
    public boolean isAtSetpoint = false;
    
    public TurretSubsystem() {

        turretMotor.setInverted(TurretConstants.kIsMotorReversed);

    }
  

    @Override
    public void periodic() {
        //System.out.println("Hall 1 : " + !turretHallEffect1.get() + "Hall 2 :" + !turretHallEffect2.get());
    }

    public void runTurret(double speed) {

        turretMotor.set(speed);
    }

    public void setTurretVolts(double volts){
        turretMotor.setVoltage(volts);
    }

    public void scanToLeft(){
        if(!turretHallEffect2.get() != true){
        setTurretVolts(-5);
        }
        else{
            setTurretVolts(0);
        }
    }

    public void scanToRight(){
        if(!turretHallEffect1.get() != true){
            setTurretVolts(5);
            }
            else{
                setTurretVolts(0);
            }

    }
}
