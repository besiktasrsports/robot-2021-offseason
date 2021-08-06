// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Intaketest {


    IntakeSubsystem intake;
    DoubleSolenoidSim piston;

    @Before
    public void setup(){

        assert HAL.initialize(500, 0);
        intake = new IntakeSubsystem();
        piston = new DoubleSolenoidSim(IntakeConstants.kIntakeDoubleSolenoidPort1, IntakeConstants.kIntakeDoubleSolenoidPort2);
    }

    @After
    public void shutdown() throws Exception{
        intake.close();
    }

    @Test
    public void intakeup() {
        intake.intakeUp();
        assertEquals(DoubleSolenoid.Value.kReverse, piston.get());
    }

    @Test
    public void intakedown(){
        intake.intakeDown();
        assertEquals(DoubleSolenoid.Value.kForward, piston.get());
    }
}
