// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    /** Creates a new FeederSubsystem. */
    private final WPI_VictorSPX FeederMotor = new WPI_VictorSPX(FeederConstants.kFeederMotorPort);

    private final DigitalInput feederSensor = new DigitalInput(4);

    public FeederSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runFeeder(double speed) {
        FeederMotor.set(speed);
    }

    public void stopFeeder() {
        FeederMotor.set(0);
    }

    public boolean getSensorStatus() {

        return !feederSensor.get();
    }
}
