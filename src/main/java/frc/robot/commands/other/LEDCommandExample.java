// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.other;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.sneakylib.drivers.WS2812LEDDriver;

public class LEDCommandExample extends CommandBase {
    /** Creates a new LEDCommandExample. */
    static WS2812LEDDriver m_ledDriver;

    public LEDCommandExample() {
        // Use addRequirements() here to declare subsystem dependencies.
        m_ledDriver = new WS2812LEDDriver(0, 40);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // bla bla run shooter here
        m_ledDriver.setColor("Red");
        m_ledDriver.setMode("blink");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
