import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.subsystems.TurretSubsystem;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class SubsystemTest {

    TurretSubsystem m_turretSubsystem;
    PWMSim m_testPWM;
    public static final double DELTA = 1e-2;

    @Before
    public void setup(){
    assert HAL.initialize(500, 0); // 500ms timeout for HAL to initialize.

    m_turretSubsystem = new TurretSubsystem();
    m_testPWM = new PWMSim(0);

    }

    @After
    public void shutdown() throws Exception{
        m_turretSubsystem.close();
    }

    @Test
    public void testTurret(){
        m_turretSubsystem.turretTestMethod(0.5);
        assertEquals(0.5, m_testPWM.getSpeed(), DELTA);
    }
}
