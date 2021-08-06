import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.TurretSubsystem;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class SubsystemTest {

    TurretSubsystem m_turretSubsystem;

    @Before
    public void setup(){
    assert HAL.initialize(500, 0);
    m_turretSubsystem = new TurretSubsystem();


    }

    @After
    public void shutdown() throws Exception{
        m_turretSubsystem.close();
    }

    @Test
    public void testTurret(){
        assertEquals("Hello, test!", m_turretSubsystem.turretTestMethod());
    }
}
