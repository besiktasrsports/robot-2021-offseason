package frc.sneakylib.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class WS2812LEDDriver extends SubsystemBase {

    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private static int i;
    private int j;
    

    public WS2812LEDDriver(int dataPort, int ledLength) {
        m_led = new AddressableLED(dataPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        i = 0;
        
        
        setBufferColor(0, 0, 0);
        m_led.start();
    }

    @Override
    public void periodic() {
        runDefault();
            
        
       
    }

    public static void setBufferColor(int r, int g, int b) {

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void turnOff() {
        setBufferColor(0, 0, 0);
        m_led.setData(m_ledBuffer);
    }

    public void runDefault(){
        if(Robot.ledCanStart == true){
         
        if(Robot.isValidAngle() == false){
            setBufferColor(255, 0, 0);
            m_led.setData(m_ledBuffer);
            j = 0;
        }
        else{
            
            if(Robot.m_robotContainer.m_turret.isFollowingTarget == false){
                if(j <=50){
                setBufferColor(0 ,i , i);
                m_led.setData(m_ledBuffer);
                if(i >= 204){
                    i = 0;
                    
                }
                i+=32;
                j++;
                
               
            }
            else{
                setBufferColor(0, 204, 204);
                
            }
           
           
           
            }
            else{
                j = 0;
                if(Robot.m_robotContainer.m_turret.isAtSetpoint == true){
                    setBufferColor(0, 255, 20);
                    m_led.setData(m_ledBuffer);
                }
                else{
                if(i <= 64){
                    setBufferColor(0, 204, 204);
                    m_led.setData(m_ledBuffer);
                }
                else{
                    setBufferColor(0, i, i);
                }
            }
              
            }
        }

    }
}

    }

