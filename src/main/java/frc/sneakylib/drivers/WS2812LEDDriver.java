package frc.sneakylib.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WS2812LEDDriver extends SubsystemBase {

    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private Timer m_timer;

    public WS2812LEDDriver(int dataPort, int ledLength) {
        m_led = new AddressableLED(dataPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_timer = new Timer();
        m_timer.start();
        setBufferColor(0, 0, 0);
        m_led.start();
    }

    @Override
    public void periodic() {
        runDefault();
    }

    public void setBufferColor(int r, int g, int b) {

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void turnOff() {
        setBufferColor(0, 0, 0);
        m_led.setData(m_ledBuffer);
    }

    public void runDefault() {

        for (int i = 0; i < m_ledBuffer.getLength(); i += 0) {

            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_led.setData(m_ledBuffer);
            final double startTime = Timer.getFPGATimestamp();
            if (Timer.getFPGATimestamp() - startTime >= 0.08) {
                i++;
            }
        }

        for (int i = 0; i < m_ledBuffer.getLength(); i += 0) {

            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
            final double startTime = Timer.getFPGATimestamp();
            if (Timer.getFPGATimestamp() - startTime >= 0.1) {
                i++;
            }
        }

        for (int i = m_ledBuffer.getLength() - 1; i >= 0; i -= 0) {

            m_ledBuffer.setRGB(i, 0, 255, 0);
            m_led.setData(m_ledBuffer);
            final double startTime = Timer.getFPGATimestamp();
            if (Timer.getFPGATimestamp() - startTime >= 0.08) {
                i--;
            }
        }

        for (int i = m_ledBuffer.getLength() - 1; i >= 0; i -= 0) {

            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
            final double startTime = Timer.getFPGATimestamp();
            if (Timer.getFPGATimestamp() - startTime >= 0.1) {
                i--;
            }
        }
    }
}
