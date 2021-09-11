package frc.sneakylib.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WS2812LEDDriver extends SubsystemBase {

    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private static int m_rainbowFirstPixelHue;
    private static String mode;

    public WS2812LEDDriver(int dataPort, int ledLength) {
        m_led = new AddressableLED(dataPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
    }

    @Override
    public void periodic() {
        run();
    }

    public void setColor(String color) {

        switch (color) {
            case "Red":
                setBufferColor(255, 0, 0);
                break;
            case "Green":
                setBufferColor(0, 255, 0);
                break;
            case "Blue":
                setBufferColor(0, 0, 255);
                break;
            case "Purple":
                setBufferColor(255, 0, 255);
                break;
            case "Aqua":
                setBufferColor(0, 153, 153);
                break;
            case "Lime":
                setBufferColor(102, 255, 178);
                break;
            case "Yellow":
                setBufferColor(255, 255, 0);
                break;
            case "Orange":
                setBufferColor(253, 153, 51);
                break;
            case "Off":
                setBufferColor(0, 0, 0);
            default:
                setBufferColor(255, 255, 255);
        }
    }

    public void run() {

        if (mode.toLowerCase() == "blink") {
            int blinkFreq = 250; // miliseconds
            double startTime = Timer.getFPGATimestamp();
            m_led.setData(m_ledBuffer);
            m_led.start();
            if (Timer.getFPGATimestamp() - blinkFreq / 1000 > startTime) {
                setColor("Off");
                m_led.setData(m_ledBuffer);
            }
        } else if (mode.toLowerCase() == "fade") {

        } else if (mode.toLowerCase() == "snake") {

        } else if (mode.toLowerCase() == "rainbow") {
            // For every pixel

            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess

                final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
                // Set the value
                m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
            m_led.setData(m_ledBuffer);
        }
    }

    private static void setBufferColor(int r, int g, int b) {

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setMode(String modeToSet) {

        mode = modeToSet;
    }
}
