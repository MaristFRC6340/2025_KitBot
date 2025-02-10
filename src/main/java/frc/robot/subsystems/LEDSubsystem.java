package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

    

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    public LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,Color.kBlue, Color.kAliceBlue, Color.kBlueViolet, Color.kCornflowerBlue).breathe(Seconds.of(2));


    public LEDSubsystem(){
        m_led = new AddressableLED(4);

            // Reuse buffer
            // Default to a length of 60, start empty output
            // Length is expensive to set, so only set it once, then just update data
            m_ledBuffer = new AddressableLEDBuffer(50);
            m_led.setLength(m_ledBuffer.getLength());

            // Set the data
            m_led.setData(m_ledBuffer);
            m_led.start();

    }
    @Override
    public void periodic(){
        m_led.setData(m_ledBuffer);

    }


    public Command runPattern(LEDPattern ledPattern){
        return run(()->ledPattern.applyTo(m_ledBuffer));
    }
}