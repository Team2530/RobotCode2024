package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;

public class LEDstripOne extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;
    public Intake intake;
//Intake intake
    public LEDstripOne(int portPWM) {
        //this.intake = intake;
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(portPWM);


        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(210);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }


 


      //R: 57, G: 255, B: 20

    @Override
    public void periodic() {    

        /* 
        if(intake.getFrontLimitClosed() == false){
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 255, 0, 0);
                System.out.println(i);
            }
        }
        else{

            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 255, 0);
                System.out.println(i);
            }
        }
         */
         m_led.setData(m_ledBuffer);

      // This built-in method will be called once per scheduler run
    }
  

    @Override
    public void simulationPeriodic() {
      // This built-in method will be called once per scheduler run during simulation
    }

    public AddressableLEDBuffer getBuffer() {
        return m_ledBuffer;
    }

    public void push() {
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    public void turnOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        push();
    }
}