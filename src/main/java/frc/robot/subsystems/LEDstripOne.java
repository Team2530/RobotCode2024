package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Intake;

public class LEDstripOne extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;
    public Intake intake;
//Intake intake
    public LEDstripOne(int portPWM, Intake intake) {
        this.intake = intake;
        // Must be a PWM header, not MXP or DIO
       m_led = new AddressableLED(8);
        


        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(40);
        
        
        
        m_led.setLength(m_ledBuffer.getLength());
        

        // Set the data
        m_led.setData(m_ledBuffer);
        

        m_led.start();
        
    }

        void setLED(int n, int r, int g, int b){
                m_ledBuffer.setRGB(n, r, g, b);
                m_ledBuffer.setRGB(39-n, r, g, b);
        }

        private void rainbow() {
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
          }

    boolean flash(double hz) {
        double prd = 1.0/hz;
        double v = (
            (((double)RobotController.getFPGATime())/1000000.0)%hz
        )/prd;
        System.out.println(v);
        return v < 0.5;
    }

    double remap(double a,double b,double c,double d,double v) {
        return MathUtil.clamp(c + (d-c)*((v-a)/(b-a)), c, d);
    }

    @Override
    public void periodic() {   
        final double ctime = 15;
        final double cscale = (255.0/ctime);
        double ltmult = ctime/20.0;

        double mtime = DriverStation.getMatchTime();
        if(DriverStation.isTeleopEnabled() && mtime <= ctime){
            for(int i=0; i < 20;i++){
                // Countdown
                if ((double)i*ltmult > mtime) {
                    // No LED
                    setLED(i, 0, 0, 0);
                } else {
                    int g = (int)remap(ctime, ctime*0.666666666666666, 255,0, mtime);
                    int r = (int)remap(ctime, ctime*0.666666666666666,0,255, mtime);
                    // if (flash(2) || (mtime > 10)) {
                        setLED(i, r, g, 0);
                    // } else {
                        // setLED(i, 0, 0, 0);
                    // }
                }
            }
        }
        else if(intake.getFrontLimitClosed()){
            rainbow();
        }
        else{
            for (var i = 0; i < 20; i++) {
                setLED(i, 0, 255, 0);
            }
        }
        
        

            
         m_led.setData(m_ledBuffer);

         // Timer.getMatchTime();
         

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