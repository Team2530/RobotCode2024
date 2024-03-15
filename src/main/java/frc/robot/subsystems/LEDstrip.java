package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommand;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Presets;

public class LEDstrip extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    int rainbow = 0;

    final int LEDLength = 10;
    int[] RGBWData = new int[200];

    public Shooter shooter;
    public Intake intake;
    public Arm arm;
    public SwerveSubsystem swerve;
    public DriveCommand drive;

    private XboxController DEBUG_XBOX = new XboxController(0);

    // Intake intake
    public LEDstrip(
            int portPWM, Intake intake, Shooter shooter, Arm arm, SwerveSubsystem swerve, DriveCommand drive) {
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.swerve = swerve;
        this.drive = drive;
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(8);

        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(LEDLength);

        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);

        m_led.start();

    }
    @Override
    public void periodic() {
        //for (int i=0;i<200;i++) {setDataRGBW(i,255,0,0,0);}
        m_ledBuffer.setRGB(0,0,255,0);

        push();

    }
    public void rainbow(double difference) {
        rainbow += 0.05;
        for (int i=0;i<LEDLength-1;i++) {
            double j = i*(Math.PI-difference);
            setDataRGBW(i,
            (int) (Math.sin(rainbow+j)*255),
            (int) (Math.sin(rainbow+2+j)*255),
            (int) (Math.sin(rainbow+4+j)*255),0);}
    }
    public void setRGBW(AddressableLEDBuffer ledBuffer, int index, int R, int G, int B, int W) {
        int offsetindex = index + (int) Math.floor(index/3); // Index offset ONLY FOR ENCODING INDO GRBGRBGRBGRB
        AddressableLEDBuffer m_ledBuffer = ledBuffer;
        int[] prevRGBW = getDataRGBW(index);
        if (index>0) {prevRGBW = getDataRGBW(index-1);}

        int[] nextRGBW = getDataRGBW(index+1);

        //print(prevRGB.toString());
        /** HOW TO READ R G B comments
         * I will use C and V as example
         * C=C: Use C to output as C. basically just normal setRGB
         * C=V: Use V to output as C. for example G=W (W is the input value from us)
         * C=+V: Use the next index's V to output
         * C=-V: Use the previous index's V to output.
         * 
         * REMEMBER: you use the INDEX, not the OFFSETINDEX
         */

        switch (index % 3) {
            case 0:
                m_ledBuffer.setRGB(offsetindex, R, G, B); // R=R G=G B=B
                m_ledBuffer.setRGB(offsetindex+1, nextRGBW[1], W, nextRGBW[0]); // R=+G G=W B=+R
                break;
            case 1:
                m_ledBuffer.setRGB(offsetindex, G, prevRGBW[3], R); // R=G G>-W B=R
                m_ledBuffer.setRGB(offsetindex+1, W, B, nextRGBW[1]); // R=W G=B B=+G FOR B, REMEMBER THAT THE INDEX is 1
                break;
            case 2:
                m_ledBuffer.setRGB(offsetindex, prevRGBW[3], prevRGBW[2], G); // R=-W G=-B B=G
                m_ledBuffer.setRGB(offsetindex+1, B, R, W); // R=B G=R B=W
                break;
            default:break;
        }
    }

    @Override
    public void simulationPeriodic() {
        // This built-in method will be called once per scheduler run during simulation
    }

    public AddressableLEDBuffer getBuffer() {
        return m_ledBuffer;
    }

    public void push() {
        for (int i=0; i<LEDLength-1;i++) {
            setRGBW(m_ledBuffer, i,
            RGBWData[i * 4],
            RGBWData[i * 4 + 1],
            RGBWData[i * 4 + 2],
            RGBWData[i * 4 + 3]);
        }
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }
    public int[] getDataRGBW(int index) {
        return new int[] {
            RGBWData[index * 4],
            RGBWData[index * 4 + 1],
            RGBWData[index * 4 + 2],
            RGBWData[index * 4 + 3]};
    }
    public void setDataRGBW(int index, int R, int G, int B, int W) {
        RGBWData[index * 4] = R;
        RGBWData[index * 4 + 1] = G;
        RGBWData[index * 4 + 2] = B;
        RGBWData[index * 4 + 3] = W;
    }

    public void turnOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            setDataRGBW(i, 0, 0, 0, 0);
        }
        push();
    }
}