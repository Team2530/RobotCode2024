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

public class LEDstripOne extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    int m_rainbowFirstPixelHue = 0;

    public Shooter shooter;
    public Intake intake;
    public Arm arm;
    public SwerveSubsystem swerve;
    public DriveCommand drive;

    private XboxController DEBUG_XBOX = new XboxController(0);

    // Intake intake
    public LEDstripOne(
            int portPWM, Intake intake, Shooter shooter, Arm arm, SwerveSubsystem swerve, DriveCommand drive) {
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.swerve = swerve;
        this.drive = drive;
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(8);

        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(40);

        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);

        m_led.start();

    }

    void setLED(int n, int r, int g, int b) {
        m_ledBuffer.setRGB(n, r, g, b);
        m_ledBuffer.setRGB(39 - n, r, g, b);
    }

    void setSolidColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); ++i)
            m_ledBuffer.setRGB(i, r, g, b);
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

    private void sinColor(int r, int g, int b, double waves, double center, double amp, double tscroll) {
        double lambda = m_ledBuffer.getLength() / (2 * waves);
        for (int i = 0; i < 20; ++i) {
            double fx = Math.cos((i / lambda) * Math.PI + Timer.getFPGATimestamp() * tscroll * Math.PI) * amp + center;
            setLED(i,
                    MathUtil.clamp((int) (r * fx), 0, 255),
                    MathUtil.clamp((int) (g * fx), 0, 255),
                    MathUtil.clamp((int) (b * fx), 0, 255));
        }
    }

    boolean flash(double hz) {
        double period = 1.0 / hz;
        double v = ((Timer.getFPGATimestamp()) % period) / period;
        return v < 0.5;
    }

    double remap(double a, double b, double c, double d, double v) {
        return MathUtil.clamp(c + (d - c) * ((v - a) / (b - a)), c, d);
    }

    public double antiAliasFac(int led, double value, double fac) {
        return MathUtil.clamp(((double) led - value) / fac + 0.5, 0, 1);
    }

    @Override
    public void periodic() {
        final double ctime = 20;
        double ltmult = ctime / 20.0;

        double mtime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomousEnabled()) {
            rainbow();
        } else if (DriverStation.isTeleopEnabled() && mtime <= ctime && (mtime >= 0.005)) {
            for (int i = 0; i < 20; i++) {
                // Countdown
                int r = MathUtil.clamp((int) (255 * ((1.5 * ctime - 2 * mtime) / ctime)), 0, 255);
                int g = MathUtil.clamp((int) (255 * ((2 * mtime - 0.75 * ctime) / ctime)), 0, 255);
                double amplitude = 0.0;
                if (mtime < 10) {
                    amplitude = flash(2) ? 1.0 : 0.0;
                } else {
                    amplitude = (0.8 + 0.2 * Math.sin(Timer.getFPGATimestamp() * Math.PI * 3));
                }
                amplitude *= 1.0 - antiAliasFac(i + 1, mtime, 2);

                r *= amplitude;
                g *= amplitude;

                setLED(i, r, g, 0);
                // }
            }

            // Ready to shoot
        } else if ((arm.getCurrentPreset() == Presets.SHOOT_HIGH || arm.getCurrentPreset() == Presets.SHOOT_LOW)
                && shooter.isReadySpooled() && drive.isSpeakerAligned()) {
            setSolidColor(0, 255, 0);
        } else if ((arm.getCurrentPreset() == Presets.TRAP || arm.getCurrentPreset() == Presets.AMP)
                && shooter.isReadySpooled()) {
            setSolidColor(0, 255, 0);
        } else if (arm.getCurrentPreset() == Presets.SOURCE) {
            // Intaking!!
            sinColor(255, 30, 0, 2, 0.5, 0.5, 5);
        } else {
            // Set idle lights based on alliance color
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                setSolidColor(100, 0, 0);
            } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                setSolidColor(0, 0, 100);
            } else {
                setSolidColor(64, 64, 64);
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