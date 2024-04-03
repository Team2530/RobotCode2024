package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Intake intake;

    private double dTime = 0.0;

    public ShootCommand(Shooter shooter, Intake intake) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        intake.coast();
        shooter.coast();
        intake.setShooterLimitEnabled(false);
        intake.setMode(IntakeMode.SHOOT);
        SmartDashboard.putString("Shootake", "Shooting note");

        dTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {

        if (!interrupted) {
            shooter.setMode(ShooterMode.STOPPED);
            intake.setMode(IntakeMode.STOPPED);
        }
        intake.setShooterLimitEnabled(true);

        SmartDashboard.putString("Shootake", "Ending Shooting Sequence");
    }

    @Override
    public boolean isFinished() {
        // terminate after 1.5 seconds has passed and front limit doesn't see the note
        // TODO Determine if 1.5 seconds is too long or not
       return (!intake.getShooterSideLimitClosed()) && (!intake.getIntakeSideLimitClosed());
    }
}
