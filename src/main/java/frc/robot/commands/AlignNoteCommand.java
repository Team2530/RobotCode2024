package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

public class AlignNoteCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;
    double starttime;

    public AlignNoteCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.brake();
        intake.setShooterLimitEnabled(true);
        SmartDashboard.putBoolean("Aligning", true);

        starttime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!intake.getShooterSideLimitClosed()) {
            intake.setMode(IntakeMode.INTAKING);
        }
        if (!intake.getIntakeSideLimitClosed()) {
            shooter.setMode(ShooterMode.REVERSE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeMode.STOPPED);
        shooter.setMode(ShooterMode.STOPPED);
        SmartDashboard.putBoolean("Aligning", false);
    }

    @Override
    public boolean isFinished() {
        if (intake.getIntakeSideLimitClosed() && intake.getShooterSideLimitClosed()) {
            return true;
        } else if ((!intake.getIntakeSideLimitClosed()) && (!intake.getShooterSideLimitClosed())) {
            return true; // FAIL!!
        } 
        return (Timer.getFPGATimestamp() - starttime) > 1.0;
    }

}
