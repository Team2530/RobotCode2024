package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.brake();
        intake.setForwardLimitEnabled(true);
        starttime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!intake.getFrontLimitClosed()) {
            intake.setCustomPercent(0.4);
        }
        if (!intake.getReverseLimitClosed()) {
            shooter.setCustomPercent(-0.9);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeMode.STOPPED);
        shooter.setMode(ShooterMode.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return (!(intake.getFrontLimitClosed() ^ intake.getReverseLimitClosed())) || ((Timer.getFPGATimestamp() - starttime) > 0.5);
    }

}
