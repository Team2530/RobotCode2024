package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeMode;

public class AlignNoteCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public AlignNoteCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.brake();
        intake.setForwardLimitEnabled(true);
    }

    @Override
    public void execute() {
        if (!intake.getFrontLimitClosed()) {
            intake.setCustomPercent(0.1);
        }
        if (!intake.getReverseLimitClosed()) {
            shooter.setCustomPercent(-0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeMode.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return !(intake.getFrontLimitClosed() ^ intake.getReverseLimitClosed());
    }

}
