package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;

public class PrepNoteCommand extends Command {
    
    private final Shooter shooter;
    private final Intake intake;
    private double startpos;

    public PrepNoteCommand(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        startpos = intake.getIntakePosition();
        shooter.coast();
        intake.brake();
        intake.setMode(IntakeMode.REVERSE);
        SmartDashboard.putString("Shootake", "Moving note back " + intake.getOutputPercent());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(IntakeMode.STOPPED);
    }

    @Override
    public boolean isFinished() {
        // note is clear of shooter and it may start to be spooled
        return (intake.getIntakePosition() <= (startpos - 0.5)); //!intake.getReverseLimitClosed();
    }

}
