package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;

public class PrepNoteCommand extends Command {
        private final Intake intake;
    private double startpos;

    public PrepNoteCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        startpos = intake.getIntakePosition();
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
        return (intake.getIntakePosition() <= (startpos - 0.2)); //!intake.getReverseLimitClosed();
    }

}
