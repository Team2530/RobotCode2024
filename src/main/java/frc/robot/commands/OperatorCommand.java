package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OperatorCommand extends Command {
    private final XboxController xbox;

    private Set<Subsystem> requirements = new HashSet<>();

    public OperatorCommand(XboxController xbox) {
        this.xbox = xbox;
    }

    public double deadband(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
