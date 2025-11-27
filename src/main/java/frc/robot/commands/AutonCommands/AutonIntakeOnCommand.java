package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeOnCommand extends Command {
    CoralIntakeSubsystem intakeSubsystem;
    Timer timer;

    public AutonIntakeOnCommand(CoralIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOn(true);
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.isBeamBroken() || timer.get() >= 2.0) {
            intakeSubsystem.intakeOff();
            return true;
        }
        return false;
    }
}