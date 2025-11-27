package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class AutonTimedIntakeCommandReverse extends Command {
    CoralIntakeSubsystem intakeSubsystem;
    Timer timer;

    public AutonTimedIntakeCommandReverse(CoralIntakeSubsystem intakeSubsystem) {
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
        intakeSubsystem.intakeOn(false);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 0.3) {
            intakeSubsystem.intakeOff();
            return true;
        }
        return false;
    }
}
