package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class AutonIntakeOffCommand extends Command {
    CoralIntakeSubsystem intakeSubsystem;

    public AutonIntakeOffCommand(CoralIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOff();
    }
}
