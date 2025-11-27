package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotConstants;

public class LEDCommand extends Command {
    LEDSubsystem ledSubsystem;
    CoralIntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ExtendoSubsystem extendoSubsystem;
    XboxController controller2;

    public LEDCommand(LEDSubsystem ledSubsystem, XboxController controller2, CoralIntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.extendoSubsystem = extendoSubsystem;
        this.controller2 = controller2;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        if (!intakeSubsystem.sensor.get() || intakeSubsystem.beambreak.isPressed()) {
            ledSubsystem.set(RobotConstants.LEDintakesensor);
        }
        // else if (measurement != null && measurement.distance_mm < 30 && extendoSubsystem.getEncoderMeasurement() < 0.16 && extendoSubsystem.getEncoderMeasurement() > 0.14){
        //     ledSubsystem.set(RobotConstants.LEDintakeReady);
        // }
        else {
            ledSubsystem.set(RobotConstants.LEDdefault);
        }
    }
}