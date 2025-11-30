package frc.robot.commands;
//Import the controllers to be used (XBox or Flight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotConstants;

public class LEDCommand extends Command {
    //Call up the Subsystems to be used in this Command
    LEDSubsystem ledSubsystem;
    CoralIntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ExtendoSubsystem extendoSubsystem;
    //Name the contorllers to be used
    XboxController controller2;

    public LEDCommand(LEDSubsystem ledSubsystem, XboxController controller2, CoralIntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.ledSubsystem = ledSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.extendoSubsystem = extendoSubsystem;
        this.controller2 = controller2;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
    //This code was to change the color of LEDs connected through a REV Blinkin
    //This was based on a sensor being tripped instead of controller buttons or axes
    //Color value is defined through a reference guide provided by REV
        if (!intakeSubsystem.sensor.get() || intakeSubsystem.beambreak.isPressed()) {
            ledSubsystem.set(RobotConstants.LEDintakesensor);
        }
        else {
            ledSubsystem.set(RobotConstants.LEDdefault);
        }
    }
}