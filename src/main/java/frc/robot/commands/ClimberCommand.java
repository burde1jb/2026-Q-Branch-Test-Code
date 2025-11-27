package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class ClimberCommand extends Command {
    ClimberSubsystem climberSubsystem;
    CommandJoystick controller1;
    XboxController controller2;

    public ClimberCommand(
            ClimberSubsystem climberSubsystem,
            CommandJoystick controller1,
            XboxController controller2) {
        this.climberSubsystem = climberSubsystem;
        this.controller1 = controller1;
        this.controller2 = controller2;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (controller2.getPOV() == 0){
            climberSubsystem.extend();
        }
        else if (controller2.getPOV() == 180){
            climberSubsystem.retract();
        }
        else{
           climberSubsystem.stop(); 
        }

        //Written withe right stick
        // if (controller2.getRightStickButton() && controller2.getRightX() > 0.2){
        //     climberSubsystem.goTo(RobotConstants.ClimberClimbgoal);
        // }
        // else if (controller2.getRightStickButton() && controller2.getRightX() < 0.2){
        //     climberSubsystem.goTo(RobotConstants.ClimberReleasegoal);
        // }
        // else{
        //    climberSubsystem.stop(); 
        // }
        
        // controller1.button(5).whileTrue(climberSubsystem.runOnce(() -> climberSubsystem.goTo(RobotConstants.ClimberClimbgoal)));
        // controller1.button(6).whileTrue(climberSubsystem.runOnce(() -> climberSubsystem.goTo(RobotConstants.ClimberReleasegoal)));
    }}