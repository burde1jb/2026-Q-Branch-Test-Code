package frc.robot.subsystems;
//Import all libraries needed to call up the systems used
//Here we call up Spark Flex motor controller because that is the controller we chose to use on this subsystem
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    //Name the motors and sensors
    SparkFlex intakeMotor;

    public AlgaeIntakeSubsystem() {
        //Define the motor with the CAN ID value and type of motor (brushed or brushless)
        intakeMotor = new SparkFlex(RobotConstants.AlgaeIntakeCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }
//Set the commands we want to use int he COMMAND portion
//Here we use a boolean because we just want set values, no variance based on controller values
    public void AlgaeIntakeOn(boolean forward) {
        if (forward) {
            intakeMotor.set(RobotConstants.AlgaeIntakeOnspeed);
        } else {
            intakeMotor.set(RobotConstants.AlgaeIntakeOutspeed);
        }
    }

    public void AlgaeIntakeOff() {
        intakeMotor.stopMotor();
    }

    public void AlgaeIntakeSlow(boolean forward) {
        if (forward) {
            intakeMotor.set(RobotConstants.AlgaeIntakeSlowspeed);
        } else {
            intakeMotor.set(-RobotConstants.AlgaeIntakeSlowspeed);
        }
    }
}