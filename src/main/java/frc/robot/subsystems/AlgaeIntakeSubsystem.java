package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    SparkFlex intakeMotor;

    public AlgaeIntakeSubsystem() {
        intakeMotor = new SparkFlex(RobotConstants.AlgaeIntakeCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }

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