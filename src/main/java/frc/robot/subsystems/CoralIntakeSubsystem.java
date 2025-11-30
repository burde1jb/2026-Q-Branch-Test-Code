package frc.robot.subsystems;
//Import all libraries needed to call up the systems used
//Here we call up Spark Flex motor controller and an Absolute Encoder
//We also import many parts of the Spark Flex due to using a Digital Limit Switch (IR Beam Break)
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
    //Name the motors and sensors
    SparkFlex intakeMotor;
    SparkFlexConfig intakeMotorConfig;
    public DigitalInput sensor;
    public SparkLimitSwitch beambreak;
    public LimitSwitchConfig beambreakconfig;

    public CoralIntakeSubsystem() {
        //Define the motor with the CAN ID value and type of motor (brushed or brushless)
        intakeMotor = new SparkFlex(RobotConstants.CoralIntakeCANid, MotorType.kBrushless);
        //Because we are using the 
        intakeMotorConfig = new SparkFlexConfig();
        sensor = new DigitalInput(RobotConstants.CoralIntakeSensorDIOid);
        beambreak = intakeMotor.getReverseLimitSwitch();

        //Define what the limit switch is to do when forward or reverse are tripped
        intakeMotorConfig.limitSwitch
                .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
                //Old Version (deprecated)
                //.forwardLimitSwitchEnabled(true);
    }

//Set the commands we want to use in the COMMAND portion
    public void intakeOn(boolean forward) {
        if (forward) {
            intakeMotor.set(RobotConstants.CoralIntakeOnspeed);
        } else {
            intakeMotor.set(RobotConstants.CoralIntakeOutspeed);
        }
    }

    public void intakeOnAuton(boolean forward) {
        if (forward) {
            intakeMotor.set(RobotConstants.CoralIntakeOnspeedAuton);
        } else {
            intakeMotor.set(RobotConstants.CoralIntakeOutspeed);
        }
    }

    public void intakeOnBypass() {
        intakeMotor.set(RobotConstants.CoralIntakeOnspeed);
    }

    public void intakeOff() {
        intakeMotor.stopMotor();
    }

    public void intakeSlow(boolean forward) {
        if (forward) {
            intakeMotor.set(RobotConstants.CoralIntakeSlowspeed);
        } else {
            intakeMotor.set(-RobotConstants.CoralIntakeSlowspeed);
        }
    }

    public boolean isBeamBroken() {
        return beambreak.isPressed();
    }

    //This function was used to bypass pre-programmed limits in the Spark Flex
    //The application was for a gamepiece to break the IR beambreak sensor to turn off the motor
    //then we would press the button AGAIN to bypass the stop motor function
    //We can keep this for future use or adapt with the new behavior function
    public void limitSwitchOff(boolean forward) {
        if (forward) {
            intakeMotorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
            intakeMotor.configure(
                    intakeMotorConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        } else {
            intakeMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
            intakeMotor.configure(
                    intakeMotorConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }
    }
}