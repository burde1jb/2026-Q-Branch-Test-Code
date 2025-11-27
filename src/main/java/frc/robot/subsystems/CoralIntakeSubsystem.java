package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
    SparkFlex intakeMotor;
    SparkFlexConfig intakeMotorConfig;
    public DigitalInput sensor;
    public SparkLimitSwitch beambreak;
    public LimitSwitchConfig beambreakconfig;

    public CoralIntakeSubsystem() {
        intakeMotor = new SparkFlex(RobotConstants.CoralIntakeCANid, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();
        sensor = new DigitalInput(RobotConstants.CoralIntakeSensorDIOid);
        beambreak = intakeMotor.getReverseLimitSwitch();

        intakeMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(true);
    }

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