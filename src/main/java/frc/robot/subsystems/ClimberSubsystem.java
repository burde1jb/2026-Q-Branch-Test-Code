package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {
    private SparkFlex climberMotor;
    private AbsoluteEncoder climberEncoder;
    private final double rangeOffset = RobotConstants.ClimberrangeOffset;
    private final double encoderOffset = RobotConstants.ClimberencoderOffset;

    public ClimberSubsystem() {
        climberMotor = new SparkFlex(RobotConstants.ClimbermotorCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        climberEncoder = climberMotor.getAbsoluteEncoder();
    }

    public void goTo(double encoderGoal) {
        if ((climberEncoder.getPosition()) < (encoderGoal - rangeOffset)) {
            this.retract();
        } else if ((climberEncoder.getPosition()) > (encoderGoal + rangeOffset)) {
            this.extend();
        } else {
            this.stop();
        }
    }

    public boolean wentTo(double encoderGoal, double extremaValue) {
        if ((climberEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
            this.retract();
            return false;
        } else if ((climberEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
            this.extend();
            return false;
        } else {
            this.stop();
            return true;
        }
    }

    public void extend() {
        climberMotor.set(RobotConstants.ClimberClimbpower);
    }

    public void retract() {
        climberMotor.set(RobotConstants.ClimberReleasepower);
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public boolean encoderCheck(double distance) {
        if (climberEncoder.getPosition() == distance) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", (climberEncoder.getPosition()));
    }
}
