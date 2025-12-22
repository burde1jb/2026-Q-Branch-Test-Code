package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

import au.grapplerobotics.LaserCan;

public class ExtendoSubsystem extends SubsystemBase {
    SparkFlex ExtendoMotor;
    AbsoluteEncoder ExtendoEncoder;
    SparkFlexConfig ExtendoMotorConfig;
    public DigitalInput sensor;
    public SparkLimitSwitch magLimitSwitch;
    public LimitSwitchConfig magLimitSwitchConfig;
    private final double rangeOffset = RobotConstants.ExtendoRangeOffset;
    private final double encoderOffset = RobotConstants.ExtendoEncoderOffset;
    //private LaserCan lc;

    public ExtendoSubsystem() {
        ExtendoMotor = new SparkFlex(RobotConstants.ExtendoMotorCANid, MotorType.kBrushless);
        ExtendoEncoder = ExtendoMotor.getAbsoluteEncoder();
        ExtendoMotorConfig = new SparkFlexConfig();
        ExtendoMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(true);
        //lc = new LaserCan(0);

    }

    public double getEncoderMeasurement() {
        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;
        return (position);
    }

    public void goTo(double degrees) {

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < degrees) {
            ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        } else if ((position  - (2 * rangeOffset)) > (degrees)) {
            ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }

        // LaserCan.Measurement measurement = lc.getMeasurement();
        // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        //     System.out.println("The target is " + measurement.distance_mm + "mm away!");
        // } else {
        //     System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
        //     // You can still use distance_mm in here, if you're ok tolerating a clamped
        //     // value or an unreliable measurement.
        // }

        // var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        // if (measurement != null){
        //         if (position < degrees) {
        //             ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        //         } else if ((position  - (2 * rangeOffset)) > (degrees + encoderOffset)) {
        //             ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        //         }
        //         else {
        //             this.stop();
        //         }
        //     }
        // else {
        //     this.stop();
        // }
    }

    public void goToL4(double degrees) {

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < degrees) {
            ExtendoMotor.set(RobotConstants.ExtendoL4ExtendSpeed);
        } else if ((position  - (2 * rangeOffset)) > (degrees)) {
            ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }
    }

    public boolean wentTo(double degrees) {
        //LaserCan.Measurement measurement = lc.getMeasurement();

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < (degrees + encoderOffset)) {
            this.Extend();
            // ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
            return false;
        } else if ((position - (2 * rangeOffset)) > (degrees)) {
            this.Retract();
            // ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
            return false;
        } else {
            this.stop();
            return true;
        }

        // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        //     System.out.println("The target is " + measurement.distance_mm + "mm away!");
        // } else {
        //     System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
        //     // You can still use distance_mm in here, if you're ok tolerating a clamped
        //     // value or an unreliable measurement.
        // }

        // if (measurement != null){
        //     if (position < (degrees + encoderOffset)) {
        //         this.Extend();
        //         // ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        //         return false;
        //     } else if ((position - (2 * rangeOffset)) > (degrees + encoderOffset)) {
        //         this.Retract();
        //         // ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        //         return false;
        //     } else {
        //         this.stop();
        //         return true;
        //     }
        // }
        // else {
        //     this.stop();
        //     return false;
        // }
    }

    public void Extend() {
        if ((ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1 < (RobotConstants.ExtendoExtendL4)) {
        ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        }
        else {
            this.stop();
        }
        } 

    public void ExtendAuton(boolean forward) {
            if (((ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1 < (RobotConstants.ExtendoExtendAuton)) && forward) {
                ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
            }
            else {
                ExtendoMotor.set(0);
            }
        }

    public void Retract() {
        if ((ExtendoEncoder.getPosition() - rangeOffset + encoderOffset) % 1 > (RobotConstants.ExtendoRetract)){
        ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }
    } 

    public void RetractAuton(boolean forward) {
        if (((ExtendoEncoder.getPosition() - rangeOffset + encoderOffset) % 1 > (RobotConstants.ExtendoRetract)) && forward) {
             ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            ExtendoMotor.set(0);
        }
    }

    public void stop() {
        ExtendoMotor.stopMotor();
    }

    public boolean encoderCheck(double distance) {
        if (ExtendoEncoder.getPosition() == distance) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Extendo Encoder", (ExtendoEncoder.getPosition() + encoderOffset) % 1);

    }
}