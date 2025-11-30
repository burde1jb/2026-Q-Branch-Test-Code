package frc.robot.subsystems;
//Imports used for a REV Blinkin LED Controller
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Import for the LaserCAN distance sensor
import au.grapplerobotics.LaserCan;

public class LEDSubsystem extends SubsystemBase {
    //Name the motors and sensors
    Spark LED;
    private LaserCan lc;

    public LEDSubsystem() {
        //Set the Channel (0) for the Blinkin
        LED = new Spark(0);
        lc = new LaserCan(0);
    }

    //This is the command to be called up later which sets the LED color
    //Color values need to be looked up with the REV website
    public void set(double powerInput) {
        LED.set(powerInput);
    }
}