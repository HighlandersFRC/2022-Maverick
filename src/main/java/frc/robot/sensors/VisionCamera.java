package frc.robot.sensors;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Lights.LEDMode;

public class VisionCamera {
    

    public VisionCamera() {
    }

    double initTime = 0;
    Boolean firstTime = true;

    double currentTime = 0;

    private boolean hasLock = false;

    public boolean hasLock() {
        return hasLock;
    }

    public double[] getVisionArray(String message) {
        double[] visionArray = new double[3];

        try {
            JSONObject jsonString = new JSONObject(message);

            // System.out.println(message);

            visionArray[0] = (jsonString.getDouble("Distance"))/39.37;
            visionArray[1] = jsonString.getDouble("Angle");
            visionArray[2] = jsonString.getDouble("Confidence");


            if(visionArray[2] == 0) {
                hasLock = false;
                SmartDashboard.putBoolean("HAS LOCK-ON", false);
                System.out.println("-----------------------------------------------");
                visionArray[0] = -1;
                visionArray[1] = 0;
            }
            else {
                hasLock = true;
                SmartDashboard.putBoolean("HAS LOCK-ON", true);
                System.out.println("||||||||||||||||||||||||||||||||||||||||||||||||");
            }
            return visionArray;
        }
        catch(Exception e) {
        }

        visionArray[0] = -1;
        visionArray[1] = 0;
        visionArray[2] = 0;
        return visionArray;
        
    }

}