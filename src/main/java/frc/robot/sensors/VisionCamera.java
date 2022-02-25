package frc.robot.sensors;

import org.json.JSONObject;

public class VisionCamera {
    
    public VisionCamera() {

    }

    public double[] getVisionArray(String message) {
        double[] visionArray = new double[3];

        try {
            JSONObject jsonString = new JSONObject(message);

            visionArray[0] = (jsonString.getDouble("Distance"))/39.37;
            // System.out.println(visionArray[0]);
            visionArray[1] = jsonString.getDouble("Angle");

            if(visionArray[0] > 160) {
                visionArray[2] = jsonString.getDouble("Confidence");
            }
            else {
                visionArray[2] = 0;
            }


            // System.out.println(visionArray[0]);

            return visionArray;
        }
        catch(Exception e) {
            // System.out.println("Didn't receive message");
        }

        visionArray[0] = -1;
        visionArray[1] = -100;
        visionArray[2] = 0;
        return visionArray;
        
    }

}