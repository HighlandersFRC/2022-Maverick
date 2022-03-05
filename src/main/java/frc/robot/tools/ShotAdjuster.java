package frc.robot.tools;

public class ShotAdjuster {
    
    private double rpmAdjuster = 0;;
    private double hoodAdjuster = 0;;

    public double getRPMAdjustment() {
        return rpmAdjuster;
    }

    public double getHoodAdjustment() {
        return hoodAdjuster;
    }

    public void increaseRPM() {
        rpmAdjuster = rpmAdjuster + 10;
    }

    public void decreaseRPM() {
        rpmAdjuster = rpmAdjuster - 10;
    }

    public void increaseHood() {
        hoodAdjuster = hoodAdjuster + 0.25;
    }

    public void decreaseHood() {
        hoodAdjuster = hoodAdjuster - 0.25;
    }

}




