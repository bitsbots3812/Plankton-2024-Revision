package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Verify Team Number
//TODO: add functions for targeting.
public class LimelightInterface {
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean offsetOn = false;

    void setOffset(boolean state) {
        offsetOn = state;
    }

    boolean getOffsetState() {
        return offsetOn;
    }

    double steeringSuggestion() {
        //Covnert angle to steering value from -1 (left) to 1 (right)
        //limelight angle range is from -29.8 to 29.8 degrees
        //limelight is NOT mounted inverted
        //TODO: Fix Skew (don't forget multipliers)
        double tx = 0;
        if (offsetOn) {
            tx = limelight.getEntry("tx").getDouble(0);
            if (tx != 0) {
                tx += 1;
            }
        }
        else {
            tx = limelight.getEntry("tx").getDouble(0);
        }
        //TODO: fix possible out of range
        double value = (1*((limelight.getEntry("tx").getDouble(0) + 4)/29.8)) + (-0*(limelight.getEntry("ts").getDouble(0)/90));
        SmartDashboard.putNumber("Limelight Steering Suggestion: ", value);
        return value;
    }

    boolean isTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1.0;
    }

    //true on; fasle off
    void setLight(boolean state) {
        //Limelight ledMode
        //0 Mode in Current Pipeline
        //1 Off
        //2 Blink
        //3 On
        if (state) 
            limelight.getEntry("ledMode").setNumber(0);
        else
            limelight.getEntry("ledMode").setNumber(1);
    }

    //Returns distance to target (-1 if no valid target)
    double distToTarget(BallisticsConstants constants) {
        if (isTarget()) {
            double distance = (constants.targetHeight - constants.limelightMountHeight)/Math.tan(Math.toRadians(constants.limelightMountAngle + limelight.getEntry("ty").getDouble(0)))+constants.targetRadius;
            return distance;
        }
        else {
            return -1;
        }
    }
}
