package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

//TODO:
//Change method of applying throttle control
//Allow whether an axis is affected by throttle or not to be a configurable option
//Add parameter for whether rumble is enabled or not
//Add button support


public class DriverControlsMap {
    
    Joystick stick;

    SingleAxisMap STICK_X_AXIS;
    SingleAxisMap STICK_Y_AXIS;
    SingleAxisMap STICK_Z_ROTATION_AXIS;
    SingleAxisMap STICK_THROTTLE_AXIS;

    int RESET_YAW_BUTTON;
    int TOGGLE_FIELD_ORIENTED_DRIVE_BUTTON;
    int BRAKE_BUTTON;

    DriverControlsMap(Joystick joystick, int xAxisMap, boolean xInverted, boolean xSquared, 
    int yAxisMap, boolean yInverted, boolean ySquared, int zrAxisMap, boolean zrInverted, boolean zrSquared,
    int throttleAxisMap, boolean throttleInverted, boolean throttleSquared, int resetYawButton, int toggleFieldOrientedButton, 
    int brakeButton) {
        stick = joystick;
        STICK_X_AXIS = new SingleAxisMap(xAxisMap, xInverted, xSquared);
        STICK_Y_AXIS = new SingleAxisMap(yAxisMap, yInverted, ySquared);
        STICK_Z_ROTATION_AXIS = new SingleAxisMap(zrAxisMap, zrInverted, zrSquared);
        STICK_THROTTLE_AXIS = new SingleAxisMap(throttleAxisMap, throttleInverted, throttleSquared);

        RESET_YAW_BUTTON = resetYawButton;
        TOGGLE_FIELD_ORIENTED_DRIVE_BUTTON = toggleFieldOrientedButton;
        BRAKE_BUTTON = brakeButton;
    }

    //gets value from an axis, with inversion and squaring applied
    double getAxisValue(SingleAxisMap axis) {
        double value = stick.getRawAxis(axis.AXIS_MAP);
        if (axis.SQUARED) {
            if(value > 0) {
                value = value * value;
            }
            else if(value < 0) {
                value = -1*value * value;
            }
            //else: value is equal to 0 no action needed
        }
        if (axis.INVERTED) {
            value = -1*value;
        }
        return value;
    }

    //gets value from axis with throttle applied
    double getAxisValueWithThrottle(SingleAxisMap axis) {
        //assuming throttle is a range from -1 to 1
        //converts to range from 0 to 1
        double value = getAxisValue(axis)*((getAxisValue(STICK_THROTTLE_AXIS)+1)/2);
        return value;
    }

    double getAxisValueWithThrottle(SingleAxisMap axis, double min, double max) {
        //assuming throttle is a range from -1 to 1
        //converts to range from min to max
        double value = getAxisValue(axis)*(Util.map(getAxisValue(STICK_THROTTLE_AXIS), -1, 1, min, max));
        return value;
    }

    //wrappers for button functions
    boolean getButtonReleased(int button) {
        return stick.getRawButtonReleased(button);
    }

    
    boolean getButtonPressed(int button) {
        return stick.getRawButtonPressed(button);
    }

    boolean getButton(int button) {
        return stick.getRawButton(button);
    }
}
