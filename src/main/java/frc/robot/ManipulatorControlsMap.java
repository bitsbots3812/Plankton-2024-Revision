package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/* 
Note: The x and y axis are different for the manipulator controls than the driver contols

The X axis controls distance from the lock point (L)
The Y axis controls rotation around the lock point

   [Y]
    \
     \
(L)  ||-------[X]
     /
    /

*/

public class ManipulatorControlsMap {
    XboxController controller;
    Joystick stick;
    private boolean isXboxController = false;

    SingleAxisMap LOCK_RELATIVE_X_AXIS;
    SingleAxisMap LOCK_RELATIVE_Y_AXIS;
    SingleAxisMap ROTATION_AXIS;
    SingleAxisMap FIRE_TRIGGER;
    SingleAxisMap HOOD_ADJUSTMENT_AXIS;
    SingleAxisMap MANIPULATOR_OVERRIDE;

    POVBind CLIMBER_UP;
    POVBind CLIMBER_DOWN;
    POVBind DEPLOY_INTAKE;
    POVBind RETRACT_INTAKE;

    int PIXY_TRACK_BUTTON;
    int LIMELIGHT_TARGET_TRACK_BUTTON;
    int COLOR_TO_RED_BUTTON;
    int COLOR_TO_BLUE_BUTTON;
    int INTAKE_BUTTON;
    int INTAKE_REJECT_BUTTON;
    int SHOOTER_MODE_TOGGLE_BUTTON;
    int INTAKE_STOP_OVERRIDE_BUTTON;

    double MANIPULATOR_THOTTLE_CONSTANT;

    ManipulatorControlsMap(Joystick Stick, int lockRelativeXMap, boolean xInverted, boolean xSquared, int lockRelativeYMap, 
    boolean yInverted, boolean ySquared, int rotationMap, boolean rotationInverted, boolean rotationSquared, int triggerMap, 
    boolean triggerInverted, boolean triggerSquared, int hoodMap, boolean hoodInverted, boolean hoodSquared, 
    int overrideMap, boolean overrideInverted, boolean overrideSquared,
    int pixyTrackButton, int limelightTrackButton, int colorToRedButton, int colorToBlueButton, 
    int intakeButton, int intakeRejectButton, int shooterModeToggleButton, int intakeStopOverrideButton, 
    int climberUpValue, int climberUpIndex, int climberDownValue, int climberDownIndex, 
    int deployIntakeValue, int deployIntakeIndex, int retractIntakeValue, int retractIntakeIndex, double throttleConstant) {
        stick = Stick;
        PIXY_TRACK_BUTTON = pixyTrackButton;
        LIMELIGHT_TARGET_TRACK_BUTTON = limelightTrackButton;
        COLOR_TO_RED_BUTTON = colorToRedButton;
        COLOR_TO_BLUE_BUTTON = colorToBlueButton;
        INTAKE_BUTTON = intakeButton;
        INTAKE_REJECT_BUTTON = intakeRejectButton;
        SHOOTER_MODE_TOGGLE_BUTTON = shooterModeToggleButton;
        INTAKE_STOP_OVERRIDE_BUTTON = intakeStopOverrideButton;

        MANIPULATOR_THOTTLE_CONSTANT = throttleConstant;

        LOCK_RELATIVE_X_AXIS = new SingleAxisMap(lockRelativeXMap, xInverted, xSquared);
        LOCK_RELATIVE_Y_AXIS = new SingleAxisMap(lockRelativeYMap, yInverted, ySquared);
        FIRE_TRIGGER = new SingleAxisMap(triggerMap, triggerInverted, triggerSquared);
        HOOD_ADJUSTMENT_AXIS = new SingleAxisMap(hoodMap, hoodInverted, hoodSquared);
        ROTATION_AXIS = new SingleAxisMap(rotationMap, rotationInverted, rotationSquared);
        MANIPULATOR_OVERRIDE = new SingleAxisMap(overrideMap, overrideInverted, overrideSquared);

        CLIMBER_UP = new POVBind(climberUpValue, climberUpIndex);
        CLIMBER_DOWN = new POVBind(climberDownValue, climberDownIndex);
        DEPLOY_INTAKE = new POVBind(deployIntakeValue, deployIntakeIndex);
        RETRACT_INTAKE = new POVBind(retractIntakeValue, retractIntakeIndex);
    }
    
    ManipulatorControlsMap(XboxController Controller, int lockRelativeXMap, boolean xInverted, boolean xSquared, int lockRelativeYMap,
    boolean yInverted, boolean ySquared, int rotationMap, boolean rotationInverted, boolean rotationSquared, int triggerMap, 
    boolean triggerInverted, boolean triggerSquared, int hoodMap, boolean hoodInverted, boolean hoodSquared, 
    int overrideMap, boolean overrideInverted, boolean overrideSquared,
    int pixyTrackButton, int limelightTrackButton, int colorToRedButton, int colorToBlueButton,
    int intakeButton, int intakeRejectButton, int shooterModeToggleButton, int intakeStopOverrideButton, 
    int climberUpValue, int climberUpIndex, int climberDownValue, int climberDownIndex, 
    int deployIntakeValue, int deployIntakeIndex, int retractIntakeValue, int retractIntakeIndex, double throttleConstant) {
        controller = Controller;
        isXboxController = true;
        PIXY_TRACK_BUTTON = pixyTrackButton;
        LIMELIGHT_TARGET_TRACK_BUTTON = limelightTrackButton;
        COLOR_TO_RED_BUTTON = colorToRedButton;
        COLOR_TO_BLUE_BUTTON = colorToBlueButton;
        INTAKE_BUTTON = intakeButton;
        INTAKE_REJECT_BUTTON = intakeRejectButton;
        SHOOTER_MODE_TOGGLE_BUTTON = shooterModeToggleButton;
        INTAKE_STOP_OVERRIDE_BUTTON = intakeStopOverrideButton;

        MANIPULATOR_THOTTLE_CONSTANT = throttleConstant;

        LOCK_RELATIVE_X_AXIS = new SingleAxisMap(lockRelativeXMap, xInverted, xSquared);
        LOCK_RELATIVE_Y_AXIS = new SingleAxisMap(lockRelativeYMap, yInverted, ySquared);
        FIRE_TRIGGER = new SingleAxisMap(triggerMap, triggerInverted, triggerSquared);
        HOOD_ADJUSTMENT_AXIS = new SingleAxisMap(hoodMap, hoodInverted, hoodSquared);
        ROTATION_AXIS = new SingleAxisMap(rotationMap, rotationInverted, rotationSquared);
        MANIPULATOR_OVERRIDE = new SingleAxisMap(overrideMap, overrideInverted, overrideSquared);

        CLIMBER_UP = new POVBind(climberUpValue, climberUpIndex);
        CLIMBER_DOWN = new POVBind(climberDownValue, climberDownIndex);
        DEPLOY_INTAKE = new POVBind(deployIntakeValue, deployIntakeIndex);
        RETRACT_INTAKE = new POVBind(retractIntakeValue, retractIntakeIndex);
    }

    //gets value from an axis, with inversion and squaring applied
    double getAxisValue(SingleAxisMap axis) {
        double value;
        if (!isXboxController) {
            value = stick.getRawAxis(axis.AXIS_MAP);
        }
        else {
            value = controller.getRawAxis(axis.AXIS_MAP);
        }
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
    double getAxisValueWithThrottle(SingleAxisMap axis, double throttleValue) {
        //assuming throttle is a range from -1 to 1
        //converts to range from 0 to 1
        double value = getAxisValue(axis)*((throttleValue+1)/2);
        return value;
    }

    //wrappers for button functions
    boolean getButtonReleased(int button) {
        if (isXboxController) {
            return controller.getRawButtonReleased(button);
        }
        else {
            return stick.getRawButtonReleased(button);
        }
    }

    
    boolean getButtonPressed(int button) {
        if (isXboxController) {
            return controller.getRawButtonPressed(button);
        }
        else {
            return stick.getRawButtonPressed(button);
        }
    }

    boolean getButton(int button) {
        if (isXboxController) {
            return controller.getRawButton(button);
        }
        else {
            return stick.getRawButton(button);
        }
    }

    boolean getButton(POVBind bind) {
        if (isXboxController) {
            return controller.getPOV(bind.index) == bind.angle;
        }
        else {
            return stick.getPOV(bind.index) == bind.angle;
        }
    }
}
