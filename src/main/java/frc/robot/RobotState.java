package frc.robot;


//Class desinged to hold current state of robot.
public class RobotState {
    boolean fieldOriented;

    enum LockMode {
        UNLOCKED,
        PIXY,
        LIMELIGHT,
        MANIPULATOR_OVERRIDE
    }
    LockMode currentLockMode;
    
    PixyInterface.BallColor selectedColor;

    boolean isLoaded = false;

    RobotState(boolean defaultFieldOriented, PixyInterface.BallColor defaultColor) {
        fieldOriented = defaultFieldOriented;
        currentLockMode = LockMode.UNLOCKED;
        selectedColor = defaultColor;
    }
}
