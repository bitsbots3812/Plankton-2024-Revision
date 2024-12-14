package frc.robot;

//TODO: Add state management and auotmatic deployment
public class ClimberInterface {
    /*public enum ClimberState {
        UP,
        DOWN,
        UNKNOWN
    }

    ClimberState currentState = ClimberState.UNKNOWN;
    ClimberState targetState;
    */
    
    CurrentLimitedMotorController climberController;

    ClimberInterface (CurrentLimitedMotorController motorController) {
        climberController = motorController;
    }

    void extend() {
        climberController.set(1);
    }

    void retract() {
        climberController.set(-1);
    }

    void stop() {
        climberController.set(0);
    }
}
