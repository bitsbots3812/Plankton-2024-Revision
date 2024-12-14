package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class CurrentLimitedMotorController {
    PowerDistribution pdp;
    private MotorController motor;
    int pdpPort;
    double maxCurrent;
    private boolean disabled;

    CurrentLimitedMotorController(MotorController Motor, PowerDistribution PDP, int PDPPort, double MaxCurrent) {
        motor = Motor;
        pdp = PDP;
        pdpPort = PDPPort;
        maxCurrent = MaxCurrent;
    }

    void set(double value) {
        motor.set(value);
        /*
        if (!disabled) {
            motor.set(value);
            if (pdp.getCurrent(pdpPort) >= maxCurrent) {
                disabled = true;
            }
        }
        else {
            motor.set(0);
        }
        */
    }

    void reset() {
        disabled = false;
    }

    boolean isDisabled() {
        return disabled;
    }
}
