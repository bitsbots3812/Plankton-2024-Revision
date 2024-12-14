package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class NavXInterface {
    AHRS navx;
    private double angleOffset = 0;

    double getAngle () {
        return navx.getAngle() + angleOffset;
    }

    void reset(double offset) {
        navx.reset();
        angleOffset = offset;
    }

    NavXInterface (AHRS NavX) {
        navx = NavX;
    }
}
