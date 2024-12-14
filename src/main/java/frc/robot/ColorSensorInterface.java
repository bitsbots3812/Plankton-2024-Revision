package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorInterface {
    ColorSensorV3 sensor;
    int loadedProximity;

    ColorSensorInterface (ColorSensorV3 ColorSensor, int LoadedProximity) {
        sensor = ColorSensor;
        loadedProximity = LoadedProximity;
    }

    boolean isLoaded() {
        int proximity = sensor.getProximity();
        SmartDashboard.putNumber("Color Sensor Proximity: ", proximity);
        if (proximity >= loadedProximity) {
            return true;
        }
        else {
            return false;
        }
    }

    PixyInterface.BallColor getColor() {
        Color color = sensor.getColor();
        if (color.blue >= color.red) {
            return PixyInterface.BallColor.BLUE_BALL;
        }
        else {
            return PixyInterface.BallColor.RED_BALL;
        }
    }   
}
