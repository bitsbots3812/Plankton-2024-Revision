package frc.robot;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LedInterface {
    
    enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    
    CANdle leds;
    CANdleConfiguration ledsConfig = new CANdleConfiguration();

    int offset = 8;
    int lps = 21;

    NetworkTable driverStationLeds = NetworkTableInstance.getDefault().getTable("rgb");

    final int INTAKE_LEDS_ANIM_CHANNEL_1 = 0;
    final int INTAKE_LEDS_ANIM_CHANNEL_2 = 1;
    final int SHOOTER_LEDS_ANIM_CHANNEL_1 = 2;
    final int SHOOTER_LEDS_ANIM_CHANNEL_2 = 3;

    LedInterface (int canPort) {
        leds = new CANdle(canPort);

        ledsConfig.statusLedOffWhenActive = true;
        ledsConfig.disableWhenLOS = false;
        ledsConfig.stripType = CANdle.LEDStripType.GRB;
        ledsConfig.brightnessScalar = 1;
        ledsConfig.vBatOutputMode = CANdle.VBatOutputMode.Modulated;

        leds.configAllSettings(ledsConfig);
    }

    //Returns an animation of specified type. Will use passed values if applicable, for values not taken as inputs, arbitarary defaults will be applied.
    Animation getAnimation(AnimationTypes type, int r, int g, int b, int w, double speed, int numLed, Direction direction, int ledOffset) {
        switch (type) {
            case ColorFlow:
                return new ColorFlowAnimation(r, g, b, w, speed, numLed, direction, ledOffset);
            case Fire:
                return new FireAnimation(1, speed, numLed, 0.5, 0.5, false, ledOffset);
            case Larson:
                return new LarsonAnimation(r, g, b, w, speed, numLed, LarsonAnimation.BounceMode.Back, 5, ledOffset);
            case Rainbow:
                return new RainbowAnimation(1, speed, numLed, false, ledOffset);
            case RgbFade:
                return new RgbFadeAnimation(1, speed, numLed, ledOffset);
            case SingleFade:
                return new SingleFadeAnimation(r, g, b, w, speed, numLed, ledOffset);
            case Strobe:
                return new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
            case Twinkle:
                return new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent64, ledOffset);
            case TwinkleOff:
                return new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent64, ledOffset);
            default:
                return new ColorFlowAnimation(r, g, b, w, speed, numLed, direction, ledOffset);
        }
    }

    void setAllSolid(int r, int g, int b) {
        leds.setLEDs(r, g, b, 255, offset, lps*4);
    }

    void setDriverStation(int r, int g, int b, boolean blinkMode) {
        double color[] = {r, g, b};
        driverStationLeds.getEntry("color").setDoubleArray(color);
        if (blinkMode) {
            driverStationLeds.getEntry("delay").setNumber(70);
            driverStationLeds.getEntry("effect").setString("pulse");
        }
        else {
            driverStationLeds.getEntry("delay").setNumber(0);
            driverStationLeds.getEntry("effect").setString("pulse");
        }
    }

    void setIntakeLedsSolid(int r, int g, int b) {
        leds.clearAnimation(INTAKE_LEDS_ANIM_CHANNEL_1);
        leds.clearAnimation(INTAKE_LEDS_ANIM_CHANNEL_2);
        Timer.delay(0.1);
        leds.setLEDs(r, g, b, 0, offset, lps*2);

    }

    void setIntakeLedsAnimated(int r, int g, int b) {
        int w =255;
        double speed = 0.7;
        int numLed = lps;
        Direction direction = Direction.Forward;

        Animation aA = new ColorFlowAnimation( r,  g,  b,  w,  speed,  numLed,  direction, offset);
        Animation aB = new ColorFlowAnimation( r,  g,  b,  w,  speed,  numLed,  direction, offset + numLed);
        leds.animate(aA, INTAKE_LEDS_ANIM_CHANNEL_1);
        leds.animate(aB, INTAKE_LEDS_ANIM_CHANNEL_2);
    }
    
    void setIntakeLedsAnimated(int r, int g, int b, AnimationTypes AnimationType) {
        int w =255;
        double speed = 0.7;
        int numLed = lps;
        Direction direction = Direction.Forward;

        Animation aA = getAnimation(AnimationType, r,  g,  b,  w,  speed,  numLed,  direction, offset);
        Animation aB = getAnimation(AnimationType, r,  g,  b,  w,  speed,  numLed,  direction, offset + numLed);
        leds.animate(aA, INTAKE_LEDS_ANIM_CHANNEL_1);
        leds.animate(aB, INTAKE_LEDS_ANIM_CHANNEL_2);
    }

    void setShooterLedsSolid(int r, int g, int b) {
        leds.clearAnimation(SHOOTER_LEDS_ANIM_CHANNEL_1);
        leds.clearAnimation(SHOOTER_LEDS_ANIM_CHANNEL_2);
        Timer.delay(0.1);
        leds.setLEDs(r, g, b, 0, offset + lps*2, lps*2);
    }

    void setShooterLedsAnimated(int r, int g, int b) {
        int w =255;
        double speed = 0.7;
        int numLed = lps;
        Direction direction = Direction.Backward;

        Animation aA = new ColorFlowAnimation( r,  g,  b,  w,  speed,  numLed,  direction, offset + numLed * 2);
        Animation aB = new ColorFlowAnimation( r,  g,  b,  w,  speed,  numLed,  direction, offset + numLed * 3);
        leds.animate(aA, SHOOTER_LEDS_ANIM_CHANNEL_1);
        leds.animate(aB, SHOOTER_LEDS_ANIM_CHANNEL_2);
    }

    void setShooterLedsAnimated(int r, int g, int b, AnimationTypes AnimationType) {
        int w =255;
        double speed = 0.7;
        int numLed = lps*2;
        Direction direction = Direction.Backward;

        Animation aA = getAnimation(AnimationType, r, g, b, w, speed, numLed, direction, offset);
        Animation aB = getAnimation(AnimationType, r,  g,  b,  w,  speed,  numLed,  direction, offset + numLed);
        leds.animate(aA, SHOOTER_LEDS_ANIM_CHANNEL_1);
        leds.animate(aB, SHOOTER_LEDS_ANIM_CHANNEL_2);
    }

    void showBatteryVoltage(double volts){
        int r = (int)Util.map(volts, 11.9, 12.3, 255, 0);
        int g = (int)Util.map(volts, 11.9, 12.3, 0, 255);
        leds.setLEDs(r, g, 0, 0, 0, offset);
    }
}
