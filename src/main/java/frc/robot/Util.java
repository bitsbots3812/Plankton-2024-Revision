package frc.robot;

public class Util {
    //return value approaches 0 as current value approaches target value
    //return range 0 to 1
    static double errorMagnitude(double current, double target) {
        if (target == 0) {
            if (current < 0) {
                return (1 / (current - 1)) + 1;
            }
            else {
                return -1 * ((1 / (current + 1)) + 1);
            }
        }
        else {
            if (current < target) {
                return 1 + (target / (current - (2 * target)));
            }
            else {
                return 1 - (target / current);
            }
        }
    }

    //return value approaches 0 as current value approaches target value
    //returns negative value if current value is less than target value
    //return range -1 to 1
    static double error(double current, double target) {
        if (target == 0) {
            if (current < 0) {
                return -1 * ((1 / (current - 1)) + 1);
            }
            else {
                return -1 * ((1 / (current + 1)) + 1);
            }
        }
        else {
            if (current < target) {
                return -1 * (1 + (target / (current - (2 * target))));
            }
            else {
                return 1 - (target / current);
            }
        }
    }

    static double maxLimit(double max, double value) {
        if (value > max) {
            return max;
        }
        else {
            return value;
        }
    }

    static double minLimit(double min, double value) {
        if (value < min) {
            return min;
        }
        else {
            return value;
        }
    }

    static double rangeLimit(double min, double max, double value) {
        if (value > max) {
            return max;
        }
        else if (value < min) {
            return min;
        }
        else {
            return value;
        }
    }

    // at 1.6 meters, best rpm is 4000 (FAKE)
    // at 3 meters, best rpm is 5700 (FAKE)
    // we are at 2 meters. what is the best RPM?
    // targetRPM = map(2, 1.6, 3, 4000, 5700);
    static double map(double x, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    static double mapLocked(double x, double fromLow, double fromHigh, double toLow, double toHigh) {
        double result = map(x, fromLow, fromHigh, toLow, toHigh);
        if (result > toHigh) return toHigh;
        if (result < toLow) return toLow;
        return result;
    }
}
