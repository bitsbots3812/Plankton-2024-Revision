package frc.robot;


public class SingleAxisMap {
    int AXIS_MAP;
    boolean INVERTED;
    boolean SQUARED;
    SingleAxisMap(int map, boolean inverted, boolean squared) {
        AXIS_MAP = map;
        INVERTED = inverted;
        SQUARED = squared;
    }
}
