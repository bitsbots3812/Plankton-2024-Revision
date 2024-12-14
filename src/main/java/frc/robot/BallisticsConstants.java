package frc.robot;

public class BallisticsConstants {
    double limelightMountHeight, limelightMountAngle, targetHeight, targetRadius, targetDepth, optimalRange, inRangeMin, inRangeMax;

    BallisticsConstants (double MountHeight, double MountAngle, double TargetHeight, double TargetRadius, double TargetDepth, double OptimalRange, double InRangeMin, double InRangeMax) {
        limelightMountHeight = MountHeight;
        limelightMountAngle = MountAngle;
        targetHeight = TargetHeight;
        targetRadius = TargetRadius;
        targetDepth = TargetDepth;
        optimalRange = OptimalRange;
        inRangeMin = InRangeMin;
        inRangeMax = InRangeMax;
    }
}
