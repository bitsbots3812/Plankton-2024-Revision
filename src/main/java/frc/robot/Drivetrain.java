package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO: Add driving functions for autonomous
//Prevent override from working if nothing to lock
//Prevent override to ball if ball indexed

public class Drivetrain {

    CANSparkMax frontLeft;
    CANSparkMax frontRight;
    CANSparkMax rearLeft;
    CANSparkMax rearRight; 


    MecanumDrivePID drive;

    double kP = 0.0016; 
    double kI = 0.00018;
    double kD = (0.00003)/4; 

    PIDController autoAngle = new PIDController(kP, kI, kD);

    double throttleMin;
    double throttleMax;

    private double previousTargetAngle = 0;
    private double absoluteTargetAngle = 0;

    Drivetrain(int CanIdFL, int CanIdFR, int CanIdRL, int CanIdRR, boolean FLInversion, boolean FRInversion, boolean RLInversion, boolean RRInversion, double ThrottleMin, double ThrottleMax) {
        frontLeft  = new CANSparkMax(CanIdFL, MotorType.kBrushless);
        frontRight = new CANSparkMax(CanIdFR, MotorType.kBrushless);
        rearLeft   = new CANSparkMax(CanIdRL, MotorType.kBrushless);
        rearRight  = new CANSparkMax(CanIdRR, MotorType.kBrushless);

        drive = new MecanumDrivePID(frontLeft, rearLeft, frontRight, rearRight);

        frontRight.setInverted(FRInversion);
        rearRight.setInverted(RRInversion);
        frontLeft.setInverted(FLInversion);
        rearLeft.setInverted(RLInversion);

        frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);

        throttleMin = ThrottleMin;
        throttleMax = ThrottleMax;
    }

    //Drives the robot. Will turn to target designated by lock mode in provided robot state
    void drive(DriverControlsMap controls, ManipulatorControlsMap manipulatorControls, RobotState state, double NavxRotation, PixyInterface pixy, LimelightInterface limelight) {
        switch (state.currentLockMode) {
            case UNLOCKED: {
                if (state.fieldOriented) {
                    drive.driveCartesian(controls.getAxisValueWithThrottle(controls.STICK_Y_AXIS), 
                    controls.getAxisValueWithThrottle(controls.STICK_X_AXIS), 
                    controls.getAxisValueWithThrottle(controls.STICK_Z_ROTATION_AXIS), NavxRotation);
                }
                else {
                    drive.driveCartesian(controls.getAxisValueWithThrottle(controls.STICK_Y_AXIS), 
                    controls.getAxisValueWithThrottle(controls.STICK_X_AXIS), 
                    controls.getAxisValueWithThrottle(controls.STICK_Z_ROTATION_AXIS));
                }
                break;
            }
            case PIXY: {
                double rotation =  pixy.steeringSuggestion(state.selectedColor);
                if (rotation == -2) {
                    rotation = 0;
                }
                drive.driveCartesian(-1*manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_X_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), 
                manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_Y_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), rotation * 0.2);
                break;
            }
            case LIMELIGHT: {
                drive.driveCartesian(manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_X_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), 
                -1*manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_Y_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), 
                limelight.steeringSuggestion()*0.2);
                break;
            }
            case MANIPULATOR_OVERRIDE: {
                drive.driveCartesian(manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_X_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), 
                manipulatorControls.getAxisValueWithThrottle(manipulatorControls.LOCK_RELATIVE_Y_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT), 
                manipulatorControls.getAxisValueWithThrottle(manipulatorControls.ROTATION_AXIS, manipulatorControls.MANIPULATOR_THOTTLE_CONSTANT));
            }
        }
    }

    void driveAutonomous(double x, double y, double targetAngle, AHRS ahrs) {
        double currentAngle = ahrs.getAngle();
        if (targetAngle != previousTargetAngle) {
            previousTargetAngle = targetAngle;
            absoluteTargetAngle = currentAngle + targetAngle;
        }
        //double diff = (absoluteTargetAngle - currentAngle);
        //while (diff > 360) diff -= 360;
        //while (diff < - 360) diff += 360;
        //double normalized = Util.map(diff, -180, 180, -1, 1);


        double amount = autoAngle.calculate(currentAngle, targetAngle);
        //if (Math.abs(normalized) > 0.01) {
        //    if (normalized > 0) {
        //        drive.driveCartesian(y, x, Util.rangeLimit(0.05, .25, normalized));
        //    } else {
        //        drive.driveCartesian(y, x, Util.rangeLimit(-0.25, -0.05, normalized));
        //    }
        //} else {
        //    drive.driveCartesian(y, x, 0);
        //}

        //float amt = Util.error(normalized, )

        //drive.driveCartesian(y, x, 0.3 * Util.rangeLimit(-1, 1, (targetAngle - currentAngle) * 0.01));

        drive.driveCartesian(y, x, amount);

        SmartDashboard.putNumber("NAVX Angle: ", ahrs.getAngle());
        SmartDashboard.putNumber("Absolute Target Angle: ", absoluteTargetAngle);
        SmartDashboard.putNumber("Target Angle: ", targetAngle);
        SmartDashboard.putNumber("Previous Target Angle: ", previousTargetAngle);
        SmartDashboard.putNumber("Error ", Util.error(ahrs.getAngle(), absoluteTargetAngle));
    }

    void brake() {
        drive.driveCartesian(0, 0, 0);
    }
}
