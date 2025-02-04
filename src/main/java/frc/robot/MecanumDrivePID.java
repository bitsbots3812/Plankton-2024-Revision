// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/**
 * A class for driving Mecanum drive platforms.
 *
 * <p>Mecanum drives are rectangular with one wheel on each corner. Each wheel has rollers toed in
 * 45 degrees toward the front or back. When looking at the wheels from the top, the roller axles
 * should form an X across the robot. Each drive() function provides different inverse kinematic
 * relations for a Mecanum drive robot.
 *
 * <p>Drive base diagram:
 *
 * <pre>
 * \\_______/
 * \\ |   | /
 *   |   |
 * /_|___|_\\
 * /       \\
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a Mecanum drive
 * robot. Motor outputs for the right side are negated, so motor direction inversion by the user is
 * usually unnecessary.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 *
 * <p>Inputs smaller then {@value edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultDeadband} will
 * be set to 0, and larger values will be scaled so that the full range is still used. This deadband
 * value can be changed with {@link #setDeadband}.
 *
 * <p>RobotDrive porting guide: <br>
 * In MecanumDrive, the right side motor controllers are automatically inverted, while in
 * RobotDrive, no motor controllers are automatically inverted. <br>
 * {@link #driveCartesian(double, double, double, double)} is equivalent to RobotDrive's
 * mecanumDrive_Cartesian(double, double, double, double) if a deadband of 0 is used, and the ySpeed
 * and gyroAngle values are inverted compared to RobotDrive (eg driveCartesian(xSpeed, -ySpeed,
 * zRotation, -gyroAngle). <br>
 * {@link #drivePolar(double, double, double)} is equivalent to RobotDrive's
 * mecanumDrive_Polar(double, double, double)} if a deadband of 0 is used.
 */
@SuppressWarnings("removal")
public class MecanumDrivePID extends RobotDriveBase implements Sendable, AutoCloseable {
  private static int instances;

  private final MotorController m_frontLeftMotor;
  private final MotorController m_rearLeftMotor;
  private final MotorController m_frontRightMotor;
  private final MotorController m_rearRightMotor;

  private final SparkMaxPIDController m_FL_pidController;
  private final SparkMaxPIDController m_RL_pidController;
  private final SparkMaxPIDController m_FR_pidController;
  private final SparkMaxPIDController m_RR_pidController;

  //private final RelativeEncoder m_FL_encoder;
  //private final RelativeEncoder m_RL_encoder;
  //private final RelativeEncoder m_FR_encoder;
  //private final RelativeEncoder m_RR_encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private boolean m_reported;

  private boolean braking = true;

  void setBraking(boolean Braking) {
    braking = Braking;
  }

  boolean getBraking() {
    return braking;
  }

  @SuppressWarnings("MemberName")
  public static class WheelSpeeds {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

    /** Constructs a WheelSpeeds with zeroes for all four speeds. */
    public WheelSpeeds() {}

    /**
     * Constructs a WheelSpeeds.
     *
     * @param frontLeft The front left speed.
     * @param frontRight The front right speed.
     * @param rearLeft The rear left speed.
     * @param rearRight The rear right speed.
     */
    public WheelSpeeds(double frontLeft, double frontRight, double rearLeft, double rearRight) {
      this.frontLeft = frontLeft;
      this.frontRight = frontRight;
      this.rearLeft = rearLeft;
      this.rearRight = rearRight;
    }
  }

  /**
   * Construct a MecanumDrive.
   *
   * <p>If a motor needs to be inverted, do so before passing it in.
   *
   * @param frontLeftMotor The motor on the front-left corner.
   * @param rearLeftMotor The motor on the rear-left corner.
   * @param frontRightMotor The motor on the front-right corner.
   * @param rearRightMotor The motor on the rear-right corner.
   */
  public MecanumDrivePID(
    CANSparkMax frontLeftMotor,
    CANSparkMax rearLeftMotor,
    CANSparkMax frontRightMotor,
    CANSparkMax rearRightMotor) {
    requireNonNull(frontLeftMotor, "Front-left motor cannot be null");
    requireNonNull(rearLeftMotor, "Rear-left motor cannot be null");
    requireNonNull(frontRightMotor, "Front-right motor cannot be null");
    requireNonNull(rearRightMotor, "Rear-right motor cannot be null");


    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;


    allowedErr = 0;

    m_frontLeftMotor = frontLeftMotor;
    m_rearLeftMotor = rearLeftMotor;
    m_frontRightMotor = frontRightMotor;
    m_rearRightMotor = rearRightMotor;
    SendableRegistry.addChild(this, m_frontLeftMotor);
    SendableRegistry.addChild(this, m_rearLeftMotor);
    SendableRegistry.addChild(this, m_frontRightMotor);
    SendableRegistry.addChild(this, m_rearRightMotor);
    instances++;
    SendableRegistry.addLW(this, "MecanumDrive", instances);

    frontLeftMotor.restoreFactoryDefaults();
    rearLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    rearRightMotor.restoreFactoryDefaults();

    m_FL_pidController = frontLeftMotor.getPIDController();
    m_RL_pidController = rearLeftMotor.getPIDController();
    m_FR_pidController = frontRightMotor.getPIDController();
    m_RR_pidController = rearRightMotor.getPIDController();

    //m_FL_encoder = frontLeftMotor.getEncoder();
    //m_RL_encoder = rearLeftMotor.getEncoder();
    //m_FR_encoder = frontRightMotor.getEncoder();
    //m_RR_encoder = rearRightMotor.getEncoder();


    // set PID coefficients
    m_FL_pidController.setP(kP);
    m_FL_pidController.setI(kI);
    m_FL_pidController.setD(kD);
    m_FL_pidController.setIZone(kIz);
    m_FL_pidController.setFF(kFF);
    m_FL_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_RL_pidController.setP(kP);
    m_RL_pidController.setI(kI);
    m_RL_pidController.setD(kD);
    m_RL_pidController.setIZone(kIz);
    m_RL_pidController.setFF(kFF);
    m_RL_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_FR_pidController.setP(kP);
    m_FR_pidController.setI(kI);
    m_FR_pidController.setD(kD);
    m_FR_pidController.setIZone(kIz);
    m_FR_pidController.setFF(kFF);
    m_FR_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_RR_pidController.setP(kP);
    m_RR_pidController.setI(kI);
    m_RR_pidController.setD(kD);
    m_RR_pidController.setIZone(kIz);
    m_RR_pidController.setFF(kFF);
    m_RR_pidController.setOutputRange(kMinOutput, kMaxOutput);

    //int smartMotionSlot = 0;

   // m_FL_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
   // m_FL_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   // m_FL_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
   // m_FL_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//
   // m_RL_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
   // m_RL_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   // m_RL_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
   // m_RL_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//
   // m_FR_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
   // m_FR_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   // m_FR_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
   // m_FR_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//
   // m_RR_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
   // m_RR_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   // m_RR_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
   // m_RR_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this
   *     to implement field-oriented controls.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    if (!m_reported) {
      HAL.report(
          tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_MecanumCartesian, 4);
      m_reported = true;
    }

    ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);
    xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);

    var speeds = driveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);

    /*
    m_frontLeftMotor.set(speeds.frontLeft * m_maxOutput);
    m_frontRightMotor.set(speeds.frontRight * m_maxOutput);
    m_rearLeftMotor.set(speeds.rearLeft * m_maxOutput);
    m_rearRightMotor.set(speeds.rearRight * m_maxOutput);
*/
    SmartDashboard.putNumber("FL Target Speed: ", speeds.frontLeft);
    SmartDashboard.putNumber("FR Target Speed: ", speeds.frontRight);
    SmartDashboard.putNumber("RL Target Speed: ", speeds.rearLeft);
    SmartDashboard.putNumber("RR Target Speed: ", speeds.rearRight);

    if (xSpeed == 0 && ySpeed == 0 && zRotation == 0 && !braking) {
      m_FL_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
      m_FR_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
      m_RL_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
      m_RR_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    }
    else {
      m_FL_pidController.setReference(speeds.frontLeft * maxRPM, CANSparkMax.ControlType.kVelocity);
      m_FR_pidController.setReference(speeds.frontRight * maxRPM, CANSparkMax.ControlType.kVelocity);
      m_RL_pidController.setReference(speeds.rearLeft * maxRPM, CANSparkMax.ControlType.kVelocity);
      m_RR_pidController.setReference(speeds.rearRight * maxRPM, CANSparkMax.ControlType.kVelocity);
    }
    
    /*
    if (Math.abs(zRotation) > 0.2){
      m_RL_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
      m_RR_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    } else {
      m_RL_pidController.setReference(speeds.rearLeft * maxRPM, CANSparkMax.ControlType.kVelocity);
      m_RR_pidController.setReference(speeds.rearRight * maxRPM, CANSparkMax.ControlType.kVelocity);
    }
    */


    feed();
  }

  /**
   * Drive method for Mecanum platform.
   *
   * <p>Angles are measured counter-clockwise from straight ahead. The speed at which the robot
   * drives (translation) is independent from its angle or rotation rate.
   *
   * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
   * @param angle The angle around the Z axis at which the robot drives in degrees [-180..180].
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   */
  @SuppressWarnings("ParameterName")
  public void drivePolar(double magnitude, double angle, double zRotation) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_MecanumPolar, 4);
      m_reported = true;
    }

    driveCartesian(
        magnitude * Math.cos(angle * (Math.PI / 180.0)),
        magnitude * Math.sin(angle * (Math.PI / 180.0)),
        zRotation,
        0.0);
  }

  /**
   * Cartesian inverse kinematics for Mecanum platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this
   *     to implement field-oriented controls.
   * @return Wheel speeds.
   */
  @SuppressWarnings("ParameterName")
  public static WheelSpeeds driveCartesianIK(
      double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    // Compensate for gyro angle.
    Translation2d input = new Translation2d(ySpeed, xSpeed);
    input.rotateBy(Rotation2d.fromDegrees(-gyroAngle));

    double[] wheelSpeeds = new double[4];
    wheelSpeeds[MotorType.kFrontLeft.value] = input.getX() + input.getY() + zRotation;
    wheelSpeeds[MotorType.kFrontRight.value] = input.getX() - input.getY() - zRotation;
    wheelSpeeds[MotorType.kRearLeft.value] = input.getX() - input.getY() + zRotation;
    wheelSpeeds[MotorType.kRearRight.value] = input.getX() + input.getY() - zRotation;

    normalize(wheelSpeeds);

    return new WheelSpeeds(
        wheelSpeeds[MotorType.kFrontLeft.value],
        wheelSpeeds[MotorType.kFrontRight.value],
        wheelSpeeds[MotorType.kRearLeft.value],
        wheelSpeeds[MotorType.kRearRight.value]);
  }

  @Override
  public void stopMotor() {
    m_frontLeftMotor.stopMotor();
    m_frontRightMotor.stopMotor();
    m_rearLeftMotor.stopMotor();
    m_rearRightMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "MecanumDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MecanumDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty(
        "Front Left Motor Speed", m_frontLeftMotor::get, m_frontLeftMotor::set);
    builder.addDoubleProperty(
        "Front Right Motor Speed",
        () -> m_frontRightMotor.get(),
        value -> m_frontRightMotor.set(value));
    builder.addDoubleProperty("Rear Left Motor Speed", m_rearLeftMotor::get, m_rearLeftMotor::set);
    builder.addDoubleProperty(
        "Rear Right Motor Speed",
        () -> m_rearRightMotor.get(),
        value -> m_rearRightMotor.set(value));
  }
}
