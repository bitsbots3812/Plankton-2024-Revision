// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
//import edu.wpi.first.wpilibj.SerialPort.Port;
//import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PixyInterface.BallColor;
import frc.robot.RobotState.LockMode;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /*
    +Y     <FRONT>
    ^   (1)-------(2)
    |   |\\      //| 
    |   | \\    // |
    |          |   |
    |   _______|   |
    |   | //    \\ |
    |   |//      \\|
    |   (3)-------(4)
    +----------------> +X
       */

  Drivetrain drive = new Drivetrain(
    1, //FL CAN ID 
    2, //FR CAN ID
    3, //RL CAN ID
    4, //RR CAN ID
    false, //FLInversion
    true, //FR Inversion
    false, //RL Inversion
    true,  //RR Inversion
    0.05, //Minimum throttle multiplier
    1 //Maximum throttle multiplier (Should not be greater than 1)
  );

  //PWM PORTS//
  /*
  0 - LEDS
  1 - Climber
  2 and 3 Intake arm 1 and 2
  4 - Intake Rotation
  */

  //PDP PORTS//
  /*
  13 - Climber motor
  4 and 11 Intake arm motors
  */
  final int HOOD_POTETNTIOMETER_PORT = 0;

  final int CLIMBER_MOTOR_PDP_PORT = 13; //confirmed
  final int INTAKE_ARM_MOTOR_PDP_PORT = 4;

  final int LED_CAN_PORT = 9;
  final int CLIMBER_PWM_PORT = 1;
  final int INTAKE_ARM_PWM_PORT = 2;
  final int SHOOTER_HOOD_PWM_PORT = 3;
  final int INTAKE_ROTATION_PWM_PORT = 4;

  final int DRIVER_CAMERA_SERVO_PWM_PORT = 5; //new

  Servo driverCamServo = new Servo(DRIVER_CAMERA_SERVO_PWM_PORT);
  double cameraAngle = 150;
  double camAngleMod = 0;

  //AnalogPotentiometer hoodPotentiometer = new AnalogPotentiometer(HOOD_POTETNTIOMETER_PORT);
  //Spark hoodAdjustment = new Spark(SHOOTER_HOOD_PWM_PORT);

  RobotState teleopDefaults = new RobotState(
    false, //Default Field oriented drive status
    PixyInterface.BallColor.RED_BALL //Default ball color
  );
  RobotState currentState = teleopDefaults;

  PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

  ColorSensorInterface colorSensor = new ColorSensorInterface(new ColorSensorV3(I2C.Port.kOnboard), 300);
  
  //Motor Can IDs
  //Shooter Motor Will be CAN ID 5
  //Indexer CAN ID 9
  final int SHOOTER_MOTOR_CAN_ID = 5;
  final int INDEXER_MOTOR_CAN_ID = 9;

  Spark intakeController = new Spark(INTAKE_ARM_PWM_PORT);

  IntakeShooterInterface fireControl = new IntakeShooterInterface(
    //TODO: Fix current limiting
    intakeController, //CurrentLimited controller for intake arm.
    new Spark(INTAKE_ROTATION_PWM_PORT), //Motor for intake rotation
    new VictorSPX(INDEXER_MOTOR_CAN_ID), //VictorSPX for indexer
    new Spark(SHOOTER_HOOD_PWM_PORT), //motor controller for hood
    new AnalogPotentiometer(HOOD_POTETNTIOMETER_PORT), //Shooter hood poteniometer
    new CANSparkMax(SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless), //Shooter Motor
    true, //Is Shooter inverted
    colorSensor, //color sensor interface
    currentState //robot state
  );  
  
  Joystick stick = new Joystick(0);

  XboxController manipulatorController = new XboxController(1);

  //TODO: 
  //Prevent Manipulator control w/out valid target
  //Interface Class for Limelight
  //Bindings for all controllers
  //AHRS Error Handling
  //Error handling in general
  //Joystick Detection?
  //Joystick Rumble With NAV-X Data - (working, although inefficiently)
  //Auto-set Field oriented drive offset

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  //Get a vaule from on axis control (with inversion applied) using: axisControlsMap.getAxisValue(axisControlsMap.AXIS); returns double from -1 to 1
  final DriverControlsMap extreme3DPro = new DriverControlsMap(
    stick, //Joystick object
    0, //X-Axis Map
    false, // Is X-Axis Inverted
    true, // Is X-Axis Squared
    1, //Y-Axis Map
    true, //Is Y-Axis Inverted
    true, // Is Y-Axis Squared
    2, //RZ Axis Map
    false, //Is RZ Inverted
    true, // Is RZ-Axis Squared
    3, //Throttle Axis Map
    true, //Is Throttle Axis Inverted
    false, // Is Throttle Axis Squared

    //Buttons:
    7, //Yaw Reset Button
    8, //Field Oriented Toggle Button
    1 //Brake Button
  );

  DriverControlsMap currentDriverControlsMap = extreme3DPro;

  /* 
Note: The x and y axis are different for the manipulator controls than the driver contols

The X axis controls distance from the lock point (L)
The Y axis controls rotation around the lock point

   [Y]
    \
     \
(L)  ||-------[X]
     /
    /

*/
  
  ManipulatorControlsMap xboxController = new ManipulatorControlsMap(
    manipulatorController, //Controller object
    XboxController.Axis.kLeftX.value, //Lock Relative Driving X Axis
    false, //X Inverted
    false, //X Squared
    XboxController.Axis.kLeftY.value, //Lock Relative Driving Y Axis
    false, //Y Inverted
    false, //Y Squared
    XboxController.Axis.kRightX.value, //manipulator rotation control axis
    false, //inverted
    false, //squared
    XboxController.Axis.kLeftTrigger.value, //Firing Trigger Axis
    false, //trigger Inverted
    false, //trigger Squared
    XboxController.Axis.kRightY.value, //Hood adsjustment axis
    false, //hood adjustment inverted
    false, //hood adjustment squared
    XboxController.Axis.kRightTrigger.value, //manual override
    false, //inverted
    false, //squared
    XboxController.Button.kLeftBumper.value, //Pixy Track Button
    XboxController.Button.kRightBumper.value, //Limelight Track Button
    XboxController.Button.kB.value, //Color to red button
    XboxController.Button.kX.value, //Color to blue button
    XboxController.Button.kA.value, //intake button
    XboxController.Button.kY.value, //intake reject button
    XboxController.Button.kBack.value, //toggle shooter mode
    XboxController.Button.kStart.value, //Override intake stop
    0, //Climber up POV Value
    0, //Climber up POV Index (Zero on controller w/ one POV)
    180, //Climber down POV value
    0, //Climber down POV index
    270, //Intake deploy POV value
    0, //Intake deploy index
    90, //Intake retract POV value
    0, //Intake retract POV index
    -0.5 //Throttle value during manipulator control range is from -1 to 1 with negative one being no throttle equivalent to axis
  );

  ManipulatorControlsMap currentManipulatorControlsMap = xboxController;
  

  PixyInterface pixy = new PixyInterface(
    LinkType.SPI, //Pixy Link type
    0.6, //Minimum target aspect ratio
    100, //Maximum target aspect ratio
    20, //Minimum block height
    20  //Minimum block width
  );

  LimelightInterface limelight = new LimelightInterface();
  
  BallisticsConstants robotRoom = new BallisticsConstants(
    0.387535, //limelight mount height (m)
    45, //Limelight mount angle (Degrees)
    2.6416, //Target Height (m)
    0.5, //Target Radius (m)
    0, //Target Depth (m)
    3.1, //Optimal Range to target (m). Distance at which firing occurs in autonomous
    2.9, //Minimum range to target considered in-range (m)
    3.4 //Maximum range to target considered in-range (m)
  );

  BallisticsConstants competitionField = new BallisticsConstants(
    0.387535, //limelight mount height (m)
    45, //Limelight mount angle (Degrees)
    2.6416, //Target Height (m)
    0.58, //Target Radius (m)
    0, //Target Depth (not used) (m)
    3.0, //Optimal Range to target (m). Distance at which firing occurs in autonomous
    2.7, //Minimum range to target considered in-range (m)
    3.15 //Maximum range to target considered in-range (m)
  );

  BallisticsConstants currentContstants = competitionField;

  //Current limited motor controller last two values are: pdp port and maximum current
  //TODO: Fix Current Limiting
  CurrentLimitedMotorController climberMotor = new CurrentLimitedMotorController(new Spark(CLIMBER_PWM_PORT), PDP, CLIMBER_MOTOR_PDP_PORT, 2000);
  ClimberInterface climber = new ClimberInterface(climberMotor);

  //LedInterface leds = new LedInterface(LED_PWM_PORT, 100);
  LedInterface leds = new LedInterface(LED_CAN_PORT);

  Timer autonomousTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driverCamServo.setAngle(cameraAngle);

    leds.setIntakeLedsSolid(100, 100, 100);

    Timer.delay(0.02);

    leds.setShooterLedsSolid(0, 50, 0);

    fireControl.setShooterTargetRPM(2800);
    leds.setDriverStation( 0, 0, 0, false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double volts = RobotController.getBatteryVoltage();
    leds.showBatteryVoltage(volts);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);

    if (isRed) {
      currentState.selectedColor = BallColor.RED_BALL;
      leds.setDriverStation(75, 0, 0, false);
    }
    else {
      currentState.selectedColor = BallColor.BLUE_BALL;
      leds.setDriverStation(0, 0, 75, false);
    }

    leds.setIntakeLedsAnimated(100, 100, 100);

    Timer.delay(0.1);

    leds.setShooterLedsAnimated(0, 255, 0);

    System.out.println("Auto selected: " + m_autoSelected);
    ahrs.reset();
    ahrs.resetDisplacement();
    limelight.setLight(true);
    autonomousTimer.reset();
    autonomousTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  //TODO: Set distance value
  public void autonomousPeriodic() {
    pixy.updateTargets();
    if (limelight.distToTarget(currentContstants) < currentContstants.optimalRange && !autonomousTimer.hasElapsed(5)) {
      drive.drive.driveCartesian(0, -0.1, limelight.steeringSuggestion()*0.2);
    }
    else if (!autonomousTimer.hasElapsed(10)){
      drive.drive.driveCartesian(0, 0, limelight.steeringSuggestion()*0.2);
      fireControl.fire();
    }
    else {
      fireControl.shooterSpinDown();
    }
    //leds.update();
    fireControl.update();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    fireControl.retractIntake();
    limelight.setLight(true);
    fireControl.shooterSpinDown();

    if (currentState.selectedColor == BallColor.RED_BALL) {
      leds.setDriverStation(75, 0, 0, false);
    }
    else {
      leds.setDriverStation(0, 0, 75, false);
    }

    leds.setIntakeLedsAnimated(100, 100, 100);

    Timer.delay(0.1);

    leds.setShooterLedsAnimated(0, 255, 0);

    SmartDashboard.putBoolean("Shooter Enabled: ", fireControl.shooterGetEnabled());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    pixy.updateTargets();
    //drive.driveCartesian(0,0,0,0);
/*
    double current0 = PDP.getCurrent(0);
    double current1 = PDP.getCurrent(1);
    double current14 = PDP.getCurrent(14);
    double current15 = PDP.getCurrent(15);

    

    SmartDashboard.putNumber("current0", current0);
    SmartDashboard.putNumber("current1", current1);
    SmartDashboard.putNumber("current14", current14);
    SmartDashboard.putNumber("current15", current15);

    double voltage = PDP.getVoltage();
    SmartDashboard.putNumber("voltage", voltage);
*/
    if (currentDriverControlsMap.getButtonReleased(currentDriverControlsMap.TOGGLE_FIELD_ORIENTED_DRIVE_BUTTON)) {
      currentState.fieldOriented = !currentState.fieldOriented;
    }
    if (currentDriverControlsMap.getButtonReleased(currentDriverControlsMap.RESET_YAW_BUTTON)) {
      ahrs.reset();
    }
    
    if (currentManipulatorControlsMap.getButtonReleased(currentManipulatorControlsMap.COLOR_TO_RED_BUTTON)) {
      currentState.selectedColor = PixyInterface.BallColor.RED_BALL;
      leds.setDriverStation(75, 0, 0, false);
    }
    if (currentManipulatorControlsMap.getButtonReleased(currentManipulatorControlsMap.COLOR_TO_BLUE_BUTTON)) {
      currentState.selectedColor = PixyInterface.BallColor.BLUE_BALL;
      leds.setDriverStation(0, 0, 75, false);
    }

    if (currentManipulatorControlsMap.getButtonPressed(currentManipulatorControlsMap.PIXY_TRACK_BUTTON) && pixy.steeringSuggestion(currentState.selectedColor) != -2) {
      currentState.currentLockMode = RobotState.LockMode.PIXY;
    }
    else if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.LIMELIGHT_TARGET_TRACK_BUTTON) && currentState.currentLockMode != RobotState.LockMode.LIMELIGHT) {
      if (limelight.isTarget()){
        currentState.currentLockMode = RobotState.LockMode.LIMELIGHT;
      }
    }
    else if (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.MANIPULATOR_OVERRIDE) > 0.9) {
      currentState.currentLockMode = RobotState.LockMode.MANIPULATOR_OVERRIDE;
    }
    
    if (((pixy.getChecksWithoutTarget() >= 10) && (currentState.currentLockMode == LockMode.PIXY)) || 
    currentManipulatorControlsMap.getButtonReleased(currentManipulatorControlsMap.PIXY_TRACK_BUTTON) || 
    currentManipulatorControlsMap.getButtonReleased(currentManipulatorControlsMap.LIMELIGHT_TARGET_TRACK_BUTTON) || 
    (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.MANIPULATOR_OVERRIDE) <= 0.1 && currentState.currentLockMode == RobotState.LockMode.MANIPULATOR_OVERRIDE)) {
      currentState.currentLockMode = RobotState.LockMode.UNLOCKED;
      manipulatorController.setRumble(RumbleType.kRightRumble, 0);
    }

    if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.SHOOTER_MODE_TOGGLE_BUTTON)) {
      fireControl.shooterSetEnalbed(!fireControl.shooterGetEnabled());
      SmartDashboard.putBoolean("Shooter Enabled: ", fireControl.shooterGetEnabled());
    }

    currentState.isLoaded = colorSensor.isLoaded();
    if (currentState.isLoaded){
      fireControl.shooterSpinUp();
      manipulatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
    }
    else {
      manipulatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    }

    switch (currentState.currentLockMode) {
      case UNLOCKED: {
        SmartDashboard.putString("Mode: ", "UNLOCKED");
        if (currentState.isLoaded){
          if (colorSensor.getColor() == BallColor.RED_BALL) {
            leds.setDriverStation(75, 0, 0, true);
          }
          else {
            leds.setDriverStation(0, 0, 75, true);
          }
        } 
        else {
          //leds.setDriverStation( 100, 100, 100, true);
        }
        
        break;
      }
      case PIXY: {
        SmartDashboard.putString("Mode: ", "PIXY");
        if (currentState.selectedColor == PixyInterface.BallColor.RED_BALL) {
          leds.setDriverStation(255, 0, 0, false);
        }
        else {
          leds.setDriverStation(0, 0, 255, false);
        }
        break;
      }
      case LIMELIGHT: {
        SmartDashboard.putString("Mode: ", "LIMELIGHT");
        SmartDashboard.putNumber("Distance From Target: ", limelight.distToTarget(currentContstants));
        if ((limelight.distToTarget(currentContstants) >= currentContstants.inRangeMin) && 
        (limelight.distToTarget(currentContstants) <= currentContstants.inRangeMax)) {
          leds.setDriverStation(0, 127, 0, false);
          if (currentState.isLoaded) {
            manipulatorController.setRumble(RumbleType.kRightRumble, 1);
          }
        }
        else {
          leds.setDriverStation(127, 127, 127, false);
          manipulatorController.setRumble(RumbleType.kRightRumble, 0);
        }
        break;
      }
      case MANIPULATOR_OVERRIDE: {
        leds.setDriverStation(127, 127, 127, false);
        SmartDashboard.putString("Mode: ", "MANIPULATOR_OVERRIDE");
      }
    }

    if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.DEPLOY_INTAKE)) {
      fireControl.deployIntake();
    }
    else if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.RETRACT_INTAKE)) {
      fireControl.retractIntake();
    }

    if(currentManipulatorControlsMap.getButtonPressed(currentManipulatorControlsMap.INTAKE_BUTTON)) {
      fireControl.deployIntake();
    }
    if (currentManipulatorControlsMap.getButtonReleased(currentManipulatorControlsMap.INTAKE_BUTTON)) {
      fireControl.retractIntake();
    }

    if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.INTAKE_BUTTON)) {
      fireControl.intake(false);
    }
    else if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.INTAKE_REJECT_BUTTON)) {
      fireControl.reject();
    }

    if (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.FIRE_TRIGGER) > 0.05) {
      if (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.FIRE_TRIGGER) > 0.95) {
        fireControl.fire();
      }
      else {
        fireControl.shooterSpinUp();
      }
    }
    else if (!currentState.isLoaded) {
      fireControl.shooterSpinDown();
    }

    if (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.HOOD_ADJUSTMENT_AXIS) > 0) {
      //fireControl.hoodDown(-0.01*currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.HOOD_ADJUSTMENT_AXIS));
    }
    else if(currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.HOOD_ADJUSTMENT_AXIS) < 0) {
      //fireControl.hoodUp(0.01*currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.HOOD_ADJUSTMENT_AXIS));
    }
    
    int contpov = currentManipulatorControlsMap.controller.getPOV();
    if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.CLIMBER_UP) || contpov == 45 || contpov == 315) {
      climber.extend();
      driverCamServo.setAngle(0);
    }
    else if (currentManipulatorControlsMap.getButton(currentManipulatorControlsMap.CLIMBER_DOWN) || contpov == 225 || contpov == 135) {
      climber.retract();
    }
    else {
      climber.stop();
    }

    //fireControl.setShooterTargetRPM(SmartDashboard.getNumber("Shooter Motor Target RPM:", 0));
    SmartDashboard.putNumber("Climber Motor Current: ", PDP.getCurrent(CLIMBER_MOTOR_PDP_PORT));
    SmartDashboard.putBoolean("Climber Motor Status: ", climberMotor.isDisabled());

    SmartDashboard.putNumber("Intake Arm Motor Current: ", PDP.getCurrent(INTAKE_ARM_MOTOR_PDP_PORT));

    //SmartDashboard.putNumber("Shooter Hood Potentiometer Value", hoodPotentiometer.get());
    //SmartDashboard.putBoolean("Intake Arm Motor Status:", intakeController.isDisabled());

    if (currentDriverControlsMap.getButton(currentDriverControlsMap.BRAKE_BUTTON)) {
      drive.brake();
    }
    else {
      drive.drive(currentDriverControlsMap, currentManipulatorControlsMap, currentState, ahrs.getAngle(), pixy, limelight);
    }

    if (currentDriverControlsMap.getButtonReleased(currentDriverControlsMap.BRAKE_BUTTON)) {
      //drive.drive.setBraking(false);
    }

    //Puts color sensor data in smartdashboard
    SmartDashboard.putBoolean("Ball Indexed", colorSensor.isLoaded());
    PixyInterface.BallColor detectedColor = colorSensor.getColor();
    if (detectedColor == BallColor.BLUE_BALL) {
      SmartDashboard.putString("Ball Color: ", "BLUE");
    }
    else {
      SmartDashboard.putString("Ball Color: ", "RED");
    }
    

    SmartDashboard.putBoolean(" Field Oriented Drive Enabled ", currentState.fieldOriented);
    SmartDashboard.putNumber("Throttle", (currentDriverControlsMap.getAxisValue(currentDriverControlsMap.STICK_THROTTLE_AXIS)+1)/2);
    SmartDashboard.putNumber("X-Axis", currentDriverControlsMap.getAxisValue(currentDriverControlsMap.STICK_X_AXIS));
    SmartDashboard.putNumber("Y-Axis", currentDriverControlsMap.getAxisValue(currentDriverControlsMap.STICK_Y_AXIS));
    SmartDashboard.putNumber("RZ-Axis", currentDriverControlsMap.getAxisValue(currentDriverControlsMap.STICK_Z_ROTATION_AXIS));
    SmartDashboard.putNumber("X-Axis (Throttled)", currentDriverControlsMap.getAxisValueWithThrottle(currentDriverControlsMap.STICK_X_AXIS));
    SmartDashboard.putNumber("Y-Axis (Throttled)", currentDriverControlsMap.getAxisValueWithThrottle(currentDriverControlsMap.STICK_Y_AXIS));
    SmartDashboard.putNumber("NAV-X Yaw", ahrs.getAngle());
    
    //TODO: Fix: Manipulator Controller rumble (Likely not an efficient implementation)
    /*double rumbleVal = ahrs.getWorldLinearAccelX() + ahrs.getWorldLinearAccelY() + ahrs.getWorldLinearAccelZ();
    if (rumbleVal > 1) {
      rumbleVal = 1;
    }*/
    //manipulatorController.setRumble(GenericHID.RumbleType.kRightRumble, rumbleVal);

    int pov = stick.getPOV();
    camAngleMod = 0;
    switch (pov) {
      case 315: // ↖
      case 0:   // ↑
      case 45:  // ↗
        // 
        camAngleMod = -1;
        break;
      case 135: // ↘
      case 180: // ↓
      case 225: // ↙
        camAngleMod = 1;
        break;
    }

    // The servo seems to lock up if it changes direction quickly. This should slow it down a bit.
    if (camAngleMod != 0) {
      cameraAngle += camAngleMod;
      if (cameraAngle > 180.0) cameraAngle = 180.0;
      if (cameraAngle < 0.0) cameraAngle = 0.0;
      double currentAngle = driverCamServo.getAngle();
      if (currentAngle != cameraAngle) {
        driverCamServo.setAngle(currentAngle + camAngleMod);
      }
    }

    //leds.update();
    fireControl.update();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    limelight.setLight(false);

    leds.setDriverStation( 0, 0, 0, false);

    leds.setIntakeLedsSolid(50, 50, 50);
    Timer.delay(0.02);
    leds.setShooterLedsSolid(0, 50, 0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    //SmartDashboard.putNumber("Angle Offset: ", 0);
    LiveWindow.setEnabled(false);
    ahrs.reset();

    SmartDashboard.putNumber("P Gain", fireControl.kP);
    SmartDashboard.putNumber("I Gain", fireControl.kI);
    SmartDashboard.putNumber("D Gain", fireControl.kD);
    SmartDashboard.putNumber("I Zone", fireControl.kIz);
    SmartDashboard.putNumber("Feed Forward", fireControl.kFF);
    SmartDashboard.putNumber("Max Output", fireControl.kMaxOutput);
    SmartDashboard.putNumber("Min Output", fireControl.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putBoolean("Shooter Pid Update", false);

    leds.setIntakeLedsSolid(127, 15, 0);
    leds.setShooterLedsSolid(0, 50, 0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    double offset = SmartDashboard.getNumber("Angle Offset: ", 0);
    //drive.driveAutonomous(0, 0, offset, ahrs);
    fireControl.kP = SmartDashboard.getNumber("P Gain", 0);
    fireControl.kI = SmartDashboard.getNumber("I Gain", 0);
    fireControl.kD = SmartDashboard.getNumber("D Gain", 0);
    fireControl.kIz = SmartDashboard.getNumber("I Zone", 0);
    fireControl.kFF = SmartDashboard.getNumber("Feed Forward", 0);
    fireControl.kMaxOutput = SmartDashboard.getNumber("Max Output", 0);
    fireControl.kMinOutput = SmartDashboard.getNumber("Min Output", 0);
    fireControl.setShooterTargetRPM(SmartDashboard.getNumber("Set Rotations", 0));

    if (currentManipulatorControlsMap.getAxisValue(currentManipulatorControlsMap.FIRE_TRIGGER) > 0.05) {
      fireControl.shooterSpinUp();
    }
    else {
      fireControl.shooterSpinDown();
    }

    fireControl.update();
  }
}
