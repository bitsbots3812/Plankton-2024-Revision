package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeShooterInterface {
    //TODO: Shooter Motor PID Control
    RobotState state;
    private MotorController intakeDeploy;
    //private double intakeDeployNextIteration = 0;
    private MotorController intakeRotation;
    private double intakeRotationNextIteration = 0;
    private VictorSPX indexer;
    private double indexerNextIteration = 0;
    private CANSparkMax shooter;
    private SparkMaxPIDController shooterPID;
    private boolean shooterSpin = false;
    private boolean shooterEnabled = true;
    private double shooterTargetRPM = 5700;
    private int shooterRPMIterationsInRange = 0;

    double kP = 0.0005; 
    double kI = 0.00000002;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000185; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double maxRPM = 5700;
    boolean updatePid = false;

    private MotorController hoodMotor;
    private AnalogPotentiometer hoodPot;
    private double hoodPotTargetValue;
    //TODO: Add to constuctor
    private double hoodPotMin = 0.16;
    private double hoodPotMax = 0.7;
    
    private ColorSensorInterface colorSensor;

    private boolean indexed = false;

    final boolean INTAKE_DEPLOYED_STATE = true;
    final boolean INTAKE_RETRACTED_STATE = false;

    Timer intakeTimer = new Timer();

    private boolean intakeDeployed = false;
    private boolean intakeRetracted = false;
    private boolean intakeTargetState = false; //false retracted; true deployed


    void deployIntake() {
        intakeTargetState = INTAKE_DEPLOYED_STATE;
        intakeTimer.reset();
        intakeTimer.start();
        intakeRetracted = false;
    }

    void retractIntake() {
        intakeTargetState = INTAKE_RETRACTED_STATE;
        intakeTimer.reset();
        intakeTimer.start();
        intakeDeployed = false;
    }
    
    void shooterSetEnalbed(boolean mode) {
        shooterEnabled = mode;
    }

    boolean shooterGetEnabled() {
        return shooterEnabled;
    }

    void shooterSpinUp() {
        shooterSpin = true;
    }

    void shooterSpinDown() {
        shooterSpin = false;
    }
    
    void setShooterTargetRPM(double rpm) {
        shooterTargetRPM = rpm;
    }

    //runs intake motors, unless ball is indexed, or override is true
    void intake(boolean override) {
        if (!indexed || override) {
            intakeRotationNextIteration = 1;
            indexerNextIteration = 1;
        } 
        else {
            shooterSpinUp();
            intakeRotationNextIteration = 0;
            indexerNextIteration = 0;
        }
    }

    void reject() {
        intakeRotationNextIteration = -1;
        indexerNextIteration = -1;
    }

    //TODO: use current to detect when ball fires.
    //Use neo encoder to only index when shooter is at desired rpm
    void fire() {
        shooterSpinUp();
        if (shooterRPMIterationsInRange >= 15) {
            indexerNextIteration = 1;
        }
    }

    boolean isIndexed() {
        return indexed;
    }

    void hoodToPotValue(double val) {
        hoodPotTargetValue = val;
    }

    void hoodUp(double amount) {
        if ((hoodPotTargetValue + amount) > hoodPotMax) {
            hoodPotTargetValue = hoodPotMax;
        }
        else {
            hoodPotTargetValue += amount;
        }
    }
    
    void hoodDown(double amount) {
        if ((hoodPotTargetValue - amount) < hoodPotMin) {
            hoodPotTargetValue = hoodPotMin;
        }
        else {
            hoodPotTargetValue -= amount;
        }
    }
    
    //TODO: detection of incorrect color
    void update() {
        indexed = colorSensor.isLoaded();

        if (intakeTargetState == INTAKE_DEPLOYED_STATE) {
            if (intakeTimer.hasElapsed(2)) { //if the intake has been disabled due to currentlimiting the limit has been hit or if 3 seconds has elapsed since command was sent
                intakeDeployed = true; //flag intake as deployed
                intakeDeploy.set(0); //set motors to zero
            }
            else if (!intakeDeployed){ //if intake is not deployed (target state is deployed)
                intakeDeploy.set(-0.3); //run motor for deployment
            }
            else {
                intakeDeploy.set(0); //intake is already deployed. Do nothing.
            }
        }
        else { //intake target state is now retracted. Otherwise same as above
            if (intakeTimer.hasElapsed(2)) {
                intakeRetracted = true;
                intakeDeploy.set(0);
            }
            else if (!intakeRetracted) {
                if (intakeTimer.get() < 1.2) {
                    intakeDeploy.set(0.9);
                }
                else {
                    intakeDeploy.set(0.1);
                }
            }
            else {
                intakeDeploy.set(0);
            }
        }
        
        //intakeDeploy.set(intakeDeployNextIteration);
        //intakeDeployNextIteration = 0; //Reset to zero if changed. Intake will only move when button is held
        if (intakeDeployed || intakeTargetState == INTAKE_DEPLOYED_STATE) {
            intakeRotation.set(intakeRotationNextIteration);
        } 
        else {
            intakeRotation.set(0);
        }
        intakeRotationNextIteration = 0;
        indexer.set(ControlMode.PercentOutput, indexerNextIteration);
        indexerNextIteration = 0;

        double currentHoodPot = hoodPot.get();
        if (hoodPotTargetValue - currentHoodPot <= -0.01 && !(currentHoodPot <= hoodPotMin)) {
            hoodMotor.set(1*(currentHoodPot/hoodPotTargetValue));
        }
        else if (hoodPotTargetValue - currentHoodPot >= 0.01 && !(currentHoodPot >= hoodPotMax)) {
            hoodMotor.set(-1*(hoodPotTargetValue/currentHoodPot));
        }
        else {
            hoodMotor.set(0);
        }

        if (SmartDashboard.getBoolean("Shooter Pid Update", false)) {
            shooterPID.setP(kP);
            shooterPID.setI(kI);
            shooterPID.setD(kD);
            shooterPID.setIZone(kIz);
            shooterPID.setFF(kFF);
            SmartDashboard.putBoolean("Shooter Pid Update", false);
        }

        double currentShooterRpm = shooter.getEncoder().getVelocity();
        if (shooterSpin && shooterEnabled) {
            shooterPID.setReference(shooterTargetRPM, ControlType.kVelocity);
        }
        else {
            shooterPID.setReference(0, ControlType.kDutyCycle);
        }

        if ((shooter.getEncoder().getVelocity() > (shooterTargetRPM-70)) && (shooter.getEncoder().getVelocity() < (shooterTargetRPM+70))) {
            shooterRPMIterationsInRange ++;
        }
        else {
            shooterRPMIterationsInRange = 0;
        }

        SmartDashboard.putBoolean("Shooter State:", shooterSpin);
        SmartDashboard.putNumber("Shooter Motor RPM:", currentShooterRpm);
        SmartDashboard.putNumber("Shooter Motor Target RPM (Set):", shooterTargetRPM);
        SmartDashboard.putNumber("Hood Target Value: ", hoodPotTargetValue);
        SmartDashboard.putNumber("Hood Potentiometer Value: ", hoodPot.get());
        //SmartDashboard.putNumber("Shooter Power: ", shooterPower);
    }

    IntakeShooterInterface (MotorController intakeDeployController, MotorController intakeRotationController, VictorSPX indexerController, MotorController hoodController, AnalogPotentiometer shooterPotentiometer, CANSparkMax shooterController, boolean shooterInverted, ColorSensorInterface intakeColorSensor, RobotState State) {
        intakeDeploy = intakeDeployController;
        intakeRotation = intakeRotationController;
        indexer = indexerController;
        hoodMotor = hoodController;
        shooter = shooterController;
        shooterPID = shooterController.getPIDController();
        hoodPot = shooterPotentiometer;
        colorSensor = intakeColorSensor;
        state = State;

        indexerController.setNeutralMode(NeutralMode.Brake);

        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIz);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);

        shooter.setInverted(shooterInverted);

        indexed = colorSensor.isLoaded();
    }
}

