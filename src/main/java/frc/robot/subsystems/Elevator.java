package frc.robot.subsystems;

import java.util.Arrays;

import org.dyn4j.geometry.Matrix22;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase{
    private static final boolean IS_ELEVATOR_ENABLED = true;
    private TalonFX mElevator1, mElevator2, mElevator3;
    private double mWantedHeight;
    private ElevatorSim mElevatorSim;
    private Servo mBrakeServo;


    private static final Elevator INSTANCE = new Elevator();

    public static Elevator GetInstance(){
        return INSTANCE;
    }

    private Elevator()
    {
        mWantedHeight = ElevatorConstants.ELEVATOR_START_HEIGHT;
        mElevator1 = new TalonFX(RobotMap.ELEVATOR_MOTOR_1);
        mElevator2 = new TalonFX(RobotMap.ELEVATOR_MOTOR_2);
        mElevator3 = new TalonFX(RobotMap.ELEVATOR_MOTOR_3);

        mElevator1.setNeutralMode(NeutralModeValue.Brake);
        mElevator2.setNeutralMode(NeutralModeValue.Brake);
        mElevator3.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator motor1Configurator = mElevator1.getConfigurator();
        TalonFXConfigurator motor2Configurator = mElevator2.getConfigurator();
        TalonFXConfigurator motor3Configurator = mElevator3.getConfigurator();

        var motorCurrent = new CurrentLimitsConfigs();
        motorCurrent.StatorCurrentLimit = 180;
        motorCurrent.SupplyCurrentLimit = 90;
        motor1Configurator.apply(motorCurrent);
        motor2Configurator.apply(motorCurrent);
        motor3Configurator.apply(motorCurrent);

        var Slot0Configs = new Slot0Configs();
        Slot0Configs.kP = 2.5;
        Slot0Configs.kI = 0;
        Slot0Configs.kD = 0;
        //Slot0Configs.kG = .057;
        //Slot0Configs.kS = 0;
        //Slot0Configs.kV = .802;
        //Slot0Configs.kA = 0.009;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 500.0;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50.0;



        motor1Configurator.apply(Slot0Configs);
        motor1Configurator.apply(motionMagicConfigs);

        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        feedbackConfigs.withSensorToMechanismRatio(ElevatorConstants.ELEVATOR_GEAR_RATIO / (ElevatorConstants.ELEVATOR_CASCADE_STAGES * ElevatorConstants.ELEVATOR_DRUM_DIAMETER * Math.PI));
        
        motor1Configurator.apply(feedbackConfigs);
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.withPeakForwardDutyCycle(1.0);
        outputConfigs.withPeakReverseDutyCycle(-1.0);
        outputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        motor1Configurator.apply(outputConfigs);
        
        SoftwareLimitSwitchConfigs softLimitConfigs  = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.ForwardSoftLimitEnable = true;
        softLimitConfigs.ForwardSoftLimitThreshold = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        softLimitConfigs.ReverseSoftLimitEnable = true;
        softLimitConfigs.ReverseSoftLimitThreshold = ElevatorConstants.ELEVATOR_MIN_HEIGHT;

        motor1Configurator.apply(softLimitConfigs);

        final DutyCycleOut stopRequest = new DutyCycleOut(0);
        mElevator2.setControl(new Follower(mElevator1.getDeviceID(), false));
        mElevator3.setControl(new Follower(mElevator1.getDeviceID(), true));
        


        mElevator1.setControl(stopRequest);
        mElevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), ElevatorConstants.ELEVATOR_GEAR_RATIO, Units.lbsToKilograms(20), Units.inchesToMeters(ElevatorConstants.ELEVATOR_DRUM_DIAMETER/2.0), Units.inchesToMeters(ElevatorConstants.ELEVATOR_MIN_HEIGHT), Units.inchesToMeters(ElevatorConstants.ELEVATOR_MAX_HEIGHT), true, Units.inchesToMeters(ElevatorConstants.ELEVATOR_START_HEIGHT));
        mElevator1.setPosition(ElevatorConstants.ELEVATOR_START_HEIGHT);


        mBrakeServo = new Servo(9);
        disengageParkingBrake();
     }

    
    
    /** 
     * Request the elevator to go to a height
     * @param inches distance of the carriage, in inches from the bottom of the elevator.
     */
    public void GoToPosition(double inches) {
        if(IS_ELEVATOR_ENABLED)
        {
            mWantedHeight = inches;
            mElevator1.setControl(new MotionMagicVoltage(inches));
        }
    }

    
    /** 
     * return the carrage height in inches
     * @return double
     */
    public double GetPosition() {
        return mElevator1.getPosition().getValueAsDouble();
    }

    
    /** 
     * @return boolean
     */
    public boolean isAtPosition(){
        return Math.abs(mWantedHeight - GetPosition()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE && Math.abs(getVelocity()) < ElevatorConstants.ELEVATOR_SPEED_TOLERANCE;
    }

    public boolean isAtArbitraryPosition(double position)
    {
        return Math.abs(position - GetPosition()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE && Math.abs(getVelocity()) < ElevatorConstants.ELEVATOR_SPEED_TOLERANCE;
    }

    /** 
     * Stops the elevator
     */
    public void stopElevator(){
        mElevator1.stopMotor();
}

    public void engageParkingBrake(){
        mBrakeServo.setAngle(ElevatorConstants.ELEVATOR_PARK_SERVO_BRAKE_ANGLE);
    }

    public void disengageParkingBrake(){
        mBrakeServo.setAngle(ElevatorConstants.ELEVATOR_PARK_SERVO_DISENGAGE_ANGLE);
    }

    public boolean isBrakeEngaged(){
        return Math.abs(mBrakeServo.getAngle() - ElevatorConstants.ELEVATOR_PARK_SERVO_BRAKE_ANGLE) < ElevatorConstants.ELEVATOR_PARK_SERVO_ANGLE_TOLERANCE;
    }

    public boolean isBrakeDisengaged(){
        return Math.abs(mBrakeServo.getAngle() - ElevatorConstants.ELEVATOR_PARK_SERVO_DISENGAGE_ANGLE) < ElevatorConstants.ELEVATOR_PARK_SERVO_ANGLE_TOLERANCE;
    }



    
    /** 
     * @return double
     */
    public double getVelocity(){
        
        return mElevator1.getVelocity().getValueAsDouble();
    }
    
    public boolean isAboveHeight(double height){
        return GetPosition() > height;
    }

    @Override
    public void simulationPeriodic()
    {
        mElevator1.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        mElevatorSim.setInputVoltage(mElevator1.getSimState().getMotorVoltage());
        if(!isBrakeEngaged())
        {
            mElevatorSim.update(Robot.kDefaultPeriod);
        }
        mElevator1.getSimState().setRotorVelocity(Units.metersToInches(mElevatorSim.getVelocityMetersPerSecond()) * (ElevatorConstants.ELEVATOR_GEAR_RATIO / (ElevatorConstants.ELEVATOR_DRUM_DIAMETER * Math.PI)));
        mElevator1.getSimState().setRawRotorPosition((Units.metersToInches(mElevatorSim.getPositionMeters()) -(ElevatorConstants.ELEVATOR_START_HEIGHT / ElevatorConstants.ELEVATOR_CASCADE_STAGES))* (ElevatorConstants.ELEVATOR_GEAR_RATIO / (ElevatorConstants.ELEVATOR_DRUM_DIAMETER * Math.PI)));
        //mElevator1.setPosition(Units.metersToInches(mElevatorSim.getPositionMeters() * 3.0)); //Convert to cascade by multiplying by 3
        SmartDashboard.putNumber("Elevator/Simulation/SimElevatorHeight", Units.metersToInches(mElevatorSim.getPositionMeters()));
        SmartDashboard.putNumber("Elevator/Simulation/CurrentDraw", mElevatorSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Elevator/Simulation/ElevatorVelocity", Units.metersToInches(mElevatorSim.getVelocityMetersPerSecond()));

    }

    public void addToOrchestra(Orchestra orchestra)
    {
        orchestra.addInstrument(mElevator1);
        orchestra.addInstrument(mElevator2);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator/CurrentHeight", GetPosition());
        SmartDashboard.putNumber("Elevator/Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator/DesiredHeight", mWantedHeight);
        SmartDashboard.putNumber("Elevator/MotorOutVolts", mElevator1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/MotorOutDutyCycle", mElevator1.getDutyCycle().getValueAsDouble());
        SmartDashboard.putBoolean("Elevator/isAtPosition", isAtPosition());
        SmartDashboard.putBoolean("Elevator/IsBrakeEngaged", isBrakeEngaged());
        SmartDashboard.putBoolean("Elevator/IsBrakeDisengaged", isBrakeDisengaged());
    }

	public void GetAngle() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'GetAngle'");
	}

    public void runManually(double value){
        mElevator1.set(value);
    }

}
