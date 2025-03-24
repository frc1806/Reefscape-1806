package frc.robot.subsystems.intakes;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public abstract class MotorizedArmIntake extends SubsystemBase{
    
    enum MotorizedIntakeArmState{
        kRetracting,
        kHoldingIn,
        kMovingToIntake,
        kHoldIntakeOut,
        kGoingToArbitraryPosition,
        kAtArbitraryPosition,
        kDisabled
    }

    private MotorizedIntakeArmState mArmState;
    private TalonFX mIntakeRollerMotor;
    private SparkMax mIntakeArmMotor;
    private boolean mSpitOut;

    private SparkMaxConfig mIntakeConfig;
    private TalonFXConfiguration mIntakeRollerConfig;
    private double mDesiredArbitraryPosition;
    private SingleJointedArmSim mArmSim;
    private SparkAbsoluteEncoderSim mEncoderSim;
    private DCMotor mArmMotorSim;
    private SparkMaxSim mArmSparkMaxSim;

    protected MotorizedArmIntake()
    {
        //Idle arm
        mArmState = isIntakeEnabled()? MotorizedIntakeArmState.kHoldingIn:MotorizedIntakeArmState.kDisabled;

        mSpitOut = false;
        mDesiredArbitraryPosition = getIntakeArmAngleAtRest(); //Set arb position to safe default.
        
        //Setup intake arm motor, use abstract function calls to get information specific to the intake type
        mIntakeArmMotor = new SparkMax(getIntakeArmMotorId(), MotorType.kBrushless);

        mIntakeConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig armEncoderConfig = new AbsoluteEncoderConfig();
        //configure through bore encoder. We will Zero them in rev's hardware client.
        armEncoderConfig.positionConversionFactor(360);
        armEncoderConfig.velocityConversionFactor(360.0 / 60.0);
        armEncoderConfig.startPulseUs(1.0);
        armEncoderConfig.endPulseUs(1024.0);
        armEncoderConfig.inverted(isIntakeArmEncoderInverted());
        armEncoderConfig.zeroCentered(true);

        //configure closed loop control of intake arm
        MAXMotionConfig armMoveConfig = new MAXMotionConfig();
        armMoveConfig.allowedClosedLoopError(getMaximumAllowedClosedLoopError());
        armMoveConfig.allowedClosedLoopError(getMaximumAllowedClosedLoopError(), ClosedLoopSlot.kSlot1);
        armMoveConfig.maxAcceleration(getMaxMotionMaxAcceleration());
        armMoveConfig.maxAcceleration(getMaxMotionMaxAcceleration(), ClosedLoopSlot.kSlot1);
        armMoveConfig.maxVelocity(getMaxMotionMaxVelocity());
        armMoveConfig.maxVelocity(getMaxMotionMaxVelocity(), ClosedLoopSlot.kSlot1);    
        armMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        armMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1);
        
        ClosedLoopConfig armClosedLoopConfig = new ClosedLoopConfig();
        armClosedLoopConfig.pid(getMovingPGain(), getMovingIGain(), getMovingDGain(), ClosedLoopSlot.kSlot0);
        armClosedLoopConfig.pid(getHoldingPGain(), getHoldingIGain(), getHoldingDGain(), ClosedLoopSlot.kSlot1);
        armClosedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armClosedLoopConfig.apply(armMoveConfig);

        mIntakeConfig.apply(armEncoderConfig);
        mIntakeConfig.apply(armClosedLoopConfig);
        mIntakeConfig.smartCurrentLimit(getIntakeArmCurrentLimit());
        mIntakeConfig.inverted(isIntakeArmMotorInverted());
        mIntakeConfig.idleMode(IdleMode.kCoast);
        mIntakeArmMotor.configure(mIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //configure intake roller motor
        mIntakeRollerMotor = new TalonFX(getIntakeRollerMotorId());
        TalonFXConfigurator intakeRollerConfigurator = mIntakeRollerMotor.getConfigurator();
        CurrentLimitsConfigs intakeRollerCurrentConfigs = new CurrentLimitsConfigs();
        intakeRollerCurrentConfigs.withSupplyCurrentLimit(getIntakeRollerSupplyCurrentLimit());
        intakeRollerCurrentConfigs.withStatorCurrentLimit(getIntakeRollerStatorCurrentLimit());

        mIntakeRollerConfig = new TalonFXConfiguration();
        mIntakeRollerConfig.withCurrentLimits(intakeRollerCurrentConfigs);
        MotorOutputConfigs rollerOutputConfig = new MotorOutputConfigs();
        rollerOutputConfig.withInverted(isIntakeRollerInverted()? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive);
        rollerOutputConfig.withNeutralMode(NeutralModeValue.Coast);
        mIntakeRollerConfig.withMotorOutput(rollerOutputConfig);
        intakeRollerConfigurator.apply(mIntakeRollerConfig);


        mArmMotorSim = DCMotor.getNEO(1);
        mArmSparkMaxSim = new SparkMaxSim(mIntakeArmMotor, mArmMotorSim);
        mEncoderSim = mArmSparkMaxSim.getAbsoluteEncoderSim();
        mArmSim = new SingleJointedArmSim(mArmMotorSim, getArmGearRatio(), SingleJointedArmSim.estimateMOI(getArmCenterOfGravityDistance(), getArmMass()), getArmCenterOfGravityDistance(), Units.degreesToRadians(getMinimumEverReasonableAngle()), Units.degreesToRadians(getMaximumEverReasonableAngle()),true, 0.0, 0.0, 0.0);
    }

    /**
     * Is this thing on?
     * @return
     */
    protected abstract boolean isIntakeEnabled();

    /**
     * Get the name of this string for use on the dashboard/debugging.
     * @return
     */
    protected abstract String getIntakeName();

    /*
     * HARDWARE
     */

     /**
      * The CAN id of the intake roller motor.
      * @return
      */
    protected abstract int getIntakeRollerMotorId();

    /**
     * Does the intake roller need to be inverted? Positive voltage/duty cycle should bring the game piece in.
     * @return
     */
    protected abstract boolean isIntakeRollerInverted();

    /**
     * Get the stator current limit for the intake roller.
     * @return
     */
    protected abstract double getIntakeRollerStatorCurrentLimit();

    /**
     * Get the supply current limit for the intake roller.
     * @return
     */
    protected abstract double getIntakeRollerSupplyCurrentLimit();


    /**
     * The CAN id of the intake arm motor.
     * @return
     */
    protected abstract int getIntakeArmMotorId();

    /**
     * Does the intake arm motor need to be inverted? Positive voltage/duty cycle should move the arm out.
     * @return
     */
    protected abstract boolean isIntakeArmMotorInverted();

    /**
     * Does the intake arm encoder need to be inverted? It should count up as it move outward.
     * @return
     */
    protected abstract boolean isIntakeArmEncoderInverted();

    /**
     * Get the smart current limit for the intake arm motor.
     * @return
     */
    protected abstract int getIntakeArmCurrentLimit();


    /*
     * PID GAINS FOR MOVING
     */

    /**
     * Get the proportional gain for moving to a setpoint
     * @return
     */
    protected abstract double getMovingPGain();
    
    /**
     * Get the integral gain for moving to a setpoint
     * @return
     */
    protected abstract double getMovingIGain();

    /**
     * Get the derivative gain for moving to a setpoint
     * @return
     */
    protected abstract double getMovingDGain();

    /**
     * Get the max velocity for moving to a setpoint in degrees per second
     *      * @return
     */
    protected abstract double getMaxMotionMaxVelocity();

    /**
     * Get the max acceleration for moving to a setpoint in degrees per second
     * @return
     */
    protected abstract double getMaxMotionMaxAcceleration();

    /**
     * Get the allowable error where the PID will "give up" and shut off
     * @return
     */
    protected abstract double getMaximumAllowedClosedLoopError();

    /*
     * PID GAINS FOR HOLDING OUT
     */

     /**
      * Get the proportional gain for holding the arm out
      * @return
      */
    protected abstract double getHoldingPGain();

    /**
     * Get the integral gain for holding the arm out
     * @return
     */
    protected abstract double getHoldingIGain();

    /**
     * Get the derivative gain for holding the arm out
     * @return
     */
    protected abstract double getHoldingDGain();

    
    /** 
     * ANGLES AND SPEEDS
     */

    /**
     * Get what angle the arm should be comanded to to be at the resting position.
     * @return
     */
    protected abstract double getIntakeArmAngleAtRest();

    /**
     * Get what angle the arm should extend to for intaking.
     * @return
     */

    protected abstract double getIntakeArmAngleAtExtension();


    /**
     * Get the voltage command for the roller motor while intaking.
     * @return
     */
    protected abstract double getIntakeRollerSpeedIntaking();

    /**
     * Get the voltage command for the roller motor while spitting out/clearing a jam.
     * @return
     */
    protected abstract double getIntakeRollerSpeedSpitOut();


    /**
     * Get how far we will allow the arm to be out in degrees before we consider it not in.
     * @return
     */
    protected abstract double getAcceptableHoldInAngleDeviation();

    /**
     * Get how far off the intake arm can be before we force a move back to it's defined intaking position.
     * @return
     */
    protected abstract double getAcceptableHoldingOutAngleDeviation();

    /**
     * Get the angle that under which we would disable the mechanism due to a fault.
     * @return
     */
    protected abstract double getMinimumEverReasonableAngle();


    /**
     * Get the angle that over which we would disable the mechanism due to a fault.
     * @return
     */
    protected abstract double getMaximumEverReasonableAngle();

    protected abstract double getArmGearRatio();
    protected abstract double getArmCenterOfGravityDistance();
    protected abstract double getArmMass();


    /*
     * FUNCTION CALLS TO DO THINGS
     */

    /**
     * Move the intake out and start the rollers inward.
     */
    public void intake(){
        if(!isDisabled())
        {
            mSpitOut = false;
            mArmState = MotorizedIntakeArmState.kMovingToIntake;
        }

    }

    /**
     * Move the intake in and stop the rollers.
     */
    public void retract(){
        if(!isDisabled())
        {
            mArmState = MotorizedIntakeArmState.kRetracting;
        }

    }


    /**
     * Move the intake out and start the rollers outward.
     */
    public void reverse(){
        if(!isDisabled())
        {
            mSpitOut = true;
            mArmState = MotorizedIntakeArmState.kMovingToIntake;
        }
    }

    /**
     * Move the intake to an arbitrary angle, does not spin rollers.
     * @param angle desired angle.
     */
    public void goToArbitraryPosition(double angle)
    {
        mDesiredArbitraryPosition = angle;
        mArmState = MotorizedIntakeArmState.kGoingToArbitraryPosition;
    }

    /**
     * See if we are within the acceptable range of the arbitrary angle given.
     * @param angle desired angle
     * @return true if we are within the holding out angle deviation of the given angle. false otherwise.
     */
    public boolean isAtArbitraryPosition(double angle)
    {
        return Math.abs(mIntakeArmMotor.getAbsoluteEncoder().getPosition() - angle) < getAcceptableHoldingOutAngleDeviation();
    }

    /**
     * FUNCTION CALLS TO CHECK IF A THING IS DONE
     */

     /**
      * Check if the intake is fully out
      * @return true if the intake is currently holding itself out.
      */
     public boolean isIntakeExtended()
     {
        return mArmState == MotorizedIntakeArmState.kHoldIntakeOut;
     }

     /**
      * check if the intake is fully in
      * @return true if the intake is currently holding itself in.
      */
     public boolean isIntakeRetracted()
     {
        return mArmState == MotorizedIntakeArmState.kHoldingIn;
     }

    /**
     * Has this thing been disabled?
     * @return
     */
    public boolean isDisabled(){
        return mArmState == MotorizedIntakeArmState.kDisabled;
    }

    @Override
    public void simulationPeriodic()
    {
        double volts = RobotController.getInputVoltage();

        mArmSim.setInputVoltage(mArmSparkMaxSim.getAppliedOutput() * volts);
        mArmSim.update(Robot.kDefaultPeriod);
        mArmSparkMaxSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        mArmSparkMaxSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        mArmSparkMaxSim.iterate(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()), volts, Robot.kDefaultPeriod);
        //mArmSparkMaxSim.iterate(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()), volts, Robot.kDefaultPeriod);
        //mArmSparkMaxSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        //mEncoderSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        //mEncoderSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        //mEncoderSim.iterate(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()), Robot.kDefaultPeriod);
        SmartDashboard.putNumber(getIntakeName() + "/Simulation/ArmPosition", Units.radiansToDegrees(mArmSim.getAngleRads()));
        SmartDashboard.putNumber(getIntakeName() + "/Simulation/ArmCurrentDraw", mArmSim.getCurrentDrawAmps());
    }

    @Override
    public void periodic()
    {
        double armPosition = mIntakeArmMotor.getAbsoluteEncoder().getPosition();
        //Check for faults
        /*if(mArmState != MotorizedIntakeArmState.kDisabled && (armPosition> getMaximumEverReasonableAngle() || armPosition < getMinimumEverReasonableAngle()))
        {
            mArmState = MotorizedIntakeArmState.kDisabled; //kill it, live to play another match unless it's already FUBAR. At least we might save the motor.
            System.out.println("!!INTAKE FAULT!! Intake:" + getIntakeName() + " has achieved an arm angle of:" + armPosition + " . Disabling.");
        }*/
        

        //Run the intake arm based on the current state
        switch(mArmState)
        {

            case kHoldingIn:
                //hold the arm in with the light PID
                mIntakeRollerMotor.stopMotor();
                mIntakeArmMotor.getClosedLoopController().setReference(getIntakeArmAngleAtRest(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
                if(armPosition - getIntakeArmAngleAtRest() >= getAcceptableHoldInAngleDeviation()){
                    mArmState = MotorizedIntakeArmState.kRetracting; //Switch to the stronger PID
                }
            break;

            case kRetracting:
                //move the arm in with haste.
                mIntakeRollerMotor.stopMotor();
                mIntakeArmMotor.getClosedLoopController().setReference(getIntakeArmAngleAtRest(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
                if(Math.abs(armPosition - getIntakeArmAngleAtRest()) < getMaximumAllowedClosedLoopError()) {
                    mArmState = MotorizedIntakeArmState.kHoldingIn; //Switch to the lighter PID
                }
            break;
            
            case kMovingToIntake:
                mIntakeRollerMotor.setControl(new VoltageOut(mSpitOut?getIntakeRollerSpeedSpitOut():getIntakeRollerSpeedIntaking()));
                mIntakeArmMotor.getClosedLoopController().setReference(getIntakeArmAngleAtExtension(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
                if(Math.abs(armPosition - getIntakeArmAngleAtExtension()) < getMaximumAllowedClosedLoopError()){
                    mArmState = MotorizedIntakeArmState.kHoldIntakeOut;
                }
            break;

            case kHoldIntakeOut:
                mIntakeRollerMotor.setControl(new VoltageOut(mSpitOut?getIntakeRollerSpeedSpitOut():getIntakeRollerSpeedIntaking()));
                mIntakeArmMotor.getClosedLoopController().setReference(getIntakeArmAngleAtExtension(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
                if(Math.abs(armPosition - getIntakeArmAngleAtExtension()) > getAcceptableHoldingOutAngleDeviation()){
                    mArmState = MotorizedIntakeArmState.kMovingToIntake;
                }
            break;

            case kDisabled:
                mIntakeRollerMotor.stopMotor();
                mIntakeArmMotor.stopMotor();
            break;
            case kAtArbitraryPosition:
                mIntakeRollerMotor.stopMotor();
                mIntakeArmMotor.getClosedLoopController().setReference(mDesiredArbitraryPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
                if(Math.abs(armPosition - mDesiredArbitraryPosition) > getAcceptableHoldingOutAngleDeviation()){
                    mArmState = MotorizedIntakeArmState.kGoingToArbitraryPosition;
                }
                break;
            case kGoingToArbitraryPosition:
                mIntakeRollerMotor.stopMotor();
                mIntakeArmMotor.getClosedLoopController().setReference(mDesiredArbitraryPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
                if(Math.abs(armPosition - mDesiredArbitraryPosition) < getMaximumAllowedClosedLoopError()){
                    mArmState = MotorizedIntakeArmState.kAtArbitraryPosition;
                }
                break;
            default:
                break;
        }

        //Update smart dashbaord
        SmartDashboard.putNumber(getIntakeName() + "/arm position", armPosition);
        SmartDashboard.putString(getIntakeName() + "/state", mArmState.name());
        SmartDashboard.putNumber(getIntakeName() + "/armMotorAppliedOutput", mIntakeArmMotor.getAppliedOutput());
        SmartDashboard.putNumber(getIntakeName() + "/rollerMotorOutputVolts", mIntakeRollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(getIntakeName() + "/armVelocity", mIntakeArmMotor.getAbsoluteEncoder().getVelocity());
    }

    public void addToOrchestra(Orchestra orchestra){
        orchestra.addInstrument(mIntakeRollerMotor);
    }

}
