package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TheClawConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class TheClaw extends SubsystemBase{
    private enum HeldGamePiece{
        kNothing,
        kCoral,
        kAlgae
    }

    private HeldGamePiece mCurrentGamePiece;

    private static final TheClaw S_INSTANCE = new TheClaw();
    private static final SparkBaseConfig S_ROLLERBRAKECONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake);
    private static final SparkBaseConfig S_ROLLERCOASTCONFIG = new SparkMaxConfig().idleMode(IdleMode.kCoast);

    public static TheClaw GetInstance(){
        return S_INSTANCE;
    }


    private SparkMax mClawRollerMotor;
    private SparkFlex mClawAngleMotor;
    private SparkFlexSim mClawAngleMotorSim;
    private SingleJointedArmSim mArmSim;
    private SparkAbsoluteEncoderSim mEncoderSim;
    private double mTargetAngle;
    DCMotor mAngleMotorSim;
    private TheClaw() {
        mClawAngleMotor = new SparkFlex(RobotMap.THE_CLAW_ANGLE_MOTOR_ID, MotorType.kBrushless);
        SparkBaseConfig clawAngleConfig = new SparkFlexConfig();

        EncoderConfig clawAngleInternalConfig = new EncoderConfig();
        clawAngleInternalConfig.positionConversionFactor(360.0 / TheClawConstants.ARM_GEAR_RATIO);
        clawAngleInternalConfig.velocityConversionFactor((360.0 / 60.0) / TheClawConstants.ARM_GEAR_RATIO);

        AbsoluteEncoderConfig clawEncoderConfig = new AbsoluteEncoderConfig();
        //configure through bore encoder. We will Zero them in rev's hardware client.
        clawEncoderConfig.positionConversionFactor(360);
        clawEncoderConfig.velocityConversionFactor(360.0/60.0);
        clawEncoderConfig.startPulseUs(1.0);
        clawEncoderConfig.endPulseUs(1024.0);
        clawEncoderConfig.inverted(false);
        clawEncoderConfig.zeroCentered(true);
        mTargetAngle = 0;

        //configure closed loop control of intake arm
        MAXMotionConfig clawMoveConfig = new MAXMotionConfig();
        clawMoveConfig.allowedClosedLoopError(TheClawConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR);
        clawMoveConfig.maxAcceleration(TheClawConstants.MAX_MOTION_MAX_ACCELERATION);
        clawMoveConfig.maxVelocity(TheClawConstants.MAX_MOTION_MAX_VELOCITY);
        clawMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        ClosedLoopConfig armClosedLoopConfig = new ClosedLoopConfig();
        armClosedLoopConfig.pid(TheClawConstants.MOVING_P_GAIN, TheClawConstants.MOVING_I_GAIN, TheClawConstants.MOVING_D_GAIN, ClosedLoopSlot.kSlot0);
        armClosedLoopConfig.apply(clawMoveConfig);

        clawAngleConfig.apply(clawAngleInternalConfig);
        clawAngleConfig.apply(clawEncoderConfig);
        clawAngleConfig.apply(armClosedLoopConfig);
        clawAngleConfig.smartCurrentLimit(TheClawConstants.CLAW_ROTATION_CURRENT_LIMIT);
        clawAngleConfig.inverted(TheClawConstants.CLAW_INTAKE_ARM_INVERTED);
        clawAngleConfig.idleMode(IdleMode.kBrake);
        mClawAngleMotor.configure(clawAngleConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mClawRollerMotor = new SparkMax(RobotMap.ALGAE_CLAW_ROLLER_MOTOR_ID, MotorType.kBrushless);
        SparkBaseConfig rollerMotorConfig = new SparkMaxConfig().smartCurrentLimit(15);
        LimitSwitchConfig rollerLimitSwitchConfig = new LimitSwitchConfig();
        rollerLimitSwitchConfig.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
        rollerMotorConfig.apply(rollerLimitSwitchConfig);
        mClawRollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mAngleMotorSim = DCMotor.getNeoVortex(1);
        mClawAngleMotorSim = new SparkFlexSim(mClawAngleMotor, mAngleMotorSim);
        mEncoderSim = mClawAngleMotorSim.getAbsoluteEncoderSim();
        mEncoderSim.setPositionConversionFactor(360.0);
        mEncoderSim.setVelocityConversionFactor(360.0);

        mArmSim = 
        new SingleJointedArmSim(mAngleMotorSim, TheClawConstants.ARM_GEAR_RATIO, SingleJointedArmSim.estimateMOI(TheClawConstants.ARM_CENTER_OF_MASS_DISTANCE, TheClawConstants.ARM_MASS), TheClawConstants.ARM_CENTER_OF_MASS_DISTANCE, 0, Units.degreesToRadians(270), true, 0.0, 0.0, 0.0);

        mCurrentGamePiece = limitSwitchHit()?HeldGamePiece.kCoral:HeldGamePiece.kNothing;
    }


    public boolean hasGamePiece(){
        return limitSwitchHit();
    }

    @Override
    public void simulationPeriodic(){
        mClawAngleMotorSim.iterate((Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()) * TheClawConstants.ARM_GEAR_RATIO), RobotController.getBatteryVoltage(), Robot.kDefaultPeriod);
        mEncoderSim.iterate(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()), Robot.kDefaultPeriod);
        mArmSim.setInput(mClawAngleMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        mArmSim.update(Robot.kDefaultPeriod);
        // mClawAngleMotorSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        // mClawAngleMotorSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        // mEncoderSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        // mEncoderSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        SmartDashboard.putNumber("TheClaw/Simulation/SimAngle", Units.radiansToDegrees(mArmSim.getAngleRads()));
        SmartDashboard.putString("TheClaw/GamePieceHeld", mCurrentGamePiece.name());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TheClaw/Angle", getAngle());
        SmartDashboard.putNumber("TheClaw/TargetAngle", mTargetAngle);
        SmartDashboard.putNumber("TheClaw/AnglePower", mClawAngleMotor.getAppliedOutput());
        SmartDashboard.putNumber("TheClaw/AngleCurrentDraw", mClawAngleMotor.getOutputCurrent());
        switch(mCurrentGamePiece){
            case kAlgae:
                mClawRollerMotor.setVoltage(-12.0);
                break;
            case kCoral:
                mClawRollerMotor.set(-.1);
                break;
            default:
            case kNothing:
                break;
        }
    }
    public void goToPosition(double angle){
        mClawAngleMotor.getClosedLoopController().setReference(angle, ControlType.kMAXMotionPositionControl);
        mTargetAngle = angle;


    }
    public boolean isAtPosition(){
           return Math.abs(getAngle() - mTargetAngle) < TheClawConstants.ANGLE_TOLERANCE;
    }

    public boolean isAtArbitraryPosition(double angle){
        return Math.abs(getAngle() - angle) < TheClawConstants.ANGLE_TOLERANCE;
    }

    public double getAngle(){
        return mClawAngleMotor.getAbsoluteEncoder().getPosition();
    }
        
    public void runRollersIn(){
        mClawRollerMotor.setVoltage(-TheClawConstants.THE_CLAW_ROLLER_IN_VOLTAGE);
        mClawRollerMotor.configureAsync(S_ROLLERBRAKECONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void stopRollers(){
        mClawRollerMotor.stopMotor();
    }

    private boolean limitSwitchHit(){
        return mClawRollerMotor.getReverseLimitSwitch().isPressed();
    }

    public void holdCoral(){
        mCurrentGamePiece = HeldGamePiece.kCoral;
    }

    public void holdAlgae(){
        mCurrentGamePiece = HeldGamePiece.kAlgae;
    }

    public void clearHeldGamePiece(){
        mCurrentGamePiece = HeldGamePiece.kNothing;
        mClawRollerMotor.set(0);
    }

    public void scoreCoral(){
        mClawRollerMotor.configureAsync(S_ROLLERCOASTCONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        mCurrentGamePiece = HeldGamePiece.kNothing;
        mClawAngleMotor.setVoltage(3.0);
    }

    public void runRollersOut(){
        mClawRollerMotor.setVoltage(TheClawConstants.THE_CLAW_ROLLER_OUT_VOLTAGE);
        clearHeldGamePiece();
    }

    public void manualArm(double value){
        mClawAngleMotor.set(value);
    }
}
