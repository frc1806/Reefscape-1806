package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
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
import frc.robot.RobotMap;

public class ClawAngler extends SubsystemBase{




    private static final ClawAngler S_INSTANCE = new ClawAngler();

    public static ClawAngler GetInstance(){
        return S_INSTANCE;
    }


    
    private SparkFlex mClawAngleMotor;
    private SparkFlexSim mClawAngleMotorSim;
    private SingleJointedArmSim mArmSim;
    private SparkAbsoluteEncoderSim mEncoderSim;
    private double mTargetAngle;
    DCMotor mAngleMotorSim;
    private ClawAngler() {
        mClawAngleMotor = new SparkFlex(RobotMap.THE_CLAW_ANGLE_MOTOR_ID, MotorType.kBrushless);
        
        SparkBaseConfig clawAngleConfig = new SparkFlexConfig();

        EncoderConfig clawAngleInternalConfig = new EncoderConfig();
        clawAngleInternalConfig.positionConversionFactor(360.0 / TheClawConstants.ARM_GEAR_RATIO);
        clawAngleInternalConfig.velocityConversionFactor((360.0 / 60.0) / TheClawConstants.ARM_GEAR_RATIO);

        AbsoluteEncoderConfig clawEncoderConfig = new AbsoluteEncoderConfig();
        //configure through bore encoder. We will Zero them in rev's hardware client.
        clawEncoderConfig.positionConversionFactor(360);
        clawEncoderConfig.velocityConversionFactor(360.0/60.0);
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

        if(Robot.isSimulation())
        {
            mAngleMotorSim = DCMotor.getNeoVortex(1);
            mClawAngleMotorSim = new SparkFlexSim(mClawAngleMotor, mAngleMotorSim);
            mEncoderSim = mClawAngleMotorSim.getAbsoluteEncoderSim();
            mEncoderSim.setPositionConversionFactor(360.0);
            mEncoderSim.setVelocityConversionFactor(360.0);
    
            mArmSim = 
            new SingleJointedArmSim(mAngleMotorSim, TheClawConstants.ARM_GEAR_RATIO, SingleJointedArmSim.estimateMOI(TheClawConstants.ARM_CENTER_OF_MASS_DISTANCE, TheClawConstants.ARM_MASS), TheClawConstants.ARM_CENTER_OF_MASS_DISTANCE, 0, Units.degreesToRadians(270), false, 0.0, 0.0, 0.0);
        }
    }

    @Override
    public void simulationPeriodic(){
        mClawAngleMotorSim.iterate((Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()) * TheClawConstants.ARM_GEAR_RATIO * 55.0), RobotController.getBatteryVoltage(), Robot.kDefaultPeriod);
        mEncoderSim.iterate(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()) * 55.0, Robot.kDefaultPeriod);
        mArmSim.setInput(mClawAngleMotor.getAppliedOutput() * 12.5);
        mArmSim.update(Robot.kDefaultPeriod);
        // mClawAngleMotorSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        // mClawAngleMotorSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        // mEncoderSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        // mEncoderSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        SmartDashboard.putNumber("TheClaw/Simulation/SimAngle", Units.radiansToDegrees(mArmSim.getAngleRads()));
      
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TheClaw/Angle", getAngle());
        SmartDashboard.putNumber("TheClaw/TargetAngle", mTargetAngle);
        SmartDashboard.putNumber("TheClaw/AnglePower", mClawAngleMotor.getAppliedOutput());
        SmartDashboard.putNumber("TheClaw/AngleCurrentDraw", mClawAngleMotor.getOutputCurrent());
        
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
        


    public void manualArm(double value){
        mClawAngleMotor.set(value);
    }
}
