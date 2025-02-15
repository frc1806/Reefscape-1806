package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
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
import frc.robot.Constants.AlgaeClawConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class AlgaeClaw extends SubsystemBase{

    private static final AlgaeClaw S_INSTANCE = new AlgaeClaw();

    public static AlgaeClaw GetInstance(){
        return S_INSTANCE;
    }


    private TalonFX mClawRollerMotor;
    private SparkFlex mClawAngleMotor;
    private SingleJointedArmSim mArmSim;
    private SparkAbsoluteEncoderSim mEncoderSim;

    private AlgaeClaw() {
        mClawAngleMotor = new SparkFlex(RobotMap.ALGAE_CLAW_ANGLE_MOTOR_ID, MotorType.kBrushless);
        SparkBaseConfig clawAngleConfig = new SparkFlexConfig();
        AbsoluteEncoderConfig clawEncoderConfig = new AbsoluteEncoderConfig();
        //configure through bore encoder. We will Zero them in rev's hardware client.
        clawEncoderConfig.positionConversionFactor(360);
        clawEncoderConfig.startPulseUs(1.0);
        clawEncoderConfig.endPulseUs(1024.0);
        clawEncoderConfig.inverted(false);

        //configure closed loop control of intake arm
        MAXMotionConfig clawMoveConfig = new MAXMotionConfig();
        clawMoveConfig.allowedClosedLoopError(AlgaeClawConstants.MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR);
        clawMoveConfig.maxAcceleration(AlgaeClawConstants.MAX_MOTION_MAX_ACCELERATION);
        clawMoveConfig.maxVelocity(AlgaeClawConstants.MAX_MOTION_MAX_VELOCITY);
        clawMoveConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        ClosedLoopConfig armClosedLoopConfig = new ClosedLoopConfig();
        armClosedLoopConfig.pid(AlgaeClawConstants.MOVING_P_GAIN, AlgaeClawConstants.MOVING_I_GAIN, AlgaeClawConstants.MOVING_D_GAIN, ClosedLoopSlot.kSlot0);
        armClosedLoopConfig.apply(clawMoveConfig);

        clawAngleConfig.apply(clawEncoderConfig);
        clawAngleConfig.apply(armClosedLoopConfig);
        clawAngleConfig.smartCurrentLimit(AlgaeClawConstants.CLAW_ROTATION_CURRENT_LIMIT);
        clawAngleConfig.inverted(AlgaeClawConstants.CLAW_INTAKE_ARM_INVERTED);
        clawAngleConfig.idleMode(IdleMode.kBrake);
        mClawAngleMotor.configure(clawAngleConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mClawRollerMotor = new TalonFX(RobotMap.ALGAE_CLAW_ROLLER_MOTOR_ID);
        TalonFXConfigurator intakeRollerConfigurator = mClawRollerMotor.getConfigurator();
        CurrentLimitsConfigs intakeRollerCurrentConfigs = new CurrentLimitsConfigs();
        intakeRollerCurrentConfigs.withSupplyCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT);
        intakeRollerCurrentConfigs.withStatorCurrentLimit(AlgaeClawConstants.INTAKE_ROLLER_STATOR_CURRENT_LIMIT);

        TalonFXConfiguration mIntakeRollerConfig = new TalonFXConfiguration();
        mIntakeRollerConfig.withCurrentLimits(intakeRollerCurrentConfigs);
        MotorOutputConfigs rollerOutputConfig = new MotorOutputConfigs();
        rollerOutputConfig.withInverted(AlgaeClawConstants.IS_INTAKE_ROLLER_INVERTED? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive);
        rollerOutputConfig.withNeutralMode(NeutralModeValue.Coast);
        mIntakeRollerConfig.withMotorOutput(rollerOutputConfig);
        intakeRollerConfigurator.apply(mIntakeRollerConfig);

        mEncoderSim = new SparkAbsoluteEncoderSim(mClawAngleMotor);

        mArmSim = 
        new SingleJointedArmSim(DCMotor.getNeoVortex(1), AlgaeClawConstants.ARM_GEAR_RATIO, SingleJointedArmSim.estimateMOI(AlgaeClawConstants.ARM_CENTER_OF_MASS_DISTANCE, AlgaeClawConstants.ARM_MASS), AlgaeClawConstants.ARM_CENTER_OF_MASS_DISTANCE, 0, Units.degreesToRadians(360), true, 0.0, 0.0);
    }

    public double getAngle(){
        return mClawAngleMotor.getAbsoluteEncoder().getPosition();
    }

    public boolean hasGamePiece(){
        return RobotContainer.S_CARRIAGE_CANDI.getS2Closed().getValue();
    }

    @Override
    public void simulationPeriodic(){
        mArmSim.setInput(mClawAngleMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        mArmSim.update(Robot.kDefaultPeriod);
        mEncoderSim.setPosition(Units.radiansToDegrees(mArmSim.getAngleRads()));
        mEncoderSim.setVelocity(Units.radiansToDegrees(mArmSim.getVelocityRadPerSec()));
        SmartDashboard.putNumber("AlgaeClaw/Simulation/SimAngle", getAngle());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("AlgaeClaw/Angle", getAngle());
    }
}
