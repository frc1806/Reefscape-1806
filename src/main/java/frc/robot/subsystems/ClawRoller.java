// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TheClawConstants;
import frc.robot.RobotMap;

public class ClawRoller extends SubsystemBase {

  public static ClawRoller S_ROLLER = new ClawRoller();

  public static ClawRoller GetInstance(){
    return S_ROLLER;
  }
  /** Creates a new ClawRoller. */
  private SparkMax mClawRollerMotor;
  private enum HeldGamePiece{
    kNothing,
    kCoral,
    kAlgae
}
private static final SparkBaseConfig S_ROLLERBRAKECONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake);
private static final SparkBaseConfig S_ROLLERCOASTCONFIG = new SparkMaxConfig().idleMode(IdleMode.kCoast);

private HeldGamePiece mCurrentGamePiece;

public ClawRoller() {
    mClawRollerMotor = new SparkMax(RobotMap.ALGAE_CLAW_ROLLER_MOTOR_ID, MotorType.kBrushless);
    SparkBaseConfig rollerMotorConfig = new SparkMaxConfig().smartCurrentLimit(15).inverted(true);
    LimitSwitchConfig rollerLimitSwitchConfig = new LimitSwitchConfig();
    rollerLimitSwitchConfig.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
    rollerLimitSwitchConfig.reverseLimitSwitchType(Type.kNormallyOpen);
    rollerMotorConfig.apply(rollerLimitSwitchConfig);
    mClawRollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mCurrentGamePiece = limitSwitchHit()?HeldGamePiece.kCoral:HeldGamePiece.kNothing;

  }

  
  public boolean hasGamePiece(){
    return limitSwitchHit();
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
        mClawRollerMotor.setVoltage(3.0);
    }

    public void runRollersOut(){
        mClawRollerMotor.setVoltage(TheClawConstants.THE_CLAW_ROLLER_OUT_VOLTAGE);
        clearHeldGamePiece();
    }

  @Override
  public void periodic() {
      SmartDashboard.putString("TheClaw/GamePieceHeld", mCurrentGamePiece.name());
      SmartDashboard.putBoolean("TheClaw/LimitSwitch", limitSwitchHit());
    switch(mCurrentGamePiece){
      case kAlgae:
          mClawRollerMotor.setVoltage(-12.0);
          break;
      case kCoral:
          mClawRollerMotor.set(-.05);
          break;
      default:
      case kNothing:
          break;
  }
  }
}
