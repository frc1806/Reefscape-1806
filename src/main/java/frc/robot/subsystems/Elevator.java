package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotMap;

public class Elevator {
    private TalonFX mElevator1, mElevator2;

    private static final Elevator INSTANCE = new Elevator();

    public static Elevator GetInstance(){
        return INSTANCE;
    }

    private Elevator()
    {
        mElevator1 = new TalonFX(RobotMap.ELEVATOR_MOTOR_1);
        mElevator2 = new TalonFX(RobotMap.ELEVATOR_MOTOR_2);

        mElevator1.setNeutralMode(NeutralModeValue.Brake);
        mElevator2.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator motor1Configurator = mElevator1.getConfigurator();
        TalonFXConfigurator motor2Configurator = mElevator2.getConfigurator();

        var motorCurrent = new CurrentLimitsConfigs();
        motorCurrent.StatorCurrentLimit = 120;
        motorCurrent.SupplyCurrentLimit = 50;
        motor1Configurator.apply(motorCurrent);
        motor2Configurator.apply(motorCurrent);

        var Slot0Configs = new Slot0Configs();
        Slot0Configs.kP = 0;
        Slot0Configs.kI = 0;
        Slot0Configs.kD = 0;
        Slot0Configs.kG = 0;
        Slot0Configs.kS = 0;
        Slot0Configs.kV = 0;
        Slot0Configs.kA = 0;


        motor1Configurator.apply(Slot0Configs);

        final DutyCycleOut stopRequest = new DutyCycleOut(0);

        
        mElevator2.setControl(new StrictFollower(mElevator1.getDeviceID()));


        mElevator1.setControl(stopRequest);

    }

    
    
    /** 
     * Request the elevator to go to a height
     * @param inches distance of the carriage, in inches from the bottom of the elevator.
     */
    public void GoToPosition(double inches) {
        //TODO: implement
    }

    
    /** 
     * @return double
     */
    public double GetPosition() {
        //TODO: implement
        return 0.0;
    }

    
    /** 
     * @return boolean
     */
    public boolean isAtPosition(){
        //TODO: implement
        return false;
    }

    /** 
     * Stops the elevator
     */
    public void stopElevator(){
        //TODO: implement
}

    
    /** 
     * @return double
     */
    public double getVelocity(){
        //TODO: implement
        return 0.0;
    }
    
    public boolean isAboveHeight(double height){
        //TODO: implement
        return false;
    }
}
