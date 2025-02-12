package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase{
    private TalonFX mElevator1, mElevator2;
    private ElevatorSim mElevatorSim;

    private double mDesiredPosition;

    private static final Elevator INSTANCE = new Elevator();

    public static Elevator GetInstance(){
        return INSTANCE;
    }

    private Elevator()
    {
        mDesiredPosition = 6.75;
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
        Slot0Configs.kP = 1.0 / 20.0;
        Slot0Configs.kI = 0;
        Slot0Configs.kD = 0;
        Slot0Configs.kG = .57;
        Slot0Configs.kS = 0;
        Slot0Configs.kV = 8.02;
        Slot0Configs.kA = 0.09;


        motor1Configurator.apply(Slot0Configs);

        final DutyCycleOut stopRequest = new DutyCycleOut(0);

        
        mElevator2.setControl(new StrictFollower(mElevator1.getDeviceID()));


        mElevator1.setControl(stopRequest);

        //Values here are weird because we're simulating a cascade as just a really heavy 1 stage
        mElevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 5.333, Units.lbsToKilograms(85), Units.inchesToMeters(0.5), Units.inchesToMeters(28), Units.inchesToMeters(28.0), true, Units.inchesToMeters(6.75), null);

     }

    
    
    /** 
     * Request the elevator to go to a height
     * @param inches distance of the carriage, in inches from the bottom of the elevator.
     */
    public void GoToPosition(double inches) {
        mDesiredPosition = inches;
        mElevator1.setControl(new MotionMagicVoltage(inches));
    }

    
    /** 
     * @return double
     */
    public double GetPosition() {
        return mElevator1.getPosition().getValueAsDouble();
    }

    
    /** 
     * @return boolean
     */
    public boolean isAtPosition(){
        return Math.abs(GetPosition() - mDesiredPosition) < 1.0;
    }

    /** 
     * Stops the elevator
     */
    public void stopElevator(){
        mElevator1.stopMotor();
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

    @Override
    public void simulationPeriodic()
    {
        mElevatorSim.setInputVoltage(mElevator1.getMotorVoltage().getValueAsDouble());
        mElevator1.setPosition(Units.metersToInches(mElevatorSim.getPositionMeters() * 3.0)); //Convert to cascade

    }

    @Override
    public void periodic()
    {

    }

}
