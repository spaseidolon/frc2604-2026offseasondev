package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class OperatorSubsystem extends SubsystemBase {
    


    //declare motors
    private static SparkMax coralLeft = new SparkMax(AuxSystems.coralLeftID, MotorType.kBrushless);
    private static SparkMax coralRight = new SparkMax(AuxSystems.coralRightID, MotorType.kBrushless);
    private static TimeOfFlight coralPresentSensor = new TimeOfFlight(AuxSystems.coralPresentID);
    private static double coralPresentRange = 0;
    public static boolean coralPresent = false;
    private static Limelight l_Limelight = new Limelight();
    
    //private static SparkMax algae = new SparkMax(AuxSystems.algaeID, MotorType.kBrushless);

    //declare elbow components
    private static TalonSRX elbow = new TalonSRX(AuxSystems.elbowID);
    private static double elbowCurrentPosition = 0;
    private static boolean elbowMoveSafety = false;  //Move OK bit to allow elevator motion. Default false, toggles true whenever elbow is forward enough to clear elevator braces
    public static boolean coralReleaseSafety = false; //Toggles true when coral mechanism is aimed outside of robot
    public static double elbowHome = 0;
    public static double elbowReef = 0;
    public static double elbowFeed = 0;

    //declare elevator components
    private static TalonFX elevator = new TalonFX(AuxSystems.elevatorID);
    private static TimeOfFlight elevatorLidar = new TimeOfFlight(AuxSystems.elevatorSensorID);
    public static double elevatorHeight = 0;
    public static PIDController elevatorPIDController = new PIDController(AuxSystems.elevatorP, AuxSystems.elevatorI, AuxSystems.elevatorD);


    
    
    public static void configOperatorParams(){
        //Coral configs
        SparkMaxConfig coralLeftConfig = new SparkMaxConfig();
        coralLeftConfig.idleMode(IdleMode.kBrake);
        coralLeftConfig.inverted(true);
        coralLeftConfig.smartCurrentLimit(AuxSystems.coralStallCurrent, AuxSystems.coralFreeCurrent, AuxSystems.coralStallThreshold);
        //create copy of config for other coral motor, so we can flip its inversion value
        SparkMaxConfig coralRightConfig = coralLeftConfig;
        coralRightConfig.inverted(false);
        //push configs to devices
        coralLeft.configure(coralLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralRight.configure(coralRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Algae configs
        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.inverted(true);
        algaeConfig.smartCurrentLimit(AuxSystems.algaeStallCurrent, AuxSystems.algaeFreeCurrent, AuxSystems.algaeStallThreshold);
        //push config to device
       // algae.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        //Elbow configs
        TalonSRXConfiguration elbowConfig = new TalonSRXConfiguration();

        //Sensing configs
        elbowConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;


        //Closed-loop parameter config, since elbow control is done with onboard PID controller on Talon
        elbowConfig.slot0.kP = Constants.AuxSystems.elbowP;
        elbowConfig.slot0.kI = Constants.AuxSystems.elbowI;
        elbowConfig.slot0.kD = Constants.AuxSystems.elbowD;
        elbowConfig.slot0.kF = Constants.AuxSystems.elbowF;

        //Motion profiling configs
        elbowConfig.closedloopRamp = Constants.AuxSystems.elbowRampRate;
        elbowConfig.auxPIDPolarity = true;

        //Current Limiting config
        elbowConfig.continuousCurrentLimit = Constants.AuxSystems.elbowContinuousCurrentLimit;
        elbowConfig.peakCurrentLimit = Constants.AuxSystems.elbowPeakCurrentLimit;
        elbowConfig.peakCurrentDuration = Constants.AuxSystems.elbowPeakCurrentDuration;

        //Push config to device
        elbow.configAllSettings(elbowConfig);


        //Elevator configs

        //PID controller configs
        elevatorPIDController.setTolerance(AuxSystems.elevatorErrorTolerance);

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        //No need to config Talon closed-loop, since PID controller for elevator is on RIO (configured above)

        


        //Motion profiling configs
        elevatorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = AuxSystems.elevatorRampRate;
        elevatorConfig.MotorOutput.Inverted = AuxSystems.elevatorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //Current Limiting config
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = AuxSystems.elevatorPeakCurrent;
        elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = AuxSystems.elevatorContinuousCurrent;
        elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = AuxSystems.elevatorPeakCurrentDuration;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = AuxSystems.elevatorStatorCurrent;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        //Push config to device
        elevator.getConfigurator().apply(elevatorConfig);

        l_Limelight.illuminate();
        elbowHome = elbow.getSelectedSensorPosition();
        elbowFeed = elbowHome + AuxSystems.elbowFeederTaughtValue;
        elbowReef = elbowHome + AuxSystems.elbowReefTaughtValue;
        

    }
public static void updateElevator(double target){
//checks elbow clear of elevator, updates PID if OK to move
double elevatorOutput = elbowMoveSafety ? elevatorPIDController.calculate(elevatorHeight, target): 0;
//neutrals the motor if commanded output is low, to avoid locked-rotor burnout
elevatorOutput = (Math.abs(elevatorOutput) > 0.10) ? elevatorOutput : 0;
if (Math.abs(elevatorPIDController.getError()) > AuxSystems.elevatorErrorTolerance) {
    elevatorPIDController.reset();
} 
//pushes output value to motor
VoltageOut elevatorRequest = new VoltageOut(elevatorOutput);
SmartDashboard.putNumber("Elevator Commanded Power: ", elevatorOutput);
SmartDashboard.putNumber("Elevator PID Error: ", elevatorPIDController.getError());

elevator.setControl(elevatorRequest);
}

public static void updateElbow(double target) {
//pushes updated setpoint to Talon
elbow.set(TalonSRXControlMode.Position, target);
}

public static void updateCoral(double power){

    coralLeft.set(power);
    coralRight.set(-power);
}

public static void updateAlgae(double power){
   // algae.set(power);
}


@Override public  void periodic(){



    /*
     * MECHANISM TELEMETRY 
     */


    //updates elevator height reading, and publishes to Dashboard
    elevatorHeight = (elevatorLidar.getRange() - AuxSystems.elevatorOffset);
    SmartDashboard.putNumber("Elevator Height :" , elevatorHeight);
    SmartDashboard.putNumber("Elevator Lidar Raw: ", elevatorLidar.getRange());
    SmartDashboard.putNumber("Elevator Stator Current: ", elevator.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Supply Current: ", elevator.getSupplyCurrent().getValueAsDouble());

    //updates elbow sensor reading, checks clear of elevator, and publishes both values to Dashboard
    elbowCurrentPosition = elbow.getSelectedSensorPosition();
    SmartDashboard.putNumber("Elbow Position: ", elbowCurrentPosition);
    elbowMoveSafety = elbowCurrentPosition > AuxSystems.elbowSoftLimit;
    SmartDashboard.putBoolean("Elbow Move Safety OK: ", elbowMoveSafety);
    coralReleaseSafety = true ;//elbowCurrentPosition > AuxSystems.elbowCoralReleaseSafety;

    SmartDashboard.putBoolean("Coral Release Safety: ", coralReleaseSafety);
    coralPresentRange = coralPresentSensor.getRange();
    coralPresent = coralPresentRange < AuxSystems.coralPresentThreshold;
    SmartDashboard.putNumber("Part Present Raw Sensor: ", coralPresentRange);
    SmartDashboard.putBoolean("Part Present: ", coralPresent);

 



}
}