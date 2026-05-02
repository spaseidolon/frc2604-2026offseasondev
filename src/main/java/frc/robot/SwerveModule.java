package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.conversions.Conversions;
import frc.lib.util.SwerveModuleConstants;

//ANALOG ENCODERS
//import frc.lib.util.Resolver;

//TALONFX
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


//VENOM
//import com.playingwithfusion.CANVenom;
//import com.playingwithfusion.CANVenom.ControlMode;

//CANCODER STEERING
//import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    public double angleOffset;
    

    //private CANVenom mAngleMotor;
    private TalonFX mAngleMotor;
   
    //private CANVenom mDriveMotor;
    private TalonFX mDriveMotor;

    //public Resolver angleEncoder;
    public CANcoder angleEncoder;
    private boolean forceAbsolute;
    private double lastAngle;
    public PIDController anglePID;
    public boolean isBackhand;
    public Rotation2d angleError;
    public Rotation2d outputAngle;
    public Rotation2d processAngle;
    public SwerveModuleState commandedState;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.forceAbsolute = moduleConstants.forceAbsolute;
        this.isBackhand = false;
        this.outputAngle = Rotation2d.fromDegrees(0);
        this.angleError = Rotation2d.fromDegrees(0);
        this.commandedState = new SwerveModuleState();
        
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        //angleEncoder = new Resolver(moduleConstants.encoderID);
        angleEncoder = new CANcoder(moduleConstants.encoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        //mAngleMotor = new CANVenom(moduleConstants.angleMotorID);
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        //mDriveMotor = new CANVenom(moduleConstants.driveMotorID);
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);

        configDriveMotor();


        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        commandedState = desiredState;
            if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            percentOutput = isBackhand ? -percentOutput : percentOutput;


            //TALONFX
            mDriveMotor.set(percentOutput);

            //VENOM
            //mDriveMotor.setCommand(CANVenom.ControlMode.VoltageControl, percentOutput * 12);
        }
        else {
    
            double velocity = Conversions.CtreV6Conversions.MPSTomotor(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);


            //TALONFX
            VelocityDutyCycle targetVelocity =  new VelocityDutyCycle(velocity);
            mDriveMotor.setControl(targetVelocity);
            
            //VENOM
            //mDriveMotor.setCommand(CANVenom.ControlMode.SpeedControl, velocity);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        outputAngle = isBackhand ? desiredState.angle.plus(Rotation2d.fromDegrees(180)) : desiredState.angle;
        angleError = desiredState.angle.plus(Rotation2d.fromDegrees(360));
        processAngle = Rotation2d.fromDegrees(angleEncoderGet() + 360);
        angleError = processAngle.minus(angleError);

        /* BACKHANDING LOGIC - UNTESTED */
       /*  if (angleError.getDegrees() > 120 || angleError.getDegrees() < -120) { 
            isBackhand = !isBackhand;
            outputAngle = outputAngle.plus(Rotation2d.fromDegrees(180));
        }*/
        
        if (angleError.getDegrees() > 270){
           outputAngle = outputAngle.minus(Rotation2d.fromDegrees(360));
           processAngle = processAngle.minus(Rotation2d.fromDegrees(360));
        }
        if (angleError.getDegrees() < -270){
            outputAngle = outputAngle.plus(Rotation2d.fromDegrees(360));
            processAngle = processAngle.plus(Rotation2d.fromDegrees(360));
        }
        processAngle = processAngle.minus(Rotation2d.fromDegrees(360));

        /*TALONFX =*/if (forceAbsolute) {mAngleMotor.setVoltage(anglePID.calculate(processAngle.getDegrees(), outputAngle.getDegrees()));}
        /*VENOM =*/ //if (forceAbsolute) {mAngleMotor.setCommand(ControlMode.VoltageControl, anglePID.calculate(processAngle.getDegrees(), outputAngle.getDegrees()));}

        /*TALONFX*/ else {  PositionDutyCycle targetPositionDutyCycle = new PositionDutyCycle(Conversions.CtreV6Conversions.degreesTomotor(angle, Constants.Swerve.angleGearRatio));
                            mAngleMotor.setControl(targetPositionDutyCycle);}
        /*VENOM*/ //else {mAngleMotor.setCommand(ControlMode.PositionControl, Conversions.CtreConversions.degreesTomotor(angle, Constants.Swerve.angleGearRatio)); }
        lastAngle = outputAngle.getDegrees();
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.CtreV6Conversions.degreesTomotor(angleEncoderGet() - angleOffset, Constants.Swerve.angleGearRatio);
        lastAngle = absolutePosition;
    }
 
    private void configAngleEncoder(){ 
        /* Not applicable for analog absolute encoders. Re-enable if using CANcoders */
        //angleEncoder.zero();
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public double angleEncoderGet(){
        //angleEncoder.update();
        //return angleEncoder.value - this.angleOffset;
        return 0;
    }

    private void configAngleMotor(){
    /* TALONFX */
    mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralModeValue);
    
    mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    
    /*  VENOM  */
    /*
        mAngleMotor.//setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.//setPID(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, Constants.Swerve.angleKF, 0);
        mAngleMotor.//setMaxAcceleration(Constants.Swerve.angleMaxAcceleration);
        mAngleMotor.//setMaxJerk(Constants.Swerve.angleMaxJerk);
    */
        if (forceAbsolute) anglePID = new PIDController(Constants.Swerve.angleKP / 100, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
        resetToAbsolute();
    }

    private void configDriveMotor(){    
            
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);//configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
       
       
        /* VENOM */
       
        /*
        mDriveMotor.//setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.//setPID(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD, Constants.Swerve.driveKF, 0);
        mDriveMotor.//setMaxAcceleration(Constants.Swerve.driveMaxAcceleration);
        mDriveMotor.//setMaxJerk(Constants.Swerve.driveMaxJerk);
        */
    }
/*
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }
*/

    public SwerveModuleState getState(){
        var velocitySignal = mDriveMotor.getVelocity();
        var positionSignal = mDriveMotor.getPosition();
        double velocity = 0;
        velocity = Conversions.CtreV6Conversions.motorToMPS(velocitySignal.getValueAsDouble(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle;
        if (this.forceAbsolute) {angle = Rotation2d.fromDegrees(angleEncoderGet());}
        else {angle = Rotation2d.fromDegrees(Conversions.CtreV6Conversions.motorToDegrees(positionSignal.getValueAsDouble(), Constants.Swerve.angleGearRatio));}        return new SwerveModuleState(velocity, angle);
    
    }
    
    public SwerveModulePosition getPosition(){
        var positionSignal = mDriveMotor.getPosition();
        double position = Conversions.CtreV6Conversions.motorToMPS(positionSignal.getValueAsDouble(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle;
        if (this.forceAbsolute) {angle = Rotation2d.fromDegrees(angleEncoderGet());}
        else {
            var currentAnglePosition = mAngleMotor.getPosition();
            angle = Rotation2d.fromDegrees(Conversions.CtreV6Conversions.motorToDegrees(currentAnglePosition.getValueAsDouble(), Constants.Swerve.angleGearRatio));}
        return new SwerveModulePosition(position, angle);
    }
}