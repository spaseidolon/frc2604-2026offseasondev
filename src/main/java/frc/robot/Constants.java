package frc.robot;

import com.ctre.phoenix6.signals.*;
//import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    

public static final class Controls{
    /* 
     * ALL ROBOT CONTROLS ARE DEFINED IN THIS CLASS
     * 
    */
    public static final Joystick driver = new Joystick(0);
    public static final Joystick operator = new Joystick(1);
    public static final double stickDeadband = 0.1;


    /* Driver Axes */
    public static final int translationAxis = XboxController.Axis.kLeftY.value;
    public static final int strafeAxis = XboxController.Axis.kLeftX.value;
    public static final int rotationAxis = XboxController.Axis.kRightX.value;
    public static final int chokeAxis = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    public static final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

    


    /*Operator Buttons */
    public static final int coralCapture = XboxController.Button.kRightBumper.value;
    public static final int coralRelease = XboxController.Axis.kRightTrigger.value;
    public static final int coralBurp = XboxController.Button.kLeftBumper.value;
    public static final int algaeCapture = XboxController.Button.kLeftBumper.value;
    public static final int algaeRelease = XboxController.Axis.kLeftTrigger.value;

    public static final int mechL1Request = XboxController.Button.kA.value;
    public static final int mechL2Request = XboxController.Button.kX.value;
    public static final int mechL3Request = XboxController.Button.kY.value;
    public static final int mechL4Request = XboxController.Button.kB.value;

    public static final int mechBias = XboxController.Axis.kLeftY.value;

}


    public static final class Swerve {
        public static final boolean fieldRelative = true;
        public static final boolean openLoop = false;
        //public static final int pigeonID = 20;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        public static final double axisScaler = 0.9;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20);
        public static final double wheelBase = Units.inchesToMeters(29.125);
        public static final double wheelDiameter = Units.inchesToMeters(3.7);//change to 3.7ish for MK4s when sure 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); //SDS MK4 L1=8.14, L2=6.75
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1 is common steer ratio for all SDS MK4 modules

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase/2, -trackWidth/2),
            new Translation2d(wheelBase/2, trackWidth/2),
            new Translation2d(-wheelBase/2, trackWidth/2),
            new Translation2d(-wheelBase/2, -trackWidth/2));
            


        /*new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));*/

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final int angleStatorCurrentLimit = 30;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final double angleMaxAcceleration = 0.05;
        public static final double angleMaxJerk = 0.7;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final int driveStatorCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        public static final double driveMaxAcceleration = 0.15;
        public static final double driveMaxJerk = 0.2;

        /* Angle Motor PID Values */
        public static final double angleKP = 3;//5 for Krakens, 5 for Venoms
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0.2;//0 for Krakens, 0.30 for Venoms

        /* Drive Motor PID Values */
        public static final double driveKP = 10;// 0.04 for Krakens, Venoms
        public static final double driveKI = 0.1;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.8;// 0.04 for Krakens, Venoms

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667);
        public static final double driveKV = (2.44);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 6.5; //meters per second
        public static final double maxAngularVelocity = 8;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralModeValue = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralModeValue = NeutralModeValue.Coast;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Right Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int encoderID = 0;
            public static final double angleOffset = 218;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int encoderID = 2;
            public static final double angleOffset = 1;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }



        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int encoderID = 3;
            public static final double angleOffset = 22;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }


        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int encoderID = 1;
            public static final double angleOffset = 343;
            public static final boolean forceabsolute = true;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, forceabsolute);
        }
        
    }
    public static final class AuxSystems {


        //elevator CAN IDs
        public static final int elevatorSensorID = 10;
        public static final int elevatorID = 20;

        //Coral CAN IDs
        public static final int coralLeftID = 11;
        public static final int coralRightID = 12;
        public static final int coralPresentID = 15;

        //Algae CAN IDs
        public static final int algaeID = 13;

        //Arm CAN IDs
        public static final int elbowID = 9;

        //=================================================================================
        //  AUX SYSTEM TUNING CONSTANTS
        //=================================================================================
        //coral power settings
        public static final double coralCapturePower = 0.3;
        public static final double coralReleasePower = 1;
        public static final double coralBurpPower = -0.5;

        public static final double coralPresentThreshold = 100;

        /*Stator current limit during bottom-end operation (i.e. starting/hard acceleration and stall conditions). 
        This limit is set higher than free limit to allow additional current (torque) during high-load scenarios. 
        Maximum safe value for NEO550 is 20A (default is 80A, which is maximum safe for a full-size NEO)*/  
        public static final int coralStallCurrent = 20; 
        //Stator current limit during top-end operation. This is the limit used during most normal operation.
        public static final int coralFreeCurrent = 10;  
        public static final int coralStallThreshold = 2000;  //Breakover RPM to switch between stall and free current limits

        public static final double algaeCapturePower = -0.3;
        public static final double algaeReleasePower = 0.5;
        /*Stator current limit during bottom-end operation (i.e. starting/hard acceleration and stall conditions). 
        This limit is set higher than free limit to allow additional current (torque) during high-load scenarios. 
        Maximum safe value for NEO550 is 20A (default is 80A, which is maximum safe for a full-size NEO)*/
        public static final int algaeStallCurrent = 15;
        //Stator current limit during top-end operation. This is the limit used during most normal operation.
        public static final int algaeFreeCurrent = 5;
        public static final int algaeStallThreshold = 2000;  //Breakover RPM to switch between stall and free current limits




        //========================================================
        //Elbow Teach data
        //========================================================

        //Elbow taught points are scaled in Talon analog resolution units (1024-tick with wraparound). Process points are absolute (Taught relative to raw sensor value)

                //TODO : Jump here to touchup Elbow points
        //Mastery data for elbow. Teach with forearm straight vertical
        public static final double elbowHome = 0;
        //Software safety position for elevator move OK. Teach with elbow behind vertical, but still far enough forward to clear elevator braces
        public static final double elbowSoftLimit = -1000;
        //Setpoint for placing a coral. Teach with coral mech at correct angle for reef placement
        public static final double elbowReefTaughtValue = 180;
        //Separate setpoint for steeper angle for L4 Reef
        public static final double elbowL4ReefTaughtValue = 180;
        //Setpoint for using the algae mechanism. Teach with coral swung clear of the algae mech
        public static final double elbowAlgaeTaughtValue = 0;
        //Setpoint for intaking from the feeder station. Teach with coral mech at correct angle to recieve piece from station
        public static final double elbowFeederTaughtValue = -180;
        //Setpoint for coral release safety. Teach with coral mechanism aimed ~6" ahead of bumpers
        public static final double elbowCoralReleaseSafety = -1000;


        //Elbow motion profiling
        public static final double elbowRampRate = 0.25; //Set to minimize chain thrashing

        //Elbow power settings
        public static final int elbowContinuousCurrentLimit = 15;
        public static final int elbowPeakCurrentLimit = 25;
        public static final int elbowPeakCurrentDuration = 2;

        //Elbow PID controller tuning coefficients
        public static final double elbowP = 6;
        public static final double elbowI = 0;
        public static final double elbowD = 0;
        public static final double elbowF = 0;


        //========================================================
        //Elevator Teach data
        //========================================================

        //Elevator taught data is scaled in millimeters. Process points are relative (Taught wih mastery position = zero)

                //TODO : Jump here to touchup Elevator heights
        //Mastery data for elevator. Teach with elevator down against hardstops
        public static final double elevatorOffset = 195;
        //Home Height for elevator in inches
        public static final double elevatorHome = 25;
        //Reef L1 height for elevator in inches
        public static final double elevatorL1 = 430;
        //Reef L2 height for elevator in inches
        public static final double elevatorL2 = 500;
        //Reef L3 height for elevator in inches
        public static final double elevatorL3 = 850;
        //Reef L4 height for elevator in inches
        public static final double elevatorL4 = 800;

        //Elevator motion profiling
        public static final double elevatorRampRate = 4; //Set conservatively to avoid drum slip
        public static final boolean elevatorInvert = true;
        
        //Elevator Current management
        public static final int elevatorStatorCurrent = 60;
        public static final int elevatorPeakCurrent = 20;
        public static final int elevatorContinuousCurrent = 15;
        public static final int elevatorPeakCurrentDuration = 5;


        //Elevator PID controller tuning coefficients
        public static final double elevatorP = 0.03;
        public static final double elevatorI = 0.0001;
        public static final double elevatorD = 0;
        public static final double elevatorErrorTolerance = 5; //Error deadband for PID controller

    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 10;
        public static final double kPYController = 10;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
