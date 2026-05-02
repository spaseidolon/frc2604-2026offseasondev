package frc.robot.commands;


import frc.robot.Constants.*;
import frc.robot.subsystems.OperatorSubsystem;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopOperator extends Command {
    

    public TeleopOperator(OperatorSubsystem o_Operator) {
        addRequirements(o_Operator);
    }
    @Override
    public void execute(){

        /*
         * Elevator/arm system has five requestable operating configurations:
         * Default - arm retracted, elevator low. This configuration keeps robot Cg low and roughly centered in frame. Ideal for transit and safety (protects mechanisms from contact). Algae can be scored into the processor from this position.
         * L1 - this is our feed/L1 reef position. Extends arm fully, causing wrist to rotate back and place coral mechanism at ~50deg up-angle. Lifts elevator to set coral mechanism at height for feeder station. This setting is also used to lob coral into the L1 trough on the reef
         * L2 - this is our L2 scoring position. Extends arm to Reef position, aligning coral mechanism at ~50deg down-angle. Lifts elevator to align coral mechanism with L2 reef peg.
         * L3 - this is our L3 scoring position. Extends arm to Reef position, aligning coral mechanism at ~50deg down-angle. Lifts elevator to align coral mechanism with L3 reef peg.
         * L4 - this is our L4 scoring position. Extends arm to Reef position, aligning coral mechanism at ~50deg down-angle. Lifts elevator to align coral mechanism with L4 reef peg.
         */


        //Defaults the elevator to home position (low hover)
        double elevatorTarget = AuxSystems.elevatorHome;
        //Elevator control logic : Goes to L1-4 based on operator input. Lower height requests supercede higher height requests, for robot stability (i.e. if L1 and L3 are requested at the same time, robot will move to L1)
        //elevatorTarget = Controls.operator.getRawButton(Controls.mechL4Request) ? AuxSystems.elevatorL4 : elevatorTarget;
        elevatorTarget = Controls.operator.getRawButton(Controls.mechL3Request) ? AuxSystems.elevatorL3 : elevatorTarget;
        elevatorTarget = Controls.operator.getRawButton(Controls.mechL2Request) ? AuxSystems.elevatorL2 : elevatorTarget;
        elevatorTarget = Controls.operator.getRawButton(Controls.mechL1Request) ? AuxSystems.elevatorL1 : elevatorTarget;

        //adds a +-75mm manual bias to the requested position, allowing the operator to do height adjustment on-the-fly
        elevatorTarget = elevatorTarget + (75 * Controls.operator.getRawAxis(Controls.mechBias));

        //Defaults the elbow to home (mastery position, tucked back inside robot for safety)
        double elbowTarget = OperatorSubsystem.elbowHome;
        
        //Elbow control logic : goes to Reef position if L2-L4 are requested, goes to Feeder position if Feeder is requested
        //elbowTarget = Controls.operator.getRawButton(Controls.mechL4Request) ? AuxSystems.elbowL4ReefTaughtValue : elbowTarget;
        elbowTarget = Controls.operator.getRawButton(Controls.mechL3Request) ? OperatorSubsystem.elbowReef : elbowTarget;
        elbowTarget = Controls.operator.getRawButton(Controls.mechL2Request) ? OperatorSubsystem.elbowReef : elbowTarget;
        elbowTarget = Controls.operator.getRawButton(Controls.mechL1Request) ? OperatorSubsystem.elbowFeed : elbowTarget;

        
        //Defaults mechanisms to neutral
        double coralPower = 0;
        double algaePower = 0;
        //Coral/Algae control logic : captures/releases coral and algae based on operator input. Release supercedes capture (stuck bumper protection)
        coralPower = Controls.operator.getRawButton(Controls.coralCapture) ? AuxSystems.coralCapturePower : coralPower ;
        //Automatically neutrals the coral motors if part present detected
        coralPower = OperatorSubsystem.coralPresent ? 0 : coralPower;
        //Burps coral
        coralPower = Controls.operator.getRawButton(Controls.coralBurp) ? AuxSystems.coralBurpPower : coralPower;
        //Checks for coral release commanded, applies power if so
        coralPower = (Math.abs(Controls.operator.getRawAxis(Controls.coralRelease)) > 0.5) ? AuxSystems.coralReleasePower : coralPower ;
        coralPower = OperatorSubsystem.coralReleaseSafety ? coralPower : 0;

        algaePower = Controls.operator.getRawButton(Controls.algaeCapture) ? AuxSystems.algaeCapturePower : algaePower ;
        algaePower = (Math.abs(Controls.operator.getRawAxis(Controls.algaeRelease)) > 0.5) ? AuxSystems.algaeReleasePower : algaePower ;

        //push values to mechanisms
            //TODO : Jump here to disable individual mechanisms - comment line to disable
        //OperatorSubsystem.updateElevator(elevatorTarget);
        //OperatorSubsystem.updateElbow(elbowTarget);        
        //OperatorSubsystem.updateCoral(coralPower);
        //OperatorSubsystem.updateAlgae(algaePower);

    }
}