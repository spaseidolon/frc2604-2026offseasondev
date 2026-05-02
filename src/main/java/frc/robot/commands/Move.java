package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class Move extends Command {
    public SwerveSubsystem swerve;

    public Move(SwerveSubsystem s_Swerve){
        this.swerve = s_Swerve;
        addRequirements(s_Swerve);
        
    }
        @Override
        public void initialize(){

        }
        
        @Override 
        public void execute(){
        Translation2d translation = new Translation2d(-0.3, 0);
        double rotation = 0;
        boolean fieldRelative = true;
        boolean openLoop = true;
        swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
    }

