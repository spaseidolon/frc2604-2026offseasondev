// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Controls;
import frc.robot.commands.TeleopOperator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.*;


public class RobotContainer {
 /* Controllers */
 
  //private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  /* Subsystems */
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final OperatorSubsystem o_Operator = new OperatorSubsystem();
  


  public RobotContainer() {
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve));
    o_Operator.setDefaultCommand(new TeleopOperator(o_Operator));
    configureBindings();
    o_Operator.configOperatorParams();
    
  }

  private void configureBindings() {
    /* Driver Buttons */
    Controls.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


    /* Operator I/O */

    
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(new Move(s_Swerve));
  }
}
