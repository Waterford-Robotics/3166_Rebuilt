// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.sampled.Control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swervesubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;



public class RobotContainer {
     private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.k_driverController);
     public final Swervesubsystem m_swerveSubsystem = new Swervesubsystem();
     public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
     public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
     
     


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {


              //resets navX
        new JoystickButton(m_driverController.getHID(), ControllerConstants.resetNavX)
        .onTrue(new InstantCommand(
            () -> m_swerveSubsystem.zeroGyro(),
            m_swerveSubsystem));
        new JoystickButton(m_driverController.getHID(), ControllerConstants.resetNavX)
        .onTrue(new InstantCommand(
            () -> m_indexerSubsystem.spinIndexerCommand(),
            m_indexerSubsystem));
        new JoystickButton(m_driverController.getHID(), ControllerConstants.resetNavX)
        .onTrue(new InstantCommand(
            () -> m_shooterSubsystem.spinShooterCommand(),
            m_indexerSubsystem)); 
        new JoystickButton(m_driverController.getHID(), ControllerConstants.resetNavX)
        .onTrue(new InstantCommand(
            () -> m_shooterSubsystem.spinIntakeCommand(),
            m_indexerSubsystem));          
        
/* 
        // strafe right (right trigger)
        new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
        .whileTrue(new RunCommand(
            () -> m_swerveSubsystem.robotOrientedDriveCommand(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig)),
            m_swerveSubsystem));
    
        //strafe left, left trigger
        new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
        .whileTrue(new RunCommand(
            () -> m_swerveSubsystem.robotOrientedDriveCommand(() -> -1*m_driverController.getRawAxis(ControllerConstants.k_lefttrig)),
            m_swerveSubsystem)); */
        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    
  }
  


   Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
        () -> m_driverController.getRightX() * DriveConstants.k_turnRate); 
}
