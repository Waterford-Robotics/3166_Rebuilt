// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {


    /*  //resets navX
     new JoystickButton(m_driverController.getHID(), ControllerConstants.resetNavX)
     .onTrue(new InstantCommand(
         () -> m_swerveSubsystem.zeroGyro(),
         m_swerveSubsystem));

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
 }


  
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }



   


Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
   () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
   () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
   () -> m_driverController.getRightX() * DriveConstants.k_turnRate); 

}