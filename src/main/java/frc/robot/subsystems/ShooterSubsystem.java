package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorChannelConstants;


public class ShooterSubsystem extends SubsystemBase{
     private Talon shooterMotors;
     private Talon intakeMotor;

    public ShooterSubsystem() {
        shooterMotors = new Talon(MotorChannelConstants.k_shooterMotors);
        intakeMotor = new Talon(MotorChannelConstants.k_intakeMotor);
    }

    public void spinShooterCommand() {
        shooterMotors.set(1);
    }

    public void stopShooterCommand() {
        shooterMotors.set(0);
    }
    public void spinIntakeCommand() {
        intakeMotor.set(1);
    }

    public void stopIntakeCommand() {
        intakeMotor.set(0);
    }


}
