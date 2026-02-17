package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorChannelConstants;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase{
     private Talon shooter1;
     private Talon shooter2;
     private Talon intake;


    public ShooterSubsystem() {
        shooter1 = new Talon(MotorChannelConstants.k_shooterShooter1);
        shooter2 = new Talon(MotorChannelConstants.k_shooterShooter2);
        intake = new Talon(MotorChannelConstants.k_shooterIntake);
        
    }

    public void spinShooterCommand() {
        shooter1.set(ShooterConstants.k_shooter1Speed);
        shooter2.set(ShooterConstants.k_shooter2Speed);
        intake.set(ShooterConstants.k_shooterIntakeSpeed);
    }

    public void stopShooterCommand() {
        shooter1.set(0);
        shooter2.set(0);
        intake.set(0);
    }
    public void spinIntakeCommand() {
        intake.set(0.2);
    }

    public void stopIntakeCommand() {
        intake.set(0);
    }


}
