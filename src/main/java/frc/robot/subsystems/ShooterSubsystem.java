package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.IndexMotorChannelConstants;
import frc.robot.Constants.MotorChannelConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
     private Talon shooter1;
     private Talon shooter2;
     
    public ShooterSubsystem() {
        shooter1 = new Talon(MotorChannelConstants.k_shooterShooter1);
        shooter2 = new Talon(MotorChannelConstants.k_shooterShooter2);
       
        
    }

    public void startShooterCommand() {
        shooter1.set(ShooterConstants.k_shooter1Speed);
        shooter2.set(ShooterConstants.k_shooter2Speed);
       
    }

    public void stopShooterCommand() {
        shooter1.set(0);
        shooter2.set(0);
        
    }


}
