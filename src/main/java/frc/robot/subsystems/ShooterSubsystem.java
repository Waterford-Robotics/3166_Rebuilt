package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.ShooterCIDConstants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase{
     private TalonFX m_shooter1;
     private TalonFX m_shooter2;
     
    public ShooterSubsystem() {
        m_shooter1 = new TalonFX(ShooterCIDConstants.k_shooterShooter1);
        m_shooter2 = new TalonFX(ShooterCIDConstants.k_shooterShooter2);
       
        
    }

    public void startShooterCommand() {
        m_shooter1.set(ShooterConstants.k_shooter1Speed);
        m_shooter2.set(ShooterConstants.k_shooter2Speed);
       
    }

    public void stopShooterCommand() {
        m_shooter1.set(0);
        m_shooter2.set(0);
        
    }


}
