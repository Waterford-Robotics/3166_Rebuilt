
package frc.robot;

//import com.pathplanner.lib.config.PIDConstants;
//import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;

public final class Constants {
    
    

    public static final class ControllerConstants {
        
        public static final int resetNavX = 8;
        public static final int intakeUp = 6; 
        public static final int intakeDown = 5; 
        public static final int intake = 4;
        public static final int intake2 = 3;
        public static final int intake3 = 2;
        public static final int intake4 = 1; 
        public static final int shooterRev = 5; 
        public static final int indexer = 6; 
       
       public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
       public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
    }
    public static final class canID {
        public static final int k_indexCID = 20;
        
    }

    public static final class AutoConstants {
        
  }  
  public static final class IndexerConstants{
        public static final double k_IndexerSpeed = .4;
        public static final double k_ShooterIntakeSpeed = -.4;

  }
    public static final class ShooterCIDConstants{
        public static final int k_shooterShooter1 = 31; 
        public static final int k_shooterShooter2 = 32; 
        public static final int k_shooterIntake = 30; 
        
    }
    public static final class IndexCIDConstants{
        public static final int k_indexMotor = 20; 
    }
    
    public static final class ShooterConstants{
        public static final double k_shooter1Speed = -.4;
        public static final double k_shooter2Speed = .4;
       
        
    }
     public static final class IntakeChannelConstants{
        public static final int k_Elavator = 1;
        public static final int k_MainIntake = 2;
       
        
    }
    public static final class IntakeConstants{
        public static final double k_ElivatorMoveSpeedDown = -.2;
        public static final double k_ElivatorMoveSpeedUp = .2;

        public static final double k_MainIntakeSpeed = .75;
       
        
    }
}

