package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;



import com.ctre.phoenix6.signals.NeutralModeValue;

import org.ejml.equation.VariableType;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.Units;





public class Configs {
  public static final class IndexerConfigs {

     public static final TalonFXConfiguration IdexerConfig = new TalonFXConfiguration();
     public static final TalonFXConfiguration siConfig = new TalonFXConfiguration();
    static{
      

    }
    
  }
  public static final class ShooterConfigs {
     public static final TalonFXConfiguration s1Configs = new TalonFXConfiguration();
     public static final TalonFXConfiguration s2Configs = new TalonFXConfiguration();
     static{
  
     }
    
  }
}