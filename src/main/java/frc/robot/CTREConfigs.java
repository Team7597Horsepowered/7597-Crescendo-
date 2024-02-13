package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public static CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */

        swerveAngleFXConfig.Slot0.kP = Constants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.angleKD;

        /* Swerve Drive Motor Configuration */

        swerveDriveFXConfig.Slot0.kP = Constants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.driveKD;
        
    }
}