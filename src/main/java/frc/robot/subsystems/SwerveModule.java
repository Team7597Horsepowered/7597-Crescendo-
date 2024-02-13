package frc.robot.subsystems;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.OnboardModuleState;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;


    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private SparkPIDController driveController;
    private SparkPIDController angleController;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
            mDriveMotor.set(percentOutput);    
        }
        else {
            driveController.setReference(
            desiredState.speedMetersPerSecond,
            CANSparkMax.ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
    }

    private void configAngleMotor(){
        
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.angleMotorInvert);
        mAngleMotor.setIdleMode(IdleMode.kCoast);
        integratedAngleEncoder.setPositionConversionFactor(Constants.angleConversionFactor);
        angleController.setP(Constants.angleKP);
        angleController.setI(Constants.angleKI);
        angleController.setD(Constants.angleKD);
        angleController.setFF(Constants.angleKF);
        mAngleMotor.enableVoltageCompensation(Constants.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        

        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.driveMotorInvert);
        mDriveMotor.setIdleMode(IdleMode.kBrake);
        driveEncoder.setVelocityConversionFactor(Constants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.driveConversionPositionFactor);
        driveController.setP(Constants.angleKP);
        driveController.setI(Constants.angleKI);
        driveController.setD(Constants.angleKD);
        driveController.setFF(Constants.angleKF);
        mDriveMotor.enableVoltageCompensation(Constants.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    //everything below here is fine.

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveEncoder.getVelocity(), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }
}