// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveDrive swerveDrive;
  private final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150/7);
  private final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.12);

  public SwerveSubsystem(File directory) {
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch(Exception e){
      throw new RuntimeException(e);
    }


    swerveDrive.setHeadingCorrection(false);
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Units.feetToMeters(20));
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY){
    return run(() ->{
      double xInput = Math.pow(translationX.getAsDouble(), 3);
      double yInput = Math.pow(translationY.getAsDouble(), 3);
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 
                                                                      headingX.getAsDouble(), 
                                                                      headingY.getAsDouble(), 
                                                                      swerveDrive.getYaw().getRadians(), 
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    System.out.println("test");
    return run(() ->{
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
      Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()), 
      Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
      true,
      false);
    });
  }

  public void setupPathPlanner(){
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotVelocity,
      this::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(0.7, 0, 0),
        new PIDConstants(0.4, 0, 0.01),
        Constants.maxSpeed, 
        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
        new ReplanningConfig()
    ),
    () ->{
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    },
    this);
  }

  public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public void resetOdometry(Pose2d newPose){
    swerveDrive.resetOdometry(newPose);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  @Override
  public void periodic() {
  }
}
