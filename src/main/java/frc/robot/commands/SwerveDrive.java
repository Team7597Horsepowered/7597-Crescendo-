// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrive extends Command {
  /** Creates a new SwerveDrive. */
  Swerve m_swerveSubsystem;
  XboxController drive;
  public SwerveDrive(Swerve m_swerveSubsystem, XboxController drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.drive(
      new Translation2d(-drive.getLeftX() * Constants.maxSpeed, -drive.getLeftY() * Constants.maxSpeed),
      -drive.getRightX() * Constants.maxAngularVelocity,
      true,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
