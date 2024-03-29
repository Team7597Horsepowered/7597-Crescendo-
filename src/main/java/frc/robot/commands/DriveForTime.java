// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForTime extends Command {
  /** Creates a new DriveForTime. */
  Timer timer;
  double time;
  double speed;
  SwerveSubsystem m_SwerveSubsystem;
  public DriveForTime(double time, double speed, SwerveSubsystem m_SwerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveSubsystem);
    this.m_SwerveSubsystem = m_SwerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < time){
      m_SwerveSubsystem.driveCommand(
      () -> speed * Constants.maxSpeed,
      () -> 0.0,
      () -> 0.0 
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
