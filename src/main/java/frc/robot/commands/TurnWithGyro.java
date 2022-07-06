// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

public class TurnWithGyro extends CommandBase {

  double targetAngle;
  DriveTrain driveSub;

  /** Creates a new TurnWithGryo. */
  public TurnWithGyro(double targetAngle, DriveTrain driveSub) {
    this.targetAngle = targetAngle;
    this.driveSub = driveSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetAngle > 0) {
      driveSub.left.set(0.3);
      driveSub.right.set(0);
      if(targetAngle > Robot.currentAngle) {
        driveSub.setMotors(0);
        
      }
    }
    if(targetAngle < 0) {
      driveSub.left.set(0);
      driveSub.right.set(0.3);
      if(targetAngle < Robot.currentAngle) {
        driveSub.setMotors(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.currentAngle) >= targetAngle;
  }
}
