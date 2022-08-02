// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TurnWithGyro extends CommandBase {

  double targetAngle;
  DriveTrain driveSub;
  AHRS NavX;

  /** Creates a new TurnWithGryo. */
  public TurnWithGyro(double targetAngle, DriveTrain driveSub, AHRS NavX) {
    this.targetAngle = targetAngle;
    this.driveSub = driveSub;
    this.NavX = NavX;
    addRequirements(driveSub);
  }

  // Called when the command is initially schedule.
  @Override
  public void initialize() {
    NavX.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("current", Robot.currentAngle);
    SmartDashboard.putNumber("target", targetAngle);

    if(targetAngle > 0) {
      CANSparkMax[] leftSide = driveSub.SparkMaxArrays.get(0);
      CANSparkMax[] rightSide = driveSub.SparkMaxArrays.get(1);
      for(CANSparkMax motor : leftSide) {
        motor.set(0.15);
      }
      for(CANSparkMax motor : rightSide) {
        motor.set(-0.15);
      }
      if(targetAngle < Robot.currentAngle) {
        driveSub.setMotors(0);
      }
    }
    if(targetAngle < 0) {
      
      if(targetAngle > Robot.currentAngle) {
        driveSub.setMotors(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.setBrakes();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(NavX.getAngle()) >= targetAngle;
  }
}
