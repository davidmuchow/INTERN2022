// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.DriveTrain;

public class EncoderDriveDistance extends CommandBase {
  /** Creates a new EncoderDriveDistance. */
  AutoDrive auto;
  DriveTrain driveSub;
  double distance;
  double requiredTicks;
  double currentDistance;
  double speed;

  public EncoderDriveDistance(AutoDrive autoHandler, DriveTrain driveSub, double distanceInMeters, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.  
    this.auto = autoHandler;
    this.distance = Units.metersToInches(distanceInMeters);
    this.speed = speed;
    this.driveSub = driveSub;

    requiredTicks = (distance / Constants.ENCODER_CONSTANTS.WHEEL_CIRCUMFRENCE * Constants.ENCODER_CONSTANTS.GEAR_RATIO) * 42 / 40 * 2; 
    
    SmartDashboard.putNumber("ticksreq", requiredTicks);

    addRequirements(autoHandler, driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    auto.resetEncoders();
    driveSub.setCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = auto.useEncoders(distance, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    auto.stopMotors();
    driveSub.setBrakes();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentDistance >= requiredTicks);
  }
}
