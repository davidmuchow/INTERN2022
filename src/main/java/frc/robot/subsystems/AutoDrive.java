// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoDrive extends SubsystemBase {
  private DriveTrain driveSub;


  public AutoDrive(DriveTrain driveSub) {
    this.driveSub = driveSub;
  }
  
  public void useEncoders(double distance) {
    driveSub.setMotors(0.25);
  }

  public void useEncoders(double distance, double speed) {
    driveSub.setMotors(speed);

  }

  @Override
  public void periodic() {
    if (RobotContainer.CUR_DRIVE_MODE == Constants.ROBOT_MODES.AUTO) {

    }
  }
}
