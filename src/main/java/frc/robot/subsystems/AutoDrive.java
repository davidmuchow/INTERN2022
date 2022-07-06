// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

public class AutoDrive extends SubsystemBase {
  private DriveTrain driveSub;
  private ArrayList<RelativeEncoder> encoders;

  public AutoDrive(DriveTrain driveSub) {
    this.driveSub = driveSub;
    encoders = new ArrayList<RelativeEncoder>();
    for(CANSparkMax[] cur_group : driveSub.SparkMaxArrays) {
      for(CANSparkMax controller_ptr : cur_group) {
        encoders.add(controller_ptr.getEncoder());
      }
    }
  }

  public void useEncoders(double distance) {
    driveSub.setMotors(0.25);
    distance = Units.metersToInches(distance);
    double distanceToRotations = distance / Constants.ENCODER_CONSTANTS.WHEEL_CIRCUMFRENCE;
    double currentTicks = encoders.get(0).getPosition();
    if((currentTicks / Constants.ENCODER_CONSTANTS.TICKS_PER_ROTATION) == distanceToRotations) {
      driveSub.setMotors(0);
    }
  }

  public double useEncoders(double distance, double speed) {
    driveSub.setMotors(speed);
    double currentTicks = Math.abs(encoders.get(0).getPosition());
    SmartDashboard.putNumber("currentTicks", currentTicks);
    return currentTicks;
  }
  
  public void stopMotors() {
    driveSub.setMotors(0);
  }
  
  public void resetEncoders() {
    for(CANSparkMax[] motorarr : driveSub.SparkMaxArrays) {
      for(CANSparkMax currMotor : motorarr) {
        currMotor.getEncoder().setPosition(0);
      }
    }
  }

  @Override
  public void periodic() {}
}
