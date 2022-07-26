// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AutoDrive extends SubsystemBase {
  private DriveTrain driveSub;
  private ArrayList<RelativeEncoder> encoders;
  private AHRS navX;
  public DifferentialDriveOdometry m_odometer;

  public AutoDrive(DriveTrain driveSub, AHRS navX) {
    this.driveSub = driveSub;
    encoders = new ArrayList<RelativeEncoder>();
    for(CANSparkMax[] cur_group : driveSub.SparkMaxArrays) {
      for(CANSparkMax controller_ptr : cur_group) {
        encoders.add(controller_ptr.getEncoder());
      }
    }
    this.navX = navX;
    resetEncoders();
    m_odometer = new DifferentialDriveOdometry(navX.getRotation2d());
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

  public void resetOdometry(Pose2d reseter) {
    resetEncoders();
    m_odometer.resetPosition(reseter, navX.getRotation2d());
  }
  
  public void resetEncoders() {
    for(CANSparkMax[] motorarr : driveSub.SparkMaxArrays) {
      for(CANSparkMax currMotor : motorarr) {
        currMotor.getEncoder().setPosition(0);
      }
    }
  }

  public double getPositionSpecific(int id) {
    SmartDashboard.putNumber("id"+id, encoders.get(id).getPosition() * Constants.ENCODER_CONSTANTS.GEARING);
    SmartDashboard.putNumber("dd", encoders.get(id).getCountsPerRevolution());
    return encoders.get(id).getPosition() * Constants.ENCODER_CONSTANTS.GEARING * Constants.ENCODER_CONSTANTS.WHEEM_CIRCUMFRENCE_METERS; // conversion * magic constants
  }

  public double getVelocitySpecific(int id) {
    return encoders.get(id).getVelocity() * Constants.ENCODER_CONSTANTS.GEARING * Constants.ENCODER_CONSTANTS.WHEEM_CIRCUMFRENCE_METERS / 60;
  }

  public Pose2d getPose() {
    return m_odometer.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocitySpecific(0), getVelocitySpecific(1));
  }

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    driveSub.left.setVoltage(leftVolts);
    driveSub.right.setVoltage(rightVolts);
    driveSub.diffDrive.feed();
  }

  @Override
  public void periodic() {
    m_odometer.update(
      navX.getRotation2d(), getPositionSpecific(0), getPositionSpecific(1));
      SmartDashboard.putNumber("curposX", m_odometer.getPoseMeters().getX());
      SmartDashboard.putNumber("curposY", m_odometer.getPoseMeters().getY());
      SmartDashboard.putNumber("PositionSpecific0", getPositionSpecific(0));
      SmartDashboard.putNumber("PositionSpecific1", getPositionSpecific(1));
    }
}
