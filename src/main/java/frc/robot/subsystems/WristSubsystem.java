// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Source: REV github Closed Loop Control

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class WristSubsystem extends SubsystemBase {
  private SparkMax wristMotor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private int targetPosition;


  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
  
    wristMotor = new SparkMax(RollerConstants.WRIST_MOTOR_ID, MotorType.kBrushless);

    closedLoopController = wristMotor.getClosedLoopController();
    encoder = wristMotor.getEncoder();

    // Create Configuration Object
    motorConfig = new SparkMaxConfig();



    // Configure the encoder - not needed but here as example
    /* 
    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    */

    motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

    // Configure the closed Loop Control (PID)
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for Position Control
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-0.75, .75);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Numbers for Smart Dashboard
    SmartDashboard.setDefaultNumber("Target Position", 0);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double position) {
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command getSetPositionCommand(double position) {
    // For Debugging
    
    return this.runOnce(() -> {
      this.setPosition(position);
      SmartDashboard.putNumber("TargetPosition", encoder.getPosition());
    });
  }

  public Command deltaPositionCommand(int deltaPos) {
    // For Debugging
    
    return this.runOnce(() -> {
      targetPosition += deltaPos;
      this.setPosition(targetPosition);
      SmartDashboard.putNumber("TargetPosition", encoder.getPosition());
    });
  }

}
