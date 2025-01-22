// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RollerConstants;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final SparkMax feedMotorTop;
  private final SparkMax feedMotorBottom;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    feedMotorTop = new SparkMax(RollerConstants.UPPER_MOTOR_ID, MotorType.kBrushed);
    feedMotorBottom = new SparkMax(RollerConstants.LOWER_MOTOR_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    feedMotorTop.setCANTimeout(250);
    feedMotorBottom.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    //SparkMaxConfig rollerConfig = new SparkMaxConfig();
    //rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    //rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    //rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  public void runIntakeMotors(double forward) {
      feedMotorTop.set(forward);
      feedMotorBottom.set(forward);
  }

  // Command to run the roller with joystick inputs
  public Command runRoller(
      CANRollerSubsystem rollerSubsystem, double forward) {
    return Commands.run(
        () -> runIntakeMotors(forward), rollerSubsystem);
        
  }

  public Command getSetSpeedCommand(double speed) {
    return this.startEnd(() -> {
      this.runIntakeMotors(speed);
    }, () -> {
       this.runIntakeMotors(0);
    });
  }

  public Command getSetSpeedCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      this.runIntakeMotors(powerSupplier.getAsDouble());
    }, () -> {
       this.runIntakeMotors(0);
    });
  }

}

  

