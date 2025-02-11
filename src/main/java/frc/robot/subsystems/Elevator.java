// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.LambdaMetafactory;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RollerConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

@Logged
public class Elevator extends SubsystemBase {
   private SparkMax elevatorMotor;
   private SparkMaxConfig motorConfig;
   private SparkClosedLoopController closedLoopController;
   private RelativeEncoder encoder;
   private int targetPosition;
   

   
   MutVoltage appliedVoltage = Volts.mutable(0);
   MutDistance distance = Meters.mutable(0);
   MutLinearVelocity vel = MetersPerSecond.mutable(0);

    public SysIdRoutine routine;


  /** Creates a new Elevator. */
  public Elevator() {

    elevatorMotor = new SparkMax(RollerConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    closedLoopController = elevatorMotor.getClosedLoopController();
    encoder = elevatorMotor.getEncoder();

    motorConfig = new SparkMaxConfig();

    // Create Configuration Object
    motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

    // Configure the closed Loop Controller
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-0.5, 0.5);

    elevatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.setDefaultNumber("Elevator/position", 0);
   routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::setVoltage, log->{
        log.motor("elevator")
        .voltage(appliedVoltage.mut_replace(elevatorMotor.getAppliedOutput()*RobotController.getBatteryVoltage(), Volts))
          .linearPosition(distance.mut_replace(encoder.getPosition(),Meters))
          .linearVelocity(vel.mut_replace(encoder.getVelocity(), MetersPerSecond));
      },this));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Elevator/encoderposition",encoder.getPosition());
    SmartDashboard.putNumber("Elevator/busvoltage",RobotController.getBatteryVoltage());

    SmartDashboard.putNumber("Elevator/voltage",elevatorMotor.get());
    SmartDashboard.putNumber("Elevator/appliedoutput",elevatorMotor.getAppliedOutput());


  }

  public void setPosition(double position) {
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public Command getSetPositionCommand(double position) {
    // For Debugging
    
    return this.runOnce(() -> {
      targetPosition = (int)position;
      if (targetPosition < 0) { // Bottom of the Elevator
        targetPosition = 0;
      }
      if (targetPosition > 200) { // Top of Elevator
        targetPosition = 200;
      }
      this.setPosition(targetPosition);
      SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    });
  }

  public Command deltaPositionCommand(int deltaPos) {
    
    return this.runOnce( () -> {
      targetPosition += deltaPos;
      // Limits - Software stop
      if (targetPosition < 0) { // Bottom of the Elevator
        targetPosition = 0;
      }
      if (targetPosition > 200) { // Top of Elevator
        targetPosition = 200;
      }
      this.setPosition(targetPosition);
      SmartDashboard.putNumber("Elevator Position", targetPosition);
    });
    

  }
  public void setVoltage(Voltage voltage){
      elevatorMotor.setVoltage(voltage);
    
  }
  // public void getVoltage(){
  //   return RobotController.getBatteryVoltage()*elevatorMotor.get();
  // }

  public double getPosition(){
    return elevatorMotor.getEncoder().getPosition();
  }

}
