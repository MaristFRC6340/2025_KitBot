package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class P3Elevator extends SubsystemBase{
    
    private SparkMax elevatorMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController loopController;
    private RelativeEncoder encoder;
    private int targetPosition;
    private ProfiledPIDController pid;
    private ElevatorFeedforward elevatorFeedforward;

    public P3Elevator() {

        // init
        elevatorMotor = new SparkMax(RollerConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        loopController = elevatorMotor.getClosedLoopController();
        encoder = elevatorMotor.getEncoder();
        motorConfig = new SparkMaxConfig();

        // config doing config things
        motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        pid = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        elevatorFeedforward = new ElevatorFeedforward(0, 0, 0);

    }

    public void periodic() {

    }

    public Command goToPosition(double position) {
        return this.run(() -> {
            elevatorMotor.set(
                pid.calculate(encoder.getPosition(), position)+elevatorFeedforward.calculate(pid.getSetpoint().velocity)
                // calculates next output of PID based on position of encoder + parameter,
                // adds it to the feedforward based on pid's vel
                );
        });
    }




}
