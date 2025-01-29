package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationSubsystem extends SubsystemBase {


    Mechanism2d mech=new Mechanism2d(50, 50);
   MechanismRoot2d root = mech.getRoot("root",25,0);
   MechanismLigament2d elevator;
   MechanismLigament2d wrist;
   DoubleSupplier elevatorSupplier;
   DoubleSupplier wristSupplier;



   public SimulationSubsystem(DoubleSupplier elevatorHeight, DoubleSupplier wristAngle){
        elevator = root.append(new MechanismLigament2d("elevator", 0, 90, 6,new Color8Bit(Color.kRed)));
        wrist = elevator.append(new MechanismLigament2d("wrist", 2, -90));
        elevatorSupplier=elevatorHeight;
        wristSupplier=wristAngle;


   }

   @Override
   public void periodic() {
    double length = ((27/50.0)*(elevatorSupplier.getAsDouble()))+22;
    elevator.setLength(length);

    //double angle = 8.91*wristSupplier.getAsDouble() - 165.3;
    double angle = 8.91*wristSupplier.getAsDouble() - 165.3;

    wrist.setAngle(angle);
    SmartDashboard.putData("MEch2d",mech);


   }
    
}
