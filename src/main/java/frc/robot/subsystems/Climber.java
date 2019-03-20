package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.constants.CAN;
import frc.robot.constants.Pneumatics;

public class Climber extends VictorSPX {

    private DoubleSolenoid front, back;

    public Climber(int master) {
        super(master);
        super.setNeutralMode(NeutralMode.Brake);

        front = new DoubleSolenoid(CAN.kPCM_24, Pneumatics.kClimbFrontForward, Pneumatics.kClimbFrontReverse);
        back = new DoubleSolenoid(CAN.kPCM_24, Pneumatics.kClimbBackForward, Pneumatics.kClimbBackReverse);
    }

    public void setFront(DoubleSolenoid.Value value) {
        front.set(value);
    }

    public void setBack(DoubleSolenoid.Value value) {
        back.set(value);
    }



    public void climb(double speed) {
        super.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        super.set(ControlMode.PercentOutput, 0);
    }

}