package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Climber extends VictorSPX {

    public Climber(int master) {
        super(master);
        super.setNeutralMode(NeutralMode.Brake);
    }

    public void climb(double speed) {
        super.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        super.set(ControlMode.PercentOutput, 0);
    }

}