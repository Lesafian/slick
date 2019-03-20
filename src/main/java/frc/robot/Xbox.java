package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.Constants;

public class Xbox extends XboxController {
	
	private double threshold = Constants.kJoystickThreshold;

	public Xbox(int port) {
		super(port);
	}
	
	public void setThreshold(double threshold) {
		this.threshold = threshold;
	}
	
	public double getThreshold() {
		return threshold;
	}
	
	public boolean exceedsThreshold() {
		if (Math.abs(getX(Hand.kLeft)) > threshold) {
			return true;
		} else if (Math.abs(getX(Hand.kRight)) > threshold) {
			return true;
		} else if (Math.abs(getY(Hand.kLeft)) > threshold) {
			return true;
		} else if (Math.abs(getY(Hand.kRight)) > threshold) {
			return true;
		} else if (getTriggerAxis(Hand.kLeft) > threshold) {
			return true;
		} else if (getTriggerAxis(Hand.kRight) > threshold) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean triggerExceedsThreshold(Hand hand) {
		if (getTriggerAxis(hand) > threshold) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean xAxisExceedsThreshold(Hand hand) {
		if (Math.abs(getX(hand)) > threshold) {
			return true;
		} else {
			return false;
		}
	}

	public boolean yAxisExceedsThreshold(Hand hand) {
		if (Math.abs(getY(hand)) > threshold) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean joysticksExceedThreshold() {
		if (Math.abs(getX(Hand.kLeft)) > threshold) {
			return true;
		} else if (Math.abs(getX(Hand.kRight)) > threshold) {
			return true;
		} else if (Math.abs(getY(Hand.kLeft)) > threshold) {
			return true;
		} else if (Math.abs(getY(Hand.kRight)) > threshold) {
			return true;
		} else {
			return false;
		}
    }

}
