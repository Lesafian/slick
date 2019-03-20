package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.Constants;

public class PS4 extends Joystick {

	private double threshold = Constants.kJoystickThreshold;

	public PS4(int port) {
		super(port);
	}

	public boolean getSquare() {
		return getRawButton(1);
	}

	public boolean getSquarePressed() {
		return getRawButtonPressed(1);
	}

	public boolean getXButton() {
		return getRawButton(2);
	}

	public boolean getXButtonPressed() {
		return getRawButtonPressed(2);
	}

	public boolean getCircle() {
		return getRawButton(3);
	}

	public boolean getCirclePressed() {
		return getRawButtonPressed(3);
	}

	public boolean getTriangle() {
		return getRawButton(4);
	}

	public boolean getTrianglePressed() {
		return getRawButtonPressed(4);
	}

	public boolean getBumper(Hand hand) {
		if (hand == Hand.kRight) {
			return getRawButton(6);
		} else {
			return getRawButton(5);
		}
	}

	public boolean getBumperPressed(Hand hand) {
		if (hand == Hand.kRight) {
			return getRawButtonPressed(6);
		} else {
			return getRawButtonPressed(5);
		}
	}

	public boolean getOptions() {
		return getRawButton(10);
	}

	public boolean getOptionsPressed() {
		return getRawButtonPressed(10);
	}

	public boolean getShare() {
		return getRawButton(9);
	}

	public boolean getSharePressed() {
		return getRawButtonPressed(9);
	}

	public boolean getPSButton() {
		return getRawButton(13);
	}

	public boolean getPSButtonPressed() {
		return getRawButtonPressed(13);
	}

	public boolean getStickButton(Hand hand) {
		if (hand == Hand.kRight) {
			return getRawButton(12);
		} else {
			return getRawButton(11);
		}
	}

	public boolean getTouchPad() {
		return getRawButton(14);
	}

	public boolean getTouchPadPressed() {
		return getRawButtonPressed(14);
	}

	public boolean getStickButtonPressed(Hand hand) {
		if (hand == Hand.kRight) {
			return getRawButtonPressed(12);
		} else {
			return getRawButtonPressed(11);
		}
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
		} else if (triggerExceedsThreshold(Hand.kRight)) {
			return true;
		} else if (triggerExceedsThreshold(Hand.kLeft)) {
			return true;
		} else {
			return false;
		}
	}

	public double getTriggerAxis(Hand hand) {
		if (hand == Hand.kRight) {
			return getRawAxis(4);
		} else {
			return getRawAxis(3);
		}
	}

	public boolean triggerExceedsThreshold(Hand hand) {
		return Math.abs(getTriggerAxis(hand)) > threshold;
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
