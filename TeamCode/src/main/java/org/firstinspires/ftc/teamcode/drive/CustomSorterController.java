package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomSorterController {
	public enum CustomColor {
		GREEN,
		PURPLE,
		NULL
	}

	int[] lifterState = new int[3];
	ElapsedTime[] lifterTimers = new ElapsedTime[3];
	private final Servo[] lifter = new Servo[3];
	private final ColorSensor[] colorSensor = new ColorSensor[6];
	private CustomRGBController RGBPrism;
	// Cache for colors and ball count
	private volatile CustomColor[] cachedColors = {CustomColor.NULL, CustomColor.NULL, CustomColor.NULL};
	private volatile int cachedBallCount = 0;
	// Lock lifters during launch to prevent balls from moving between pits
	private volatile boolean liftersLocked = false;

	// ===== DEBUG TELEMETRY DATA (updated in background thread) =====
	private volatile int[][] cachedRGB = new int[6][3];
	private volatile float[][] cachedHSV = new float[6][3];

	public CustomSorterController(HardwareMap hardwareMap) {
		lifter[0] = hardwareMap.get(Servo.class, "lifter0");
		lifter[1] = hardwareMap.get(Servo.class, "lifter1");
		lifter[2] = hardwareMap.get(Servo.class, "lifter2");
		colorSensor[0] = hardwareMap.get(ColorSensor.class, "colorSensor0");
		colorSensor[1] = hardwareMap.get(ColorSensor.class, "colorSensor0-1");
		colorSensor[2] = hardwareMap.get(ColorSensor.class, "colorSensor1");
		colorSensor[3] = hardwareMap.get(ColorSensor.class, "colorSensor1-1");
		colorSensor[4] = hardwareMap.get(ColorSensor.class, "colorSensor2");
		colorSensor[5] = hardwareMap.get(ColorSensor.class, "colorSensor2-1");
		RGBPrism = new CustomRGBController(hardwareMap, 6);
		lifterTimers[0] = new ElapsedTime();
		lifterTimers[1] = new ElapsedTime();
		lifterTimers[2] = new ElapsedTime();

		// Initialize lifters to low position
		int[] lifterMapping = MDOConstants.LifterPitMapping;
		boolean[] reverseMap = MDOConstants.LifterReverseMap;
		for (int i = 0; i < 3; i++) {
			int lifterIndex = lifterMapping[i];
			lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
		}
	}

	/**
	 * Updates the cached color values from sensors.
	 * This involves I2C reads and should be called from a separate thread.
	 */
	public void updateSensors() {
		int ballCount = 0;
		CustomColor[] newColors = new CustomColor[3];

		int[][] newRGB = new int[6][3];
		float[][] newHSV = new float[6][3];

		for (int s = 0; s < 6; s++) {
			int argb = colorSensor[s].argb();
			int red = (argb >> 16) & 0xFF;
			int green = (argb >> 8) & 0xFF;
			int blue = argb & 0xFF;
			newRGB[s] = new int[]{red, green, blue};
			newHSV[s] = rgbToHsv(red, green, blue);
		}
		cachedRGB = newRGB;
		cachedHSV = newHSV;

		for (int i = 0; i < 3; i++) {
			int sensorIndex0, sensorIndex1;
			int[] mapping = MDOConstants.ColorSensorPitMapping;

			switch (i) {
				case 0:
					sensorIndex0 = mapping[0];
					sensorIndex1 = mapping[1];
					break;
				case 1:
					sensorIndex0 = mapping[2];
					sensorIndex1 = mapping[3];
					break;
				case 2:
					sensorIndex0 = mapping[4];
					sensorIndex1 = mapping[5];
					break;
				default:
					continue;
			}

			int colorARGB0 = colorSensor[sensorIndex0].argb();
			CustomColor color = calcColor(colorARGB0);

			if (color == CustomColor.NULL) {
				int colorARGB1 = colorSensor[sensorIndex1].argb();
				color = calcColor(colorARGB1);
			}

			newColors[i] = color;
			if (color != CustomColor.NULL) {
				ballCount++;
			}
		}

		cachedColors = newColors;
		cachedBallCount = ballCount;
	}

	public CustomColor getCachedColor(int pitSelector) {
		if (pitSelector >= 0 && pitSelector < 3) {
			return cachedColors[pitSelector];
		}
		return CustomColor.NULL;
	}

	public int getCachedBallCount() {
		return cachedBallCount;
	}

	public int[] getCachedRGB(int sensorIndex) {
		if (sensorIndex >= 0 && sensorIndex < 6) {
			return cachedRGB[sensorIndex];
		}
		return new int[]{0, 0, 0};
	}

	public float[] getCachedHSV(int sensorIndex) {
		if (sensorIndex >= 0 && sensorIndex < 6) {
			return cachedHSV[sensorIndex];
		}
		return new float[]{0, 0, 0};
	}

	public String getDebugString(int sensorIndex) {
		int[] rgb = getCachedRGB(sensorIndex);
		float[] hsv = getCachedHSV(sensorIndex);
		int rawSum = rgb[0] + rgb[1] + rgb[2];
		return String.format("RGB(%d,%d,%d) Sum:%d H:%.0f S:%.2f V:%.2f",
				rgb[0], rgb[1], rgb[2], rawSum, hsv[0], hsv[1], hsv[2]);
	}

	public CustomColor getDirectColor(int pitSelector) {
		return getColor(pitSelector);
	}

	public CustomColor getColor(int pitSelector) {
		int sensorIndex0, sensorIndex1;
		int[] mapping = MDOConstants.ColorSensorPitMapping;

		switch (pitSelector) {
			case 0:
				sensorIndex0 = mapping[0];
				sensorIndex1 = mapping[1];
				break;
			case 1:
				sensorIndex0 = mapping[2];
				sensorIndex1 = mapping[3];
				break;
			case 2:
				sensorIndex0 = mapping[4];
				sensorIndex1 = mapping[5];
				break;
			default:
				throw new IllegalArgumentException("Invalid pit selector: " + pitSelector);
		}

		int colorARGB0 = colorSensor[sensorIndex0].argb();
		CustomColor color0 = calcColor(colorARGB0);
		if (color0 != CustomColor.NULL) {
			return color0;
		} else {
			int colorARGB1 = colorSensor[sensorIndex1].argb();
			return calcColor(colorARGB1);
		}
	}

	public CustomColor getSensorColor(int sensorIndex) {
		if (sensorIndex < 0 || sensorIndex >= 6) {
			throw new IllegalArgumentException("Invalid sensor index: " + sensorIndex);
		}
		int colorARGB = colorSensor[sensorIndex].argb();
		return calcColor(colorARGB);
	}

	public int[] getSensorRGB(int sensorIndex) {
		if (sensorIndex < 0 || sensorIndex >= 6) {
			throw new IllegalArgumentException("Invalid sensor index: " + sensorIndex);
		}
		int argb = colorSensor[sensorIndex].argb();
		int red = (argb >> 16) & 0xFF;
		int green = (argb >> 8) & 0xFF;
		int blue = argb & 0xFF;
		return new int[]{red, green, blue};
	}

	public void launch(CustomColor color) {
		boolean isLaunched = false;
		int[] slotPriority = {2, 1, 0};
		if (color == CustomColor.NULL) {
			for (int i : slotPriority) {
				if (lifterState[i] == 0 && getColor(i) != CustomColor.NULL && !isLaunched) {
					isLaunched = true;
					lifterState[i] = 1;
				}
			}
		} else {
			for (int i : slotPriority) {
				if (lifterState[i] == 0 && getColor(i) == color && !isLaunched) {
					isLaunched = true;
					lifterState[i] = 1;
				}
			}
			if (!isLaunched) {
				for (int i : slotPriority) {
					if (lifterState[i] == 0 && getColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			}
		}
	}

	public boolean launchCached(CustomColor color) {
		boolean isLaunched = false;
		int[] slotPriority = {2, 1, 0};
		if (color == CustomColor.NULL) {
			for (int i : slotPriority) {
				if (lifterState[i] == 0 && getCachedColor(i) != CustomColor.NULL && !isLaunched) {
					isLaunched = true;
					lifterState[i] = 1;
				}
			}
		} else {
			for (int i : slotPriority) {
				if (lifterState[i] == 0 && getCachedColor(i) == color && !isLaunched) {
					isLaunched = true;
					lifterState[i] = 1;
				}
			}
			if (!isLaunched) {
				for (int i : slotPriority) {
					if (lifterState[i] == 0 && getCachedColor(i) != CustomColor.NULL && !isLaunched) {
						isLaunched = true;
						lifterState[i] = 1;
					}
				}
			}
		}
		return isLaunched;
	}

	public boolean launchCachedStrict(CustomColor color) {
		if (color == CustomColor.NULL) {
			return false;
		}

		boolean isLaunched = false;
		int[] slotPriority = {2, 1, 0};

		for (int i : slotPriority) {
			if (lifterState[i] == 0 && getCachedColor(i) == color && !isLaunched) {
				isLaunched = true;
				lifterState[i] = 1;
			}
		}

		return isLaunched;
	}

	public void lockLiftersForLaunch(boolean lock) {
		liftersLocked = lock;
		if (lock) {
			int[] lifterMapping = MDOConstants.LifterPitMapping;
			boolean[] reverseMap = MDOConstants.LifterReverseMap;
			for (int i = 0; i < 3; i++) {
				int lifterIndex = lifterMapping[i];
				if (lifterState[i] == 0) {
					lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
				}
			}
		}
	}

	public boolean areLiftersLocked() {
		return liftersLocked;
	}

	public void forceLaunchSlot(int slot) {
		if (slot >= 0 && slot < 3) {
			lifterState[slot] = 1;
		}
	}

	public boolean launchFromPit(int pitIndex) {
		if (pitIndex < 0 || pitIndex >= 3) {
			return false;
		}
		if (lifterState[pitIndex] == 0) {
			lifterState[pitIndex] = 1;
			return true;
		}
		return false;
	}

	public int getSlotCount() {
		return 3;
	}

	public void lifterUpdater() {
		int[] lifterMapping = MDOConstants.LifterPitMapping;
		boolean[] reverseMap = MDOConstants.LifterReverseMap;
		for (int i = 0; i < 3; i++) {
			int lifterIndex = lifterMapping[i];
			if (lifterState[i] == 1) {
				lifterState[i] = 2;
				lifterTimers[i].reset();
				lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionHigh : MDOConstants.LifterPositionHigh);
			} else if (lifterState[i] == 2 && lifterTimers[i].milliseconds() > MDOConstants.LifterWaitToTopTimerMillis) {
				lifterState[i] = 0;
				lifter[lifterIndex].setPosition(reverseMap[lifterIndex] ? 1 - MDOConstants.LifterPositionLow : MDOConstants.LifterPositionLow);
			}
		}
	}

	public void lightingUpdater() {
		for (int i = 0; i < 3; i++) {
			CustomColor color = getCachedColor(i);
			switch (color) {
				case GREEN:
					break;
				case PURPLE:
					break;
				case NULL:
					break;
			}
		}
	}

	private CustomColor calcColor(int argb) {
		int red = (argb >> 16) & 0xFF;
		int green = (argb >> 8) & 0xFF;
		int blue = argb & 0xFF;

		int rawSum = red + green + blue;
		if (rawSum < MDOConstants.ColorMinRawSum) {
			return CustomColor.NULL;
		}

		float[] hsv = rgbToHsv(red, green, blue);
		float hue = hsv[0];
		float saturation = hsv[1];
		float value = hsv[2];

		float minBrightness = (float) MDOConstants.ColorMinBrightness;
		float minSaturation = (float) MDOConstants.ColorMinSaturation;
		float greenHueMin = (float) MDOConstants.GreenHueMin;
		float greenHueMax = (float) MDOConstants.GreenHueMax;
		float purpleHueMin = (float) MDOConstants.PurpleHueMin;
		float purpleHueMax = (float) MDOConstants.PurpleHueMax;

		if (minBrightness > 0 && value < minBrightness) {
			return CustomColor.NULL;
		}

		if (saturation < minSaturation) {
			return CustomColor.NULL;
		}

		if (hue >= greenHueMin && hue <= greenHueMax) {
			return CustomColor.GREEN;
		}

		if (hue >= purpleHueMin && hue <= purpleHueMax) {
			return CustomColor.PURPLE;
		}

		return CustomColor.NULL;
	}

	private float[] rgbToHsv(int r, int g, int b) {
		float rf = r / 255f;
		float gf = g / 255f;
		float bf = b / 255f;

		float max = Math.max(rf, Math.max(gf, bf));
		float min = Math.min(rf, Math.min(gf, bf));
		float delta = max - min;

		float hue = 0;
		float saturation = (max == 0) ? 0 : (delta / max);
		float value = max;

		if (delta != 0) {
			if (max == rf) {
				hue = 60 * (((gf - bf) / delta) % 6);
			} else if (max == gf) {
				hue = 60 * (((bf - rf) / delta) + 2);
			} else {
				hue = 60 * (((rf - gf) / delta) + 4);
			}
		}

		if (hue < 0) {
			hue += 360;
		}

		return new float[]{hue, saturation, value};
	}
}

