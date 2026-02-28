package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class CustomRGBController {
	GoBildaPrismDriver prism;

	public CustomRGBController(HardwareMap hardwareMap, int stripLength) {
		prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
		prism.setStripLength(stripLength);
	}

	public void setSolidColor(int startingPixel, int endingPixel, int red, int green, int blue) {
		if (startingPixel < 0 || endingPixel >= prism.getNumberOfLEDs() || startingPixel > endingPixel) {
			throw new IllegalArgumentException("Invalid pixel range: " + startingPixel + " to " + endingPixel);
		}
		PrismAnimations.Solid solid = new PrismAnimations.Solid(new org.firstinspires.ftc.teamcode.Prism.Color(red, green, blue));
		solid.setStartIndex(startingPixel);
		solid.setStopIndex(endingPixel);
		solid.setBrightness(red + green + blue / 3);
		prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
	}

	public void setThird(int third, int[] rgb) {
		if (rgb.length != 3) return;
		PrismAnimations.Solid solid = new PrismAnimations.Solid(new org.firstinspires.ftc.teamcode.Prism.Color(rgb[0], rgb[1], rgb[2]));
		solid.setStartIndex(prism.getNumberOfLEDs() / third * (third - 1));
		solid.setStopIndex(prism.getNumberOfLEDs() / third);
		solid.setBrightness((rgb[0] + rgb[1] + rgb[2]) / 3);
		prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
	}
}

