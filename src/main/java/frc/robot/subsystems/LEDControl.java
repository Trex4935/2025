package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDControl extends SubsystemBase {

  // Declaring ledstrip
  int m_rainbowFirstPixelHue;
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  // Declares a counter and a toggle to be used in flashing the LEDs
  boolean ledToggle;
  int counter, ledBufferLength;
  private int walkStartingPosition = 0;

  /** Creates a new LEDControl. */
  public LEDControl() {

    // Creating addressable led Objects
    m_rainbowFirstPixelHue = 0;
    ledStrip = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(180);
    ledBufferLength = ledBuffer.getLength();
    ledStrip.setLength(ledBuffer.getLength());
    // Makes the counter and toggle
    ledToggle = false;
    counter = 0;
  }

  // Used to create rainbow LEDs
  public void LEDController() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 32);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  // Used to create a 'walking' LED
  public void sectionedLEDControl() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, Color.kBlue);
      if (i < ledBuffer.getLength() - 1) {
        // System.out.println("+1: "+(i+1));
        ledBuffer.setLED(i + 1, Color.kBlack);
      }
      if (i >= 1) {
        // System.out.println("-1: " + (i - 1));
        ledBuffer.setLED(i - 1, Color.kBlack);
      }
      if (i == 0) {
        ledBuffer.setLED(89, Color.kBlack);
      }

      Timer.delay(.05);
      ledStrip.setData(ledBuffer);
      ledStrip.start();
    }
  }

  // A method used to create non changing LEDs or solid LEDs
  public void solidLEDS(int hue, int saturation, int brightness) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // Set the value
      ledBuffer.setHSV(i, hue, saturation, brightness);
    }
    // Increase by to make the rainbow "move"
    // Check bounds
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  // A method used to create flashing LEDs taking blink rate into account
  public void flashLEDS(int hue, int saturation, int brightness, int blinkRate) {
    counter++;
    // Checks to see how long one cycle has passed
    if (counter % blinkRate == 0) {
      // Checks to see if LEDs are off and turns them on
      if (!ledToggle) {
        solidLEDS(hue, saturation, brightness);
        ledToggle = true;
      }
      // Turns off the LEDs if they are on
      else {
        solidLEDS(0, 0, 0);
        ledToggle = false;
      }
    }
  }

  /**
   * @param pirmaryColor Primary color of LED strip e.g Color.kRed
   * @param secondaryColor Secondary color of LED strip e.g. Color.kBlack
   * @param startPosition Initial position of Primary color
   * @param lengthOfPrimary Number of LEDs to set to Primary Color
   */
  public void setLEDSectionOn(
      Color pirmaryColor, Color secondaryColor, int startPosition, int lengthOfPrimary) {

    // set all of the leds to the secondary color
    for (int i = 0; i < ledBufferLength; i++) {
      ledBuffer.setLED(i, secondaryColor);
    }

    // starting at the start position set lengthOfPrimary leds to the primary color
    for (int j = 0; j < lengthOfPrimary; j++) {

      // detect if we are at the end of the strip and reset the start of the strip
      if (startPosition >= ledBufferLength) {
        startPosition = 0;
      }

      ledBuffer.setLED(startPosition, pirmaryColor);
      startPosition++;
    }
  }

  /**
   * @param primaryColor Color that will be moving e.g. Color.kRed
   * @param secondaryColor Color that will be the background e.g. Color.kBlack
   * @param lengthOfPrimary Length of moving strip.
   */
  public void setLEDsWalking(Color primaryColor, Color secondaryColor, int lengthOfPrimary) {

    // Set up the strip based on our recorded starting position
    setLEDSectionOn(primaryColor, secondaryColor, walkStartingPosition, lengthOfPrimary);

    // Increment starting position
    walkStartingPosition++;

    // Make sure starting position isn't beyond the length of the strip
    walkStartingPosition = walkStartingPosition % ledBufferLength;
  }

  @Override
  public void periodic() {}