package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static final AddressableLED led = new AddressableLED(LEDConstants.PWM_OUTPUT);
    private static final AddressableLEDBuffer masterBuffer = new AddressableLEDBuffer(LEDConstants.TOTAL_LEDS);

    private static final AddressableLEDBufferView[] buffers = new AddressableLEDBufferView[4];

    private static LEDPattern currentPattern;

    private static LEDPattern waveRainbow;
    private static LEDPattern waveRed;
    private static LEDPattern waveBlue;
    private static LEDPattern wavePurple;

    private static LEDPattern flashRed;
    private static LEDPattern flashBlue;
    private static LEDPattern flashPurple;

    private LEDType LEDState = LEDType.waveRainbow;
    private boolean stateChanged = true;

    public LEDSubsystem() {
        led.setLength(LEDConstants.TOTAL_LEDS);

        waveRainbow = LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(LEDConstants.SCROLL_SPEED, LEDConstants.LEDS_PER_METER);

        waveRed = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,LEDConstants.COLOR_RED, LEDConstants.COLOR_RED_LIGHTEN)
        .scrollAtAbsoluteSpeed(LEDConstants.SCROLL_SPEED, LEDConstants.LEDS_PER_METER);
        waveBlue = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,LEDConstants.COLOR_BLUE, LEDConstants.COLOR_BLUE_LIGHTEN)
        .scrollAtAbsoluteSpeed(LEDConstants.SCROLL_SPEED, LEDConstants.LEDS_PER_METER);
        wavePurple = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,LEDConstants.COLOR_PURPLE, LEDConstants.COLOR_PURPLE_LIGHTEN)
        .scrollAtAbsoluteSpeed(LEDConstants.SCROLL_SPEED, LEDConstants.LEDS_PER_METER);

        flashRed = LEDPattern.solid(LEDConstants.COLOR_RED)
        .breathe(LEDConstants.BLINK_PERIOD);
        flashBlue = LEDPattern.solid(LEDConstants.COLOR_BLUE)
        .breathe(LEDConstants.BLINK_PERIOD);
        flashPurple = LEDPattern.solid(LEDConstants.COLOR_PURPLE)
        .breathe(LEDConstants.BLINK_PERIOD);

        for (var i = 0; i < buffers.length; i++) {

            var buffer = masterBuffer.createView(LEDConstants.LEDS_PER_STRIP * i,
                    LEDConstants.LEDS_PER_STRIP * (i + 1) - 1);
            if (i % 2 == 0)
                buffer = buffer.reversed();
            buffers[i] = buffer;
        }
    }

    public void startAll() {
        led.start();
    }

    public void stopAll() {
        led.stop();
    }

    public void changeLED(LEDType state) {
        LEDState = state;
        stateChanged = true;
    }

    public Command startAllCommand() {
        return runOnce(this::startAll);
    }

    public Command stopAllCommand() {
        return runOnce(this::stopAll);
    }

    public Command changeLEDCommand(LEDType state) {
        return runOnce(() -> changeLED(state));
    }

    private void updateState() {

        if (stateChanged) {
            stateChanged = false;
            if (LEDState == LEDType.waveRainbow) {
                currentPattern = waveRainbow;
            }else if (LEDState == LEDType.waveRed) {
                currentPattern = waveRed;
            }else if (LEDState == LEDType.waveBlue) {
                currentPattern = waveBlue;
            }else if (LEDState == LEDType.wavePurple) {
                currentPattern = wavePurple;
            }else if (LEDState == LEDType.flashRed) {
                currentPattern = flashRed;
            }else if (LEDState == LEDType.flashBlue) {
                currentPattern = flashBlue;
            }else if (LEDState == LEDType.flashPurple) {
                currentPattern = flashPurple;
            }else{
                currentPattern = LEDPattern.solid(Color.kBlack);
            }
        }

        for(var buffer : buffers) {
            currentPattern.applyTo(buffer);
        }
        led.setData(masterBuffer);
    }

    public void periodic() {

        updateState();

    }
}