package frc.robot.subsystems.lights;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;

public class LightsIOAddressable implements LightsIO {
    private AddressableLED lights;
    public LightStatesEnum usingState;
    private AddressableLEDBuffer buffer;

    public LightsIOAddressable() {
        this.lights = new AddressableLED(LightsConstants.kPWMPort);

        this.buffer = new AddressableLEDBuffer(66);

        this.usingState = null;

        this.lights.setLength(buffer.getLength());
        this.lights.setData(this.buffer);
        this.lights.start();
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.currentState = this.usingState;
    }

    public LEDPattern flashPattern(LEDPattern pattern) {
        return pattern.breathe(Seconds.of(0.15));
    }

    @Override
    public void setLEDData() {
        this.lights.setData(this.buffer);
    }

    @Override
    public void setLEDPattern(LightStatesEnum state, boolean isFlashing) {
        LEDPattern pattern = LightsConstants.LightStates.get(state);

        if (isFlashing) pattern = flashPattern(pattern);

        pattern.applyTo(this.buffer);
        this.usingState = state;
    }
}