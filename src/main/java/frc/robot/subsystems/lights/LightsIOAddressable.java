package frc.robot.subsystems.lights;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;

public class LightsIOAddressable implements LightsIO {
    private AddressableLED lights;
    public LightStatesEnum usingState;
    private AddressableLEDBuffer buffer;

    public LightsIOAddressable() {
        this.lights = new AddressableLED(LightsConstants.kPWMPort);
        this.buffer = new AddressableLEDBuffer(66);
        this.usingState = LightStatesEnum.kIdle;

        this.lights.setLength(buffer.getLength());
        this.lights.setData(this.buffer);
        this.lights.start();
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.currentState = this.usingState;
    }

    @Override
    public void setLED(LightStatesEnum state) {
        LEDPattern pattern = LightsConstants.LightStates.get(state);

        if (!state.equals(this.usingState)) {
            pattern = pattern.breathe(Seconds.of(1));
        }

        pattern.applyTo(this.buffer);
        this.lights.setData(this.buffer);

        this.usingState = state;
    }
}
 