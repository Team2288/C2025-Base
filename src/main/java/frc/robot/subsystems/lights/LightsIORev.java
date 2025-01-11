package frc.robot.subsystems.lights;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;

public class LightsIORev implements LightsIO {
    private final Debouncer isConnectedDebouncer = new Debouncer(0.5);
    private Spark lightsBlinkin;
    private LightStatesEnum usingState;

    public LightsIORev() {
        this.lightsBlinkin = new Spark(LightsConstants.kBlinkinPWMPort);
        this.usingState = LightStatesEnum.kIdle;
    }

    @Override
    public void updateInputs(LightsIOInputs inputs) {
        inputs.connected = isConnectedDebouncer.calculate(lightsBlinkin.isAlive());
        inputs.currentState = this.usingState;
    }

    @Override
    public void setLED(LightStatesEnum state) {
        this.usingState = state;
        double power = LightsConstants.LightStates.get(state);
        this.lightsBlinkin.set(power);
    }
}
