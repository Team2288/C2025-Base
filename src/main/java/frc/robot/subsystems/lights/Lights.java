package frc.robot.subsystems.lights;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.subsystems.lights.LightsConstants.*;

public class Lights extends SubsystemBase {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs;
    private final Alert disconnectedAlert;
    private final LightsSupplier lightsSuppliers[]; // Assumed that coral intake sensor is 1st, algae intake sensor is 2nd
    private final boolean areSensorsEnabled[];

    public Lights(LightsIO io, LightsSupplier... supplier) {
        this.io = io;
        this.inputs = new LightsIOInputsAutoLogged();
        this.disconnectedAlert = new Alert("REV Blinkin is disconnected.", AlertType.kWarning);

        this.lightsSuppliers = new LightsSupplier[supplier.length];
        this.areSensorsEnabled = new boolean[supplier.length];
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lights/", inputs);

        disconnectedAlert.set(!inputs.connected);
        

        // poll whether these are enabled
        for (int i = 0; i < lightsSuppliers.length; i++) {
            areSensorsEnabled[i] = lightsSuppliers[i].getSensor();
        }

        if (areSensorsEnabled[0] && areSensorsEnabled[1]) { // if robot has both algae and coral
            io.setLED(LightStatesEnum.kHasBoth);
        } else if (!areSensorsEnabled[0] && areSensorsEnabled[1]) { // if robot has algae and not coral
            io.setLED(LightStatesEnum.kHasAlgae);
        } else if (areSensorsEnabled[0] && !areSensorsEnabled[1]) { // if robot has coral and not algae
            io.setLED(LightStatesEnum.kHasCoral);
        } else { // if robot has neither algae nor coral
            if (DriverStation.isEnabled()) { // if robot is enabled we assume it's driving
                io.setLED(LightStatesEnum.kDriving);
            } else { // if its not enabled, we assume its disabled and idle
                io.setLED(LightStatesEnum.kIdle);
            }
        }


    }

    @FunctionalInterface
    public static interface LightsSupplier {
        public boolean getSensor();
    }
}
