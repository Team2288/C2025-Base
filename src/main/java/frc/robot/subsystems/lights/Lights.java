package frc.robot.subsystems.lights;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;

public class Lights extends SubsystemBase {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs;
    private final LightsSupplier lightsSuppliers[]; // Assumed that coral intake sensor is 1st, algae intake sensor is 2nd, acting is 3rd
    private boolean areSensorsEnabled[];

    public Lights(LightsIO io, LightsSupplier... supplier) {
        this.io = io;
        this.inputs = new LightsIOInputsAutoLogged();
        this.lightsSuppliers = new LightsSupplier[supplier.length+1];
        this.areSensorsEnabled = new boolean[supplier.length+1];

        this.areSensorsEnabled[0] = DriverStation.isEnabled();

        if (supplier.length > 0) {
            for (int i = 0; i < lightsSuppliers.length; i++) { // populate array
                areSensorsEnabled[i+1] = lightsSuppliers[i].supplyLED();
            }
        }

        setDefaultCommand(runPattern(LightStatesEnum.kIdle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lights/", inputs);

        areSensorsEnabled[0] = DriverStation.isEnabled();
        LightStatesEnum usingState;

        if (areSensorsEnabled.length > 1) {
            for (int i = 0; i < lightsSuppliers.length; i++) { // populate array
                areSensorsEnabled[i+1] = lightsSuppliers[i].supplyLED();
            }
        }

        if (areSensorsEnabled[0]) { // if robot is enabled
            try {
                if (areSensorsEnabled[3]) { // if the robot is acting (intaking or scoring)
                    usingState = LightStatesEnum.kActing;
                } else { // if the robot is not acting (driving)
                    usingState = LightStatesEnum.kDriving;
                }
            } catch (Exception e) {
                Logger.recordOutput("Lights/Error2", "Sensor down for robot action checking");
                usingState = LightStatesEnum.kDriving;
            }
        } else { // if its not enabled, we assume its disabled and idle
            usingState = LightStatesEnum.kIdle;
        }

        try {
            boolean hasAlgae = areSensorsEnabled[2];
            boolean hasCoral =  areSensorsEnabled[3];//areSensorsEnabled[1];

            if (hasCoral && hasAlgae) { 
                usingState = LightStatesEnum.kHasBoth;
            } else if (!hasCoral && hasAlgae) { // if robot has algae and not coral
                usingState = LightStatesEnum.kHasAlgae;
            } else if (hasCoral && !hasAlgae) { // if robot has coral and not algae
                usingState = LightStatesEnum.kHasCoral;
            } 
        } catch (Exception e) {
            Logger.recordOutput("Lights/Error", "Sensors down for coral/algae intakes");
        }

        if (inputs.currentState != usingState) {
            runPattern(usingState).schedule();
        }

        io.setLEDData();
    }
    
    public Command runPattern(LightStatesEnum state) {
        return run(
            () -> io.setLEDPattern(state, true)).withTimeout(1.0) // flash for 1 second during a state change
            .andThen(run(() -> io.setLEDPattern(state, false)))
            .ignoringDisable(true);
    }
    
    @FunctionalInterface
    public static interface LightsSupplier {
        public boolean supplyLED();
    }
}
