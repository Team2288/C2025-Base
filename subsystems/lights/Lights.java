package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;

public class Lights extends SubsystemBase {
    private final LightsIO io;
    private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();
    private LightsSupplier lightsSuppliers[]; // Assumed that acting is 1st
    private boolean areSensorsEnabled[];
    

    public Lights(LightsIO io, LightsSupplier... supplier) {
        this.io = io;
        this.lightsSuppliers = new LightsSupplier[supplier.length];
        this.areSensorsEnabled = new boolean[supplier.length+1];

        this.areSensorsEnabled[0] = DriverStation.isEnabled();

        for (int i = 0; i < supplier.length; i++) { // populate array
            lightsSuppliers[i] = supplier[i];
        }
        setDefaultCommand(runPattern(LightStatesEnum.kIdle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lights/", inputs);

        populateLightSuppliers();
        areSensorsEnabled[0] = DriverStation.isEnabled();
        LightStatesEnum usingState;

        if (areSensorsEnabled[0]) { // if robot is enabled
            try {
                if (areSensorsEnabled[1]) { // if the robot is acting (intaking or scoring)
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
            boolean hasAlgae = areSensorsEnabled[3];
            boolean hasCoral =  areSensorsEnabled[2];

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

        if (DriverStation.isEStopped()) {
            usingState = LightStatesEnum.kStopped;
        }

        if (inputs.currentState != usingState) {
            if (usingState == LightStatesEnum.kDriving) {
                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                    runColor(LEDPattern.steps(
                        Map.of(0, 
                        Color.kDarkViolet,
                        0.5, 
                        Color.kRed
                        )).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1 / 60.0))).schedule();
                } else {
                    runColor(LEDPattern.steps(
                        Map.of(0, 
                        Color.kDarkViolet,
                        0.5, 
                        Color.kBlue
                        )).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1 / 60.0))).schedule();
                }
            } else {
                runPattern(usingState).schedule();
            }
        }

        io.setLEDData();
    }
    
    public Command runPattern(LightStatesEnum state) {
        return run(
            () -> io.setLEDPattern(state, true)).withTimeout(1.0) // flash for 1 second during a state change
            .andThen(run(() -> io.setLEDPattern(state, false)))
            .ignoringDisable(true);
    }

    public Command runColor(LEDPattern color) {
        return run(
            () -> io.setLEDColor(color, true)).withTimeout(1.0) // flash for 1 second during a state change
            .andThen(run(() -> io.setLEDColor(color, false)))
            .ignoringDisable(true);
    }


    public void populateLightSuppliers(){
        for (int i = 0; i < lightsSuppliers.length; i++) { // populate array
            areSensorsEnabled[i+1] = lightsSuppliers[i].supplyLED();
        }
    }
    
    @FunctionalInterface
    public static interface LightsSupplier {
        public boolean supplyLED();
    }
}
