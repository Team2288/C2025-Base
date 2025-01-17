package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.*;

import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.math.filter.Debouncer;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

public class ElevatorIOTalonFX implements ElevatorIO {
    TalonFX krakenLeader;
    TalonFX krakenFollower;
    
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    // Inputs from leader motor
    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Voltage> leaderAppliedVolts;
    private final StatusSignal<Current> leaderCurrent;

    // Inputs from follower motor
    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerCurrent;

    // Connection debouncers
    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOTalonFX() {
        krakenLeader = new TalonFX(ElevatorConstants.kKrakenLeaderPort);
        krakenFollower = new TalonFX(ElevatorConstants.kKrakenFollowerPort);

        krakenFollower.setControl(new Follower(ElevatorConstants.kKrakenLeaderPort, false));
        
        var elevatorConfig = ElevatorConstants.elevatorConfiguration;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.Slot0 = ElevatorConstants.elevatorGains;
        elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorGearRatio;

        tryUntilOk(5, () -> krakenLeader.getConfigurator().apply(elevatorConfig, 0.25));
        tryUntilOk(5, () -> krakenLeader.setPosition(0.0, 0.25));

        // Create leader status signals
        leaderPosition = krakenLeader.getPosition();
        leaderVelocity = krakenLeader.getVelocity();
        leaderAppliedVolts = krakenLeader.getMotorVoltage();
        leaderCurrent = krakenLeader.getStatorCurrent();

        // Create follower status signals
        followerPosition = krakenFollower.getPosition();
        followerVelocity = krakenFollower.getVelocity();
        followerAppliedVolts = krakenFollower.getMotorVoltage();
        followerCurrent = krakenFollower.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderCurrent,
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerCurrent);
        ParentDevice.optimizeBusUtilizationForAll(krakenLeader, krakenFollower);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Refresh all signals
        var leaderStatus =
            BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
        var followerStatus =
            BaseStatusSignal.refreshAll(followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

        // Update leader inputs
        inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
        inputs.leaderPosition = leaderPosition.getValueAsDouble();
        inputs.leaderVelocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
        inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();

        // Update follower inputs
        inputs.followerConnected = followerConnectedDebounce.calculate(followerStatus.isOK());
        inputs.followerPosition = followerPosition.getValueAsDouble();
        inputs.followerVelocityRadPerSec = Units.rotationsToRadians(followerVelocity.getValueAsDouble());
        inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    }

    @Override
    public void setPosition(double position) {
        krakenLeader.setControl(positionVoltageRequest.withPosition(position));
    }
}
