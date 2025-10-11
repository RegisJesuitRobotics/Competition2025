package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import sun.misc.Signal;

import java.util.function.DoubleSupplier;

public class Algae extends SubsystemBase {
    private final TelemetryTalonFX algaeMotor = 
        new TelemetryTalonFX(Constants.AlgaeConstants.ID, "/algae/motor", Constants.MiscConstants.CANIVORE_NAME, Constants.MiscConstants.TUNING_MODE);
    private final SysIdRoutine algaeSysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(6), null),
        new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
    private final TunableTelemetryProfiledPIDController algaepid = 
        new TunableTelemetryProfiledPIDController("algae/profiledpid", Constants.AlgaeConstants.ALGAE_PID_GAINS, Constants.AlgaeConstants.ALGAE_TRAP_GAINS);
    
}