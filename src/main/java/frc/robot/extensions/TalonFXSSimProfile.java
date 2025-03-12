package frc.robot.extensions;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.extensions.PhysicsSim.SimProfile;

/** Holds information about a simulated TalonFXS. */
class TalonFXSSimProfile extends SimProfile {
  private static final double kMotorResistance =
      0.002; // Assume 2mOhm resistance for voltage drop calculation

  private final DCMotorSim _motorSim;
  private final TalonFXSSimState _talonFXSSim;

  /**
   * Creates a new simulation profile for a TalonFXS device.
   *
   * @param talonFXS The TalonFXS device
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public TalonFXSSimProfile(final TalonFXS talonFXS, final double rotorInertia) {
    var gearbox = DCMotor.getNeo550(1);
    this._motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
    this._talonFXSSim = talonFXS.getSimState();
  }

  /**
   * Runs the simulation profile.
   *
   * <p>This uses very rudimentary physics simulation and exists to allow users to test features of
   * our products in simulation using our examples out of the box. Users may modify this to utilize
   * more accurate physics simulation.
   */
  public void run() {
    /// DEVICE SPEED SIMULATION

    _motorSim.setInputVoltage(_talonFXSSim.getMotorVoltage());

    _motorSim.update(getPeriod());

    /// SET SIM PHYSICS INPUTS
    final double position_rot = _motorSim.getAngularPositionRotations();
    final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

    _talonFXSSim.setRawRotorPosition(position_rot);
    _talonFXSSim.setRotorVelocity(velocity_rps);

    _talonFXSSim.setSupplyVoltage(12 - _talonFXSSim.getSupplyCurrent() * kMotorResistance);
  }
}
