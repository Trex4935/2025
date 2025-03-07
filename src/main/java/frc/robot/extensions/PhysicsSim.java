package frc.robot.extensions;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import java.util.ArrayList;

/** Manages physics simulation for CTRE products. */
public class PhysicsSim {
  private static final PhysicsSim sim = new PhysicsSim();

  /** Gets the robot simulator instance. */
  public static PhysicsSim getInstance() {
    return sim;
  }

  /**
   * Adds a TalonFX controller to the simulator.
   *
   * @param talonFX The TalonFX device
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public void addTalonFX(TalonFX talonFX, final double rotorInertia) {
    if (talonFX != null) {
      TalonFXSimProfile simTalonFX = new TalonFXSimProfile(talonFX, rotorInertia);
      _simProfiles.add(simTalonFX);
    }
  }

  /**
   * Adds a TalonFXS controller to the simulator.
   *
   * @param talonFXS The TalonFX device
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public void addTalonFXS(TalonFXS talonFXS, final double rotorInertia) {
    if (talonFXS != null) {
      TalonFXSSimProfile simTalonFXS = new TalonFXSSimProfile(talonFXS, rotorInertia);
      _simProfiles.add(simTalonFXS);
    }
  }

  /** Runs the simulator: - enable the robot - simulate sensors */
  public void run() {
    // Simulate devices
    for (SimProfile simProfile : _simProfiles) {
      simProfile.run();
    }
  }

  private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

  /** Holds information about a simulated device. */
  static class SimProfile {
    private double _lastTime;
    private boolean _running = false;

    /** Runs the simulation profile. Implemented by device-specific profiles. */
    public void run() {}

    /** Returns the time since last call, in seconds. */
    protected double getPeriod() {
      // set the start time if not yet running
      if (!_running) {
        _lastTime = Utils.getCurrentTimeSeconds();
        _running = true;
      }

      double now = Utils.getCurrentTimeSeconds();
      final double period = now - _lastTime;
      _lastTime = now;

      return period;
    }
  }
}
