package frc.sneakylib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
* This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping
* duplicate set commands. (By default the Talon flushes the Tx buffer on every set call).
*/

// TODO: This is not tested yet. Test it!

public class LazyVictorSPX extends VictorSPX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}
