package com.gos.crescendo2024;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import static com.gos.crescendo2024.subsystems.ShooterSubsystem.MAX_SHOOTER_RPM;
import static com.gos.crescendo2024.subsystems.ShooterSubsystem.TARMAC_EDGE_RPM_HIGH;

public class SpeakerLookupTable
{
    private final InterpolatingTreeMap<Double, ShootingPair> m_lookupTable;


    public SpeakerLookupTable()
    {
        m_lookupTable = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), new ShootingPairInterpolator());
        m_lookupTable.put(Units.inchesToMeters(12), new ShootingPair(800, 10));

    }

    private static class ShootingPair {
        private final double m_rpm;
        private final double m_pivotAngle;

        private ShootingPair(double rpm, double angle) {
            m_rpm = rpm;
            m_pivotAngle = angle;
        }
    }

    private static class ShootingPairInterpolator implements Interpolator<ShootingPair> {
        @Override
        public ShootingPair interpolate(ShootingPair startValue, ShootingPair endValue, double t) {
            return new ShootingPair(
                MathUtil.interpolate(startValue.m_rpm, endValue.m_rpm, t),
                MathUtil.interpolate(startValue.m_pivotAngle, endValue.m_pivotAngle, t));
        }
    }

    public double getVelocityTable(double distance)
    {
        return m_lookupTable.get(distance).m_rpm;
    }
}