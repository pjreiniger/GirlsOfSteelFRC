package com.gos.crescendo2024.leds;

import com.gos.lib.led.LEDPattern;

public class HasGamePiecePattern implements LEDPattern {
    private final int m_maxCtr;

    private boolean m_hasPiece;
    private int m_highCtr;

    private final LEDPattern m_patternToShow;

    public HasGamePiecePattern(LEDPattern pattern, double seconds) {
        m_patternToShow = pattern;
        m_maxCtr = (int) (seconds / 0.02);
    }


    public void setPiece(boolean hasPiece) {
        m_hasPiece = hasPiece;
    }

    public boolean isPatternActive() {
        return m_hasPiece && m_highCtr < m_maxCtr;
    }

    @Override
    public void writeLeds() {
        if (m_hasPiece) {
            ++m_highCtr;
        } else {
            m_highCtr = 0;
        }

        if (isPatternActive()) {
            m_patternToShow.writeLeds();
        }
    }
}
