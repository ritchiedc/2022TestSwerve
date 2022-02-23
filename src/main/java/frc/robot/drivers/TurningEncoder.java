// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class TurningEncoder  implements Sendable {
  private final double PI2 = Math.PI*2.0;
  private final AnalogInput m_analogInput;
  private double m_positionOffset;
  private String m_prefName;
  private boolean m_reverse = false;

  public TurningEncoder(int channel) {
    this(new AnalogInput(channel));
  }

  /**
   * Construct a new TurningEncoder attached to a specific AnalogInput.
   *
   * @param analogInput the analog input to attach to
   */
  public TurningEncoder(AnalogInput analogInput) {
    m_analogInput = analogInput;
    init();
  }

  private void init() {
    m_prefName = String.format("TurningEncoderOffset(%d)", m_analogInput.getChannel());
    m_positionOffset = Preferences.getDouble(m_prefName, 0);

    SendableRegistry.addLW(this, "TurningEncoder", m_analogInput.getChannel());
  }

  public double get() {
    double position = (m_analogInput.getVoltage() * (PI2 / 5.0)) - m_positionOffset;
    if (position >  Math.PI) position -= PI2;
    if (position < -Math.PI) position += PI2;
    if (m_reverse) position = -position;
    return position;
  }

  public void setReverseDirection(boolean reverse) {
    m_reverse = reverse;
  }

  public void reset() {
    m_positionOffset = m_analogInput.getVoltage() * (PI2 / 5.0);
    Preferences.setDouble(m_prefName, m_positionOffset);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TurningEncoder");
    builder.addDoubleProperty("Angle(rad)", this::get, null);
  }

}
