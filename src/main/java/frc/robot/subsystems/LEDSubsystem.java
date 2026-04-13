// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(62);
  AddressableLEDBufferView m_leftView = m_ledBuffer.createView(0, 30);
  AddressableLEDBufferView m_rightView = m_ledBuffer.createView(31, 61);

  LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  Distance kLEDSpacing = Meters.of(1/120.0);
  LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLEDSpacing);

  LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  LEDPattern m_off = LEDPattern.kOff;
  LEDPattern m_purple = LEDPattern.solid(Color.kPurple);
  LEDPattern m_green = LEDPattern.solid(Color.kGreen);
  LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  LEDPattern m_red = LEDPattern.solid(Color.kRed);


  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }

  public Command rainbowChase() {
    return this.runOnce(() -> {
      m_scrollingRainbow.applyTo(m_ledBuffer);
    })
    .repeatedly()
    .ignoringDisable(true);
  }

  public Command yellow() {
    return this.runOnce(() -> {
      m_yellow.applyTo(m_ledBuffer);
    });
  }

  public Command off() {
    return this.runOnce(() -> {
      m_off.applyTo(m_ledBuffer);
    });
  }

  public Command blinkYellow() {
    return Commands.sequence(
      yellow(),
      Commands.waitSeconds(0.5),
      off(),
      Commands.waitSeconds(0.5)
    )
    .repeatedly()
    .ignoringDisable(true);
  }

  public Command purple() {
    return this.runOnce(() -> {
      m_purple.applyTo(m_ledBuffer);
    })
    .repeatedly()
    .ignoringDisable(true);
  }

  public Command green() {
    return this.runOnce(() -> {
      m_green.applyTo(m_ledBuffer);
    });
  }

  public Command red() {
    return this.runOnce(() -> {
      m_red.applyTo(m_ledBuffer);
    });
  }

  public Command blue() {
    return this.runOnce(() -> {
      m_blue.applyTo(m_ledBuffer);
    });
  }

  public Command flashRed() {
    return Commands.sequence(
      red(),
      Commands.waitSeconds(0.25),
      off(),
      Commands.waitSeconds(0.25)
    )
    .repeatedly();
  }

  public Command flashGreen() {
    return Commands.sequence(
      green(),
      Commands.waitSeconds(0.25),
      off(),
      Commands.waitSeconds(0.25)
    )
    .repeatedly();
  }

  public Command flashBlue() {
    return Commands.sequence(
      blue(),
      Commands.waitSeconds(0.25),
      off(),
      Commands.waitSeconds(0.25)
    )
    .repeatedly();
  }


  public Command flashShift() {
    return Commands.sequence(
      red(),
      Commands.waitSeconds(0.25),
      blue(),
      Commands.waitSeconds(0.25)
    )
    .repeatedly();
  }

}
