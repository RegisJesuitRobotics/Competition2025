// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandButtonBoard extends CommandGenericHID {
  private final ButtonBoard hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandButtonBoard(int port) {
    super(port);
    hid = new ButtonBoard(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public ButtonBoard getHID() {
    return hid;
  }

  public Trigger Button1() {
    return Button1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button1(EventLoop loop) {
    return hid.button1(loop).castTo(Trigger::new);
  }

  public Trigger Button2() {
    return Button2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button2(EventLoop loop) {
    return hid.button2(loop).castTo(Trigger::new);
  }

  public Trigger Button3() {
    return Button3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button3(EventLoop loop) {
    return hid.button3(loop).castTo(Trigger::new);
  }

  public Trigger Button4() {
    return Button4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button4(EventLoop loop) {
    return hid.button4(loop).castTo(Trigger::new);
  }

  public Trigger Button5() {
    return Button5(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button5(EventLoop loop) {
    return hid.button5(loop).castTo(Trigger::new);
  }

  public Trigger Button6() {
    return Button6(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button6(EventLoop loop) {
    return hid.button6(loop).castTo(Trigger::new);
  }

  public Trigger Button7() {
    return Button7(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button7(EventLoop loop) {
    return hid.button7(loop).castTo(Trigger::new);
  }

  public Trigger Button8() {
    return Button8(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button8(EventLoop loop) {
    return hid.button8(loop).castTo(Trigger::new);
  }

  public Trigger Button9() {
    return Button9(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button9(EventLoop loop) {
    return hid.button9(loop).castTo(Trigger::new);
  }

  public Trigger Button10() {
    return Button10(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button10(EventLoop loop) {
    return hid.button10(loop).castTo(Trigger::new);
  }

  public Trigger Button11() {
    return Button11(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button11(EventLoop loop) {
    return hid.button11(loop).castTo(Trigger::new);
  }

  public Trigger Button12() {
    return Button12(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger Button12(EventLoop loop) {
    return hid.button12(loop).castTo(Trigger::new);
  }
}
