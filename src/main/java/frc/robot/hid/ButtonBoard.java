// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
    
    public class ButtonBoard extends GenericHID {
       
        public enum Button {
            K1(1),
            K2(2),
            K3(3),
            K4(4),
            L1(5),
            R1(6),
            L2(7),
            R2(8),
            SE(9),
            ST(10),
            K11(11),
            K12(12);
    
            public final int value;
    
            Button(int value) {
                this.value = value;
            }
        }
    
        public ButtonBoard(int port) {
            super(port);
        }

        //button 1

        public boolean getButton1() {
            return getRawButton(Button.K1.value);
          }
        
          public boolean getButton1Pressed() {
            return getRawButtonPressed(Button.K1.value);
          }
      
          public boolean getButton1Released() {
            return getRawButtonReleased(Button.K1.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button1(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton1);
          }


        // button 2
          
        public boolean getButton2() {
            return getRawButton(Button.K2.value);
          }
        
          public boolean getButton2Pressed() {
            return getRawButtonPressed(Button.K2.value);
          }
      
          public boolean getButton2Released() {
            return getRawButtonReleased(Button.K2.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button2(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton2);
          }

        
        //button3
          
        public boolean getButton3() {
            return getRawButton(Button.K3.value);
          }
        
          public boolean getButton3Pressed() {
            return getRawButtonPressed(Button.K3.value);
          }
      
          public boolean getButton3Released() {
            return getRawButtonReleased(Button.K3.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button3(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton3);
          }



        //button4
          
        public boolean getButton4() {
            return getRawButton(Button.K4.value);
          }
        
          public boolean getButton4Pressed() {
            return getRawButtonPressed(Button.K4.value);
          }
      
          public boolean getButton4Released() {
            return getRawButtonReleased(Button.K4.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button4(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton4);
          }


        //button 5
          
        public boolean getButton5() {
            return getRawButton(Button.L1.value);
          }
        
          public boolean getButton5Pressed() {
            return getRawButtonPressed(Button.L1.value);
          }
      
          public boolean getButton5Released() {
            return getRawButtonReleased(Button.L1.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button5(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton5);
          }

        //button 6
          
        public boolean getButton6() {
            return getRawButton(Button.R1.value);
          }
        
          public boolean getButton6Pressed() {
            return getRawButtonPressed(Button.R1.value);
          }
      
          public boolean getButton6Released() {
            return getRawButtonReleased(Button.R1.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button6(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton6);
          }


        //button 7
          
        public boolean getButton7() {
            return getRawButton(Button.L2.value);
          }
        
          public boolean getButton7Pressed() {
            return getRawButtonPressed(Button.L2.value);
          }
      
          public boolean getButton7Released() {
            return getRawButtonReleased(Button.L2.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button7(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton7);
          }

          
        //button 8

        public boolean getButton8() {
            return getRawButton(Button.R2.value);
          }
        
          public boolean getButton8Pressed() {
            return getRawButtonPressed(Button.R2.value);
          }
      
          public boolean getButton8Released() {
            return getRawButtonReleased(Button.R2.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button8(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton8);
          }


        //button 9
          
        public boolean getButton9() {
            return getRawButton(Button.SE.value);
          }
        
          public boolean getButton9Pressed() {
            return getRawButtonPressed(Button.SE.value);
          }
      
          public boolean getButton9Released() {
            return getRawButtonReleased(Button.SE.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button9(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton9);
          }


        //button 10

        public boolean getButton10() {
            return getRawButton(Button.ST.value);
          }
        
          public boolean getButton10Pressed() {
            return getRawButtonPressed(Button.ST.value);
          }
      
          public boolean getButton10Released() {
            return getRawButtonReleased(Button.ST.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button10(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton10);
          }


        //button 11
          
        public boolean getButton11() {
            return getRawButton(Button.K11.value);
          }
        
          public boolean getButton11Pressed() {
            return getRawButtonPressed(Button.K11.value);
          }
      
          public boolean getButton11Released() {
            return getRawButtonReleased(Button.K11.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button11(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton11);
          }


        //button 12
          
        public boolean getButton12() {
            return getRawButton(Button.K12.value);
          }
        
          public boolean getButton12Pressed() {
            return getRawButtonPressed(Button.K12.value);
          }
      
          public boolean getButton12Released() {
            return getRawButtonReleased(Button.K12.value);
          }
    
          @SuppressWarnings("MethodName")
          public BooleanEvent button12(EventLoop loop) {
            return new BooleanEvent(loop, this::getButton12);
          }
}


