package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

public class Robot extends TimedRobot {

    // Joysticks
    Joystick joystick0;

    // Drivetrain
    DifferentialDrive myDrive;
    double leftPower = 0;
    double rightPower = 0;

    // Motors

    // Sparks
    Spark lbank;
    Spark rbank;
   
 
    @Override
    public void robotInit() {
        
        // Joystick
        joystick0 = new Joystick(0);

        rbank = new Spark(0);
        lbank = new Spark(1);
        // Drive Train
        myDrive = new DifferentialDrive(lbank, rbank);
    }

    @Override
    public void robotPeriodic() {
       
    }
    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
       
    }

    @Override
    public void teleopPeriodic() {
        lbank.setVoltage(12);
        rbank.setVoltage(12);
        // Turning speed limit
        double limitTurnSpeed = 0.75; // EDITABLE VALUE

        // Default manual Drive Values
        double joystickLValue =
                (-joystick0.getRawAxis(1) + (joystick0.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick0.getRawAxis(1) - (joystick0.getRawAxis(2) * limitTurnSpeed));

        // ADDITIONAL DRIVE CODE HERE

       
        // Actual Drive code
        //myDrive.tankDrive(joystickLValue, joystickRValue);
        myDrive.tankDrive(-leftPower, -rightPower, false);
    }
  
    @Override
    public void testPeriodic() {}

  }
  