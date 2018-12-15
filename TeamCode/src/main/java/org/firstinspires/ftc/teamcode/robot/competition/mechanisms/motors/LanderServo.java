package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderServo {

    // instance variables
    public Servo landerServo;

    public LinearOpMode landerServoLinearOp = null;
    public LinearOpMode linearOp = null;

    public double LSOpen = 1;
    public double LSClose = 0;

    // constructor

    public LanderServo ( Servo LS) {
        LS = landerServo;

        landerServo.setDirection(Servo.Direction.FORWARD);
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public void landerServoOpened () {          // drops minerals into the lander
        landerServo.setPosition(LSOpen);
    }

    public void landerServoClosed () {
        landerServo.setPosition(LSClose);      // keeps minerals in the little object
    }
}



