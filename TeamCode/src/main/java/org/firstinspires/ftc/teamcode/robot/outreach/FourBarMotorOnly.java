package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.FourBarLink;

@TeleOp(name = "4 Bar Link Motor Only")


public class FourBarMotorOnly extends OpMode {
    FourBarLink myFourBarLink;

    public void init () {
        myFourBarLink = new FourBarLink(hardwareMap.dcMotor.get("four_bar_motor"));

    }

    public void loop() {
        controlFourBarLink();

    }

    public void controlFourBarLink () {
        if (gamepad1.left_stick_y != 0) {
            myFourBarLink.FourBarManualControl(gamepad1.left_stick_y);
        }
        else {
            myFourBarLink.stopMotor();
        }
    }
}
