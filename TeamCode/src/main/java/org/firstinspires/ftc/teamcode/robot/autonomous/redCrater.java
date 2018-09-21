package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.constructor.motors.driveAuto;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.mechDriveAuto;

public class redCrater extends LinearOpMode {

    driveAuto myDriveAuto;


    @Override
    public void runOpMode() throws InterruptedException {
        myDriveAuto = new driveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));


    }
}
