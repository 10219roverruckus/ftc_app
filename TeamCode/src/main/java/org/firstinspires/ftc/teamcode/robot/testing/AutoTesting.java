package org.firstinspires.ftc.teamcode.robot.testing;

import android.sax.TextElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.outreach.OutreachRobot;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.outreachMotors;


@Autonomous(name = "Auto Testing - Main Channel Robot")

public class AutoTesting extends LinearOpMode  {

    MecanumDrive myMechDrive;
    outreachMotors myOutReachMotors;




    //gyro
    Gyro myGyro;

    @Override
    public void runOpMode() throws InterruptedException {



        final long sleepTime = 200;
        final double SPD_DRIVE_MED = .5;
        myOutReachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
        myGyro.setLinearOp(this); //


        waitForStart();

        boolean active = true;
        while (opModeIsActive()) {
            while (active) {
                myOutReachMotors.drive(.4, .4); // forward
                telemetry.addLine("DRIVE FORWARD FROM CRATER");
                telemetry.update();
                sleep(1350);
                myOutReachMotors.drive(-.2, .2); // rotate left
                sleep(300);
                telemetry.addLine("ROTATE TOWARDS WALL");
                telemetry.update();
                myGyro.gyroOrientOutreach(90, myOutReachMotors); //-90
                sleep(300);
                myOutReachMotors.drive(.8,.8); // drive forward
                sleep(2800);
                telemetry.addLine("DRIVE TOWARD WALL");
                telemetry.update();
                myOutReachMotors.drive(-.2,.2);
                sleep(850);

                telemetry.addLine("ROTATE TOWARDS DEPO");
                telemetry.update();
                myGyro.gyroOrientOutreach(70, myOutReachMotors); //-90
                myOutReachMotors.drive(0,0);
                active = false;
            }
            //requestOpModeStop();
            idle();
        }
    }
}