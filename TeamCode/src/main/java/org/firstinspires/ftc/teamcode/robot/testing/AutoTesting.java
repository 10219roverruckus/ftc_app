package org.firstinspires.ftc.teamcode.robot.testing;

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


        while (opModeIsActive()) {

            myOutReachMotors.drive (.5,.5); // forward
            sleep (600);
            myOutReachMotors.drive(-.5,.5); // rotate left
            sleep (300);
            myGyro.gyroOrientOutreach(90, myOutReachMotors); //-90
            sleep(300);
            myOutReachMotors.drive(.5,.5); // drive forward
            sleep(300);
            myOutReachMotors.drive(-.5,.5);

            idle();
        }
       // myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
        //myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));

     //   myGyro.gyroOrientOutreach(-40,myOutReachMotors);


//        myMechDrive.driveForward(SPD_DRIVE_MED, 2.6);
////        myMechDrive.rotateRight(SPD_DRIVE_MED, .7);
////        myGyro.gyroOrient(-60, myMechDrive);
//        myMechDrive.driveForward(SPD_DRIVE_MED,1);
//        myMechDrive.rotateLeft(SPD_DRIVE_MED,.6);
//        myGyro.gyroOrientMecanum(-80, myMechDrive);



        //testing gyro to adjust to face the depot after it reaches the wall
        //LET ORIENT OURSELVES!  (???)
//        myGyro.gyroOrient(70, myMechDrive);

    }
}