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
//import org.firstinspires.ftc.teamcode.robot.outreach.outreachMotors;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.MineralMiner;



    @Autonomous(name = "AUTO Testing - Outreach Crater")

public class AutoTesting extends LinearOpMode  {

    //MecanumDrive myMechDrive;
    //outreachMotors myOutReachMotors;
    //MotorsPID myMotorsPID;

    DriveMotors myDriveMotors;
    Gyro myGyro;
    MineralMiner myMineralMiner;

    @Override
    public void runOpMode() throws InterruptedException {



        final long sleepTime = 100;
        final double SPD_DRIVE_MED = .5;
  //      myMotorsPID = new MotorsPID(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
//        myMotorsPID.setLinearOp(this);
        myDriveMotors = new DriveMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        myDriveMotors.setLinearOp(this);
        //myOutReachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        //myOutReachMotors.setLinearOp(this);

        myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
        myGyro.setLinearOp(this); //

        myMineralMiner = new MineralMiner();
        myMineralMiner.setLinearOp(this);


        waitForStart();

        boolean active = true;
        while (opModeIsActive() && !isStopRequested()) {
            while (active && !isStopRequested()) {
//                telemetry.addLine("DRIVE FORWARD FROM CRATER");
//                telemetry.update();
                idle();
                myMineralMiner.findingMineral();
                idle();
                myMineralMiner.driveMineral(myGyro, myDriveMotors);
                //myMineralMiner.driveMineral(myGyro, myDriveMotors);
                //  back up to tape w/ color sensor
                //  turn left 90 degrees.






                //myDriveMotors.drivePID(.6, 3);
                //sleep(sleepTime);
                //myOutReachMotors.drive(.4, .4); //forward -
                //sleep(1350);
//                telemetry.addLine("ROTATE TOWARDS WALL");
//                telemetry.update();
                //myOutReachMotors.drive(-.2, .2); // rotate left
                //sleep(sleepTime);
//                telemetry.addLine("ORIENT WALL WITH GYRO");
//                telemetry.update();
               // myGyro.gyroOrientOutreach(87, myDriveMotors); //-90
                //sleep(sleepTime);
//                telemetry.addLine("DRIVE TOWARD WALL");
//                telemetry.update();
               // myDriveMotors.drivePID (.6, 5.3);
                //sleep(sleepTime);
                //myOutReachMotors.drive(.8,.8); // drive forward
                //sleep(1700);
//                telemetry.addLine("ROTATE WITH WALL");
//                telemetry.update();
               // myOutReachMotors.drive(-.2,.2);
                idle();
//                telemetry.addLine("ORIENT WITH WALL USING GYRO");
//                telemetry.update();
                //myGyro.gyroOrientOutreach(135, myDriveMotors); //-70
                //sleep(sleepTime);
                //make the program stop!

//                telemetry.addLine("TOWARD DEPOT");
//                telemetry.update();
                //myDriveMotors.drivePID(.6,3.3);
                //sleep(sleepTime);
                //myGyro.gyroOrientOutreach(132, myDriveMotors);
                //sleep(sleepTime);

                //drop servo Team Marker arm
                //lift servo Team Marker arm

//                telemetry.addLine("GO BACK TO CRATER");
//                telemetry.update();
               //// myDriveMotors.drivePID(-.6, 3.5); //half way backwards
               //// sleep(sleepTime);
                ////myGyro.gyroOrientOutreach(135, myDriveMotors); // adjust angle
                ////sleep(sleepTime);
                ////myDriveMotors.drivePID(-.6, 3.3); // last half way backwards
                ////sleep(sleepTime);
                ////myGyro.gyroOrientOutreach(135, myDriveMotors); // adjust angle

                active = false;
            }
            //requestOpModeStop();
            idle();
            requestOpModeStop();
        }
    }
}