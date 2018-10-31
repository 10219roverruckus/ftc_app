package org.firstinspires.ftc.teamcode.robot.competition.autonomous;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;




public class MineralMiner {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 15; //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100; //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MineralMiner() {
    }

    public void findingMineral() {
//        if (detector.getXPosition() < 280) {
//            goldPosition = GoldPosition.LEFT;
//        }
//        else if (detector.getXPosition() > 500) {
//            goldPosition = GoldPosition.RIGHT;
//        }
//        else {
//            goldPosition = GoldPosition.MIDDLE;
//        }

        goldPosition = GoldPosition.RIGHT;
    }

    public void driveMineral(Gyro myGyro, DriveMotors myDriveMotors) {
        myDriveMotors.drivePID(.3, .5); //DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        switch (goldPosition) {  //ANGLES SELF AND GOES TOWARD GOLD MINERAL
            case LEFT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(34, myDriveMotors);
                myDriveMotors.drivePID(.4, 3.4);
                break;

            case MIDDLE:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(7, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.8);
                break;

            case RIGHT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(-11, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.7);
                break;
        }
    }

    //new class name (BackUpColorSensor)
    public void craterMineralToWall(Gyro myGyro, DriveMotors myDriveMotors, RevColorDistance myRevColorDisance) {
        //Drives back to tape
        // SHOULD BE ABLE TO USE FOR DETECTING EITHER COLOR.
        myDriveMotors.stopMotors();
        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myDriveMotors.drive(-.4, -.4);
            linearOp.idle();
        }
        myDriveMotors.stopMotors();
        //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT
        myDriveMotors.drivePID(.3, .4);
        myGyro.gyroOrientOutreach(87, myDriveMotors);  //orients self with red tape so parallel to tape.
        //
        //MAY NEED TO BE LESS THAN 90 DEGRESS SO ROBOT DOES NOT HIT THE LANDER LEG!
        //


        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        //DRIVES TO WALL
        switch (goldPosition) {  //DRIVE TO WALL
            case LEFT:
                myDriveMotors.drivePID(.6, 6);
                break;
            case MIDDLE:
                myDriveMotors.drivePID(.6, 6);
                break;
            case RIGHT:
                myDriveMotors.drivePID(.6, 6.5);
                break;
        }
    }

    public void wallToDepot(Gyro myGyro, DriveMotors myDriveMotors, RevColorDistance myRevColorDisance) {
        myGyro.gyroOrientOutreach(130, myDriveMotors);
        myDriveMotors.drivePID(.6, 3);
        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myDriveMotors.drive(.4, .4);
            linearOp.idle();
        }
        myDriveMotors.stopMotors();
        myDriveMotors.drivePID(-.6, 6);
        //EMMA's got this...
        // REMEMBER: WE HAVE NOT TURNED YET. WE ARE NOT ANGLED PARALLEL WITH THE WALL.

    }
}

