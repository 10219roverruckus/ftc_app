package org.firstinspires.ftc.teamcode.robot.competition.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;




public class MineralMiner {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 10;
    public final int BLUE_THRESHOLD = 10;

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MineralMiner () { }

    public void findingMineral () {
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

    public void driveMineral (Gyro myGyro, DriveMotors myDriveMotors) {
        myDriveMotors.drivePID(.3, .5); //DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        switch(goldPosition) {  //ANGLES SELF AND GOES TOWARD GOLD MINERAL
            case LEFT:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(34, myDriveMotors);
                myDriveMotors.drivePID(.4, 3.4);
                break;

            case MIDDLE:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(7, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.8);
                break;

            case RIGHT:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(-11, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.7);
                break;
        }
    }

    public void craterMineralToWall (Gyro myGyro, DriveMotors myDriveMotors, RevColorDistance myRevColorDisance) {
        //Drives back to tape
        // SHOULD BE ABLE TO USE FOR DETECTING EITHER COLOR.
        while (myRevColorDisance.revColorSensor.red() < RED_THRESHOLD && myRevColorDisance.revColorSensor.blue() < BLUE_THRESHOLD) {
            myDriveMotors.drive(-.4, -.4);
            linearOp.idle();
        }
        myDriveMotors.stopMotors();  //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT

        myGyro.gyroOrientOutreach(87, myDriveMotors);  //orients self with red tape so parallel to tape.
        //
        //MAY NEED TO BE LESS THAN 90 DEGRESS SO ROBOT DOES NOT HIT THE LANDER LEG!
        //


        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        switch (goldPosition) {  //DRIVE TO WALL
            case LEFT:
                myDriveMotors.drivePID(.6, 2);
                break;
            case MIDDLE:
                myDriveMotors.drivePID(.6, 2);
                break;
            case RIGHT:
                myDriveMotors.drivePID(.6, 2);
                break;
        }
    }

    public void wallToDepot (Gyro myGyro, DriveMotors myDriveMotors) {
        //EMMA's got this...
        // REMEMBER: WE HAVE NOT TURNED YET SO WE'RE ANGLED PARALLEL WITH THE WALL.

    }

}
