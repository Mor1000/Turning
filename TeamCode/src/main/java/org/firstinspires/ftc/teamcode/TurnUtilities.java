package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * Summary: This class stores functions for turning the robot.
 */
public class TurnUtilities {


    /**
     * @param goalAngle   desired position angle for turning
     * @param driveMotors the robot drivetrain motors
     * @param imu         gyro sensor for measuring the angle
     * @param power       turning motors power
     */
    void Turn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
        boolean sideOfTurn = true;//true = turn clockwise, false = turn counter-clockwise
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //finding the shorter direction to turn
        if (goalAngle == 0) {
            if (currentAngle < 0) {
                sideOfTurn = false;    //counter-clockwise
            } else if (currentAngle > 0) {
                sideOfTurn = true;    //clockwise
            }
        }
       else if (currentAngle <= 0 && goalAngle < 0 &&
              currentAngle < goalAngle) {
            sideOfTurn = false;     //counter-clockwise

        } else if (currentAngle <= 0 && goalAngle < 0
                && currentAngle > goalAngle) {
            sideOfTurn = true;    //clockwise

        }
       else if (currentAngle >= 0 && goalAngle > 0 &&
                currentAngle < goalAngle)

        {
            sideOfTurn = false;    //counter-clockwise

        } else if (currentAngle >= 0 && goalAngle > 0 &&
                currentAngle > goalAngle) {
            sideOfTurn = true;    //clockwise

        } else if (currentAngle >= 0 && goalAngle < 0)

        {
            if (currentAngle - goalAngle >
                    360 -( currentAngle -goalAngle)) {
                sideOfTurn = false;    //counter-clockwise
            } else {
                sideOfTurn = true;    //clockwise
            }
        } else if (currentAngle <= 0 && goalAngle > 0)

        {
            if (goalAngle-currentAngle <
                    360 -(goalAngle-currentAngle)) {
                sideOfTurn = false;    //counter-clockwise
            } else {
                sideOfTurn = true;    //clockwise
            }

        }
        if (sideOfTurn) {
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
            if (goalAngle > 0 && currentAngle < 0)

                while (goalAngle > currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running

            else {

                while (goalAngle < currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }

                //motors running

            }
        } else {
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
            if (goalAngle < 0 && currentAngle > 0)
                while (goalAngle < currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running

            else
                while (goalAngle > currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
            //motors running

        }
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
    }

    private static void setMotorPower(DcMotor[][] motors, double[][] powers) {
        for (int i = 0; i < motors.length; i++)
            for (int j = 0; j < motors[i].length; j++)
                motors[i][j].setPower(powers[i][j]);
    }
}
