package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class IMUControl {
    public BNO055IMU    imu;
    public Orientation  angles;
    public Acceleration gravity;
    public double       StartingHeading;
    Robot robot;

    public IMUControl(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Init IMU
        imu = robot.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit    = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit    = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        StartingHeading = angles.firstAngle;
    }
    public float readimuheading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        robot.opMode.telemetry.addData("Heading ", angles.firstAngle);
        return angles.firstAngle;
    }

    String formatAngle(AngleUnit angleunit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleunit, angle));
    }

    String formatDegrees(double degrees){
        return  String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
