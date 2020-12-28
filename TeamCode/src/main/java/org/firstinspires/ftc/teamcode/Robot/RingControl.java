package org.firstinspires.ftc.teamcode.Robot;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RingControl {
    DcMotorEx     ringintake;

    Robot robot;

    public RingControl(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Wheel Drive Motors Setup
        ringintake = robot.opMode.hardwareMap.get(DcMotorEx.class, "ringintake");
        ringintake.setDirection(DcMotorEx.Direction.REVERSE);
        ringintake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ringintake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ringintake.setPower(0);
    }
    public void DriverControlledRingIntake(double ringpower){
        ringintake.setPower(ringpower);
        robot.opMode.telemetry.addLine("Ring Intake Speed")
                .addData("Right", "%.3f", ringpower);
    }
    public void DriverControlledRingShooter(){

    }
}
