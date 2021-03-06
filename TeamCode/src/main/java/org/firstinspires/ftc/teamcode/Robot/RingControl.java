package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RingControl {
    DcMotorEx ringintake;
    DcMotorEx ringshooterleft;
    DcMotorEx ringshooterright;
    Servo     bucketservo;

    Robot robot;

    public RingControl(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Wheel Drive Motors Setup
        ringintake  = robot.opMode.hardwareMap.get(DcMotorEx.class, "ringintake");
        ringshooterright = robot.opMode.hardwareMap.get(DcMotorEx.class, "ringshooterright");
        ringshooterleft = robot.opMode.hardwareMap.get(DcMotorEx.class, "ringshooterleft");
        bucketservo = robot.opMode.hardwareMap.get(Servo.class, "bucketServo");

        ringintake.setDirection(DcMotorEx.Direction.REVERSE);
        ringintake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ringintake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ringintake.setPower(0);

        ringshooterleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringshooterright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void DriverControlledRingIntake(double ringpower){
        ringintake.setPower(ringpower);
        robot.opMode.telemetry.addLine("Ring Intake Speed")
                .addData("Right", "%.3f", ringpower);
    }
    public void DriverControlledRingShooter(double position){

        bucketservo.setPosition(position);
        while (bucketservo.getPosition() != position){};
    }
    public void ConstantRingShooter(double YEETFORCE){

        ringshooterleft.setPower(YEETFORCE);
        ringshooterright.setPower(-YEETFORCE);
    }
}
