package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
public class WobbleGripper {
    static final double MAX_POS     = 1.0;
    static final double MIN_POS     = 0.0;
    Servo     wobbleservo;
    DcMotorEx wobblearmmotor;
    Robot robot;

    public WobbleGripper(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Wheel Drive Motors Setup
        wobbleservo = robot.opMode.hardwareMap.get(Servo.class, "wobbleservo");
        wobblearmmotor =   robot.opMode.hardwareMap.get(DcMotorEx.class, "wobblearmmotor");
        wobblearmmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleservo.setPosition(0);
    }

    public  void  setArmPower(double wobblepower) {
        wobblearmmotor.setPower(wobblepower);
        while (wobblearmmotor.isBusy()); {

        }
    };

    public void autonomousArmMovement(double wobblepower) {
        wobblearmmotor.setPower(wobblepower);
    }

    public void setWobbleGripper(double wobbleposition){

        wobbleservo.setPosition(wobbleposition);
        double currentwobbleposition = wobbleservo.getPosition();
        while (currentwobbleposition != wobbleposition){
            //wait until in position
        }

    }


}
