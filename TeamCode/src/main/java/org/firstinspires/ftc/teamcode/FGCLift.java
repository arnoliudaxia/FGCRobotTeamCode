package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FGCLift", group="Linear Opmode")
//@Disabled
public class FGCLift extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();//计时
    public DcMotor motorLeftLift;
    public DcMotor motorRightLift;
    boolean isLiftRunning =false;
    double LiftPower =.4;



    public void init()
    {
        LiftConfigue();
        telemetry.addData("Inited","Done");
        telemetry.addData("Ammo(Useful)\t",getBatteryVoltage());
        telemetry.update();
    }
    @Override
    public void init_loop()
    {
    }
    @Override
    public void start()
    {
        telemetry.clearAll();
        runtime.reset();
    }
    public void loop()
    {
        if (gamepad1.y) {
            while(gamepad1.y){}
                motorLeftLift.setPower(LiftPower);
                motorRightLift.setPower(LiftPower);
                isLiftRunning =true;
        }
        if (gamepad1.b)
        {
            while (gamepad1.b){}
            motorLeftLift.setPower(0);
            motorRightLift.setPower(0);
            isLiftRunning =false;
        }
        if (gamepad1.a)
        {
            while(gamepad1.a){}
            motorLeftLift.setPower(-LiftPower);
            motorRightLift.setPower(-LiftPower);
            isLiftRunning =true;
        }
        //region 改变升降机功率
        //        if(gamepad1.left_bumper&&!isLiftRunning)
//        {
//            LiftPower= Range.clip(LiftPower+.1,0,1);
//        }
//        if (gamepad1.right_bumper&&!isLiftRunning)
//        {
//            LiftPower= Range.clip(LiftPower-.1,0,1);
//        }
        //endregion
//        telemetry.addData("LiftPower",LiftPower);
//        telemetry.update();
    }
    public double getBatteryVoltage()//获取电量
    {
        double result = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if(voltage > 0)
            {
                result = Math.min(result,voltage);
            }
        }
        return result;
    }
    public void LiftConfigue()
    {
        motorLeftLift=hardwareMap.get(DcMotor.class,"motorLeftLift");
        motorRightLift=hardwareMap.get(DcMotor.class,"motorRightLift");
        motorLeftLift.setDirection(DcMotor.Direction.FORWARD);
        motorRightLift.setDirection(DcMotor.Direction.FORWARD);
    }
}
