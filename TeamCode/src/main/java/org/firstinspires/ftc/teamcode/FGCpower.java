package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FGCpower", group = "Iterative Opmode")
//@Disabled
public class FGCpower extends OpMode {
    //region 定义底盘、卷机、抬升电机
    public DcMotor motorLF;
    public DcMotor motorRF;
    public DcMotor motorLB;
    public DcMotor motorRB;
    public DcMotor motorLeftLift;
    public DcMotor motorRightLift;
    //endregion
    //region 定义档位
    double powerMode = 1;//切换快/慢速模式
    final double POWER_MODE_SLOW = 2.5;
    final double POWER_MODE_FAST = 1;
    //endregion
    private ElapsedTime runtime = new ElapsedTime();//计时
    @Override
    public void init()
    {
        PowerSystemConfigure();//初始化硬件
        // CLS.CollectSystemConfigue();
        telemetry.addData("Commander(HardWare)\t","Ready!!!");
        telemetry.addData("Ammo(Useful)\t",getBatteryVoltage());
        telemetry.update();
    }
    public void PowerSystemConfigure()
    {
        //region 实例化电机
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");
        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
//        motorLeftLift = hardwareMap.get(DcMotor.class,"motorLeftLift");
//        motorRightLift = hardwareMap.get(DcMotor.class,"motorRightLift");
        //endregion
        //region 设置电机转向
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
//        motorLeftLift.setDirection(DcMotor.Direction.REVERSE);
//        motorRightLift.setDirection(DcMotor.Direction.REVERSE);
        //endregion
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
    @Override
    public void loop()
    {
        /*telemetry.addData("LY",gamepad1.left_stick_y);
        telemetry.addData("LX",gamepad1.left_stick_x);
        telemetry.addData("RX",gamepad1.right_stick_x);
        telemetry.addData("RY",gamepad1.right_stick_y);
        telemetry.addData("X",gamepad1.x);
        telemetry.addData("Y",gamepad1.y);
        telemetry.addData("B",gamepad1.b);
        telemetry.addData("A",gamepad1.a);
        telemetry.addData("LB",gamepad1.left_bumper);
        telemetry.addData("RB",gamepad1.right_bumper);
        telemetry.addData("LT",gamepad1.left_trigger);
        telemetry.addData("RT",gamepad1.right_trigger);
        telemetry.addData("UP",gamepad1.dpad_up);
        telemetry.addData("DOWN",gamepad1.dpad_down);
        telemetry.addData("LEFT",gamepad1.dpad_left);
        telemetry.addData("RIGHT",gamepad1.dpad_right);*/
        frameStop();
        telemetry.update();
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
    //region 动力系统
    enum moveDirection
    {
        F,B,R,L
    }

    //由定量输入值的底盘移动方法，0<power<1
    public void moveFix(double power,moveDirection moveDirection)
    {
        switch(moveDirection)
        {
            case F:
                motorLF.setPower(power);
                motorRF.setPower(power);
                motorLB.setPower(power);
                motorRB.setPower(power);
                break;
            case B:
                motorLF.setPower(-power);
                motorRF.setPower(-power);
                motorLB.setPower(-power);
                motorRB.setPower(-power);
                break;
            case L:
                motorLF.setPower(-power);
                motorRF.setPower(power);
                motorLB.setPower(-power);
                motorRB.setPower(power);
                break;
            case R:
                motorLF.setPower(power);
                motorRF.setPower(-power);
                motorLB.setPower(power);
                motorRB.setPower(-power);
        }
    }


    /**
     * @param yPower    y轴功率（前后平移方向上的功率）
     * @param rPower    自转功率（左右转向的功率）
     * @param powerMode 快慢模式，越大越慢
     */
    public void moveVar(double yPower,double rPower,double powerMode)
    {
        double FinalPower1 = Range.clip((yPower+rPower)/powerMode,-1,1);
        double FinalPower2 = Range.clip((yPower-rPower)/powerMode,-1,1);
        double FinalPower3 = Range.clip((yPower+rPower)/powerMode,-1,1);
        double FinalPower4 = Range.clip((yPower-rPower)/powerMode,-1,1);

        motorLF.setPower(FinalPower1);
        motorRF.setPower(FinalPower2);
        motorLB.setPower(FinalPower3);
        motorRB.setPower(FinalPower4);
    }

    //停止
    public void frameStop()
    {
//        motorLF.setPower(0);
//        motorRF.setPower(0);
//        motorLB.setPower(0);
//        motorRB.setPower(0);

        moveVar(0,0,1);

        moveFix(0, moveDirection.F);
    }

    //变速
    public double switchPowerMode()
    {
        if(gamepad1.left_bumper)
        {
            return powerMode-.25;
        } else if(gamepad1.right_bumper)
        {
            return powerMode+.25;
        }
        return powerMode;
    }
    public void PowerLoop()
    {
            if (gamepad1.dpad_left)  moveFix(.7,moveDirection.L);
            if (gamepad1.dpad_right)   moveFix(.7,moveDirection.R);
            if (gamepad1.right_trigger!=0)  moveFix(Range.clip(Math.abs(gamepad1.right_trigger)+.3,0,1),moveDirection.F);
            if (gamepad1.left_trigger!=0)   moveFix(Range.clip(Math.abs(gamepad1.left_trigger)+.3,0,1),moveDirection.B);

         else
        {
            frameStop();

        }
        telemetry.addData("y",gamepad1.left_stick_y);
        telemetry.addData("x",gamepad1.left_stick_x);
    }

    //endregion
}
