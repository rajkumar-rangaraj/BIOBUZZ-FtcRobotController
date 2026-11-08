package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OLD AUTO RED", group = "BioBuzz Old", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzAutoRed extends BioBuzzAutoBase {
    @Override
    protected HiveVision.Alliance alliance() {
        return HiveVision.Alliance.RED;
    }
}
