package SACG_15minTiaoTime10;

import ilog.concert.*;
import ilog.cp.IloCP;
import ilog.cplex.IloCplex;

import java.io.IOException;
import java.util.Arrays;

public class GR {
    static int DataBase = 9;
    static double epson = 1e-6;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 15;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);

    static int OneCancelCost = 200;
    static int MintDelayCost = 1;
    static int OneSwapCost = 0;
    static int GateCost = 30;

    static int MaxRouteLegNum = 10;
    static int MaxPatternLegNum = 12;
    static int GateCap = 500;


    static double[][] GRLP(int[] yCoverCf, int[][] yCoverCfPair,
                           int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightType, int GateTypeDivideline,
                           int[][][][] AirpTypeLegnumPattern, int[][] AirpTypePatternNum)
            throws IloException {

        int CfNum = CopyFlightOFID.length;
        int ApNum = AirpTypeLegnumPattern.length;
        int ConNum = yCoverCfPair.length;

        IloCplex cplex = new IloCplex();

        //定义变量
        IloNumVar[][][] w = new IloNumVar[ApNum][2][];


        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                String[] wname = new String[AirpTypePatternNum[ap][tp]];
                for(int i=0; i<AirpTypePatternNum[ap][tp]; i++){
                    wname[i]="w"+ap +","+ tp +","+ i;
                }
                w[ap][tp] = cplex.numVarArray(AirpTypePatternNum[ap][tp],0,1, wname);
            }
        }

        //目标函数：cost和最小的特例->使用的停机位数尽可能少
//        IloNumExpr obj = cplex.prod(2,cplex.sum(wc));
        IloNumExpr obj = cplex.prod(0,w[0][0][0]);
        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                obj=cplex.sum(obj, cplex.sum(w[ap][tp]));
            }
        }
        cplex.addMinimize(cplex.prod(GateCost, obj));

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[CfNum];
        rng[1] = new IloRange[CfNum];
        rng[2] = new IloRange[ConNum];
        rng[3] = new IloRange[ApNum*2];


        //约束：复制航班的决策与SARP统一  形式1
        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
        for(int cf=0; cf<CfNum; cf++) {
            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
        }
        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
                        int cf = AirpTypeLegnumPattern[ap][tp][i][1+j];
                        if(ap==CopyFlightDepS[cf]){
                            coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
                        }
                        if(ap==CopyFlightArrS[cf]){
                            coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
                        }
                    }
                }
            }
        }
        for(int cf=0; cf<CfNum; cf++) {
            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);

//            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
//                rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//                rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//            }else{
//                rng[0][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//                rng[1][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//            }

        }


        //约束：保证一架飞机进出的是同一个Gate yCoverCfPair[con][2]
        IloNumExpr[] coverCon = new IloNumExpr[ConNum];
        for(int con=0; con<ConNum; con++){
            coverCon[con] = cplex.prod(0, w[0][0][0]);
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(    AirpTypeLegnumPattern[ap][tp][i][1+j] == yCoverCfPair[con][0]
                            && AirpTypeLegnumPattern[ap][tp][i][1+j+1] == yCoverCfPair[con][1]){
                        coverCon[con] = cplex.sum(coverCon[con], w[ap][tp][i]);
                        break;
                    }
                }
            }
//            rng[2][con] = cplex.addEq(cplex.sum(coverCon[con],wc[con]), 1);
            if(  CopyFlightDepS[yCoverCfPair[con][0]]!=CopyFlightArrS[yCoverCfPair[con][0]]
                    && CopyFlightDepS[yCoverCfPair[con][1]]!=CopyFlightArrS[yCoverCfPair[con][1]]
            ){
                rng[2][con] = cplex.addEq(coverCon[con], 1);
            }else{
                rng[2][con] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }

        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                IloNumExpr gatecap = cplex.prod(0, w[0][0][0]);
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    gatecap = cplex.sum(gatecap, w[ap][tp][i]);
                }
                rng[3][ap*2+tp] = cplex.addGe(GateCap, cplex.prod(0, gatecap));
            }
        }


        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
//        cplex.setParam(IloCplex.Param.Threads, 1);
//        cplex.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
//        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.exportModel("GRPLP.lp");
        cplex.solve();

        System.out.println(cplex.getStatus());
        System.out.println("LpObj = "+cplex.getObjValue());

//        double[] solwc = cplex.getValues(wc);
//        for(int i=0; i<solwc.length; i++){
//            if(solwc[i]>0){
//                System.out.println(i+"~"+solwc[i]);
//            }
//        }

        double[][] LpDualValueLpValue = new double[5][];
        for(int i=0; i<4; i++){
            LpDualValueLpValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }
        LpDualValueLpValue[4] = new double[1];
        LpDualValueLpValue[4][0] = cplex.getObjValue();


        cplex.end();
        return LpDualValueLpValue;
    }

    static IloCplex GRLPphase1 (int[] yCoverCf, int[][] yCoverCfPair,
                                int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightType, int GateTypeDivideline,
                                int[][][][] AirpTypeLegnumPattern, int[][] AirpTypePatternNum)
            throws IloException {

        int CfNum = CopyFlightOFID.length;
        int ApNum = AirpTypeLegnumPattern.length;
        int ConNum = yCoverCfPair.length;

        IloCplex cplex = new IloCplex();

        //定义变量
        IloNumVar[][][] w = new IloNumVar[ApNum][2][];


        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                String[] wname = new String[AirpTypePatternNum[ap][tp]];
                for(int i=0; i<AirpTypePatternNum[ap][tp]; i++){
                    wname[i]="w"+ap +","+ tp +","+ i;
                }
                w[ap][tp] = cplex.numVarArray(AirpTypePatternNum[ap][tp],0,1, wname);
            }
        }

        //目标函数：cost和最小的特例->使用的停机位数尽可能少
//        IloNumExpr obj = cplex.prod(2,cplex.sum(wc));
        IloNumExpr obj = cplex.prod(0,w[0][0][0]);
        cplex.addMinimize(obj);

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[CfNum];
        rng[1] = new IloRange[CfNum];
        rng[2] = new IloRange[ConNum];
        rng[3] = new IloRange[ApNum*2];


        //约束：复制航班的决策与SARP统一  形式1
        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
        for(int cf=0; cf<CfNum; cf++) {
            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
        }
        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
                        int cf = AirpTypeLegnumPattern[ap][tp][i][1+j];
                        if(ap==CopyFlightDepS[cf]){
                            coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
                        }
                        if(ap==CopyFlightArrS[cf]){
                            coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
                        }
                    }
                }
            }
        }
        for(int cf=0; cf<CfNum; cf++) {
//            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
            //注释哪个？
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
                rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
            }else{
                rng[0][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
                rng[1][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }


        //约束：保证一架飞机进出的是同一个Gate yCoverCfPair[con][2]
        IloNumExpr[] coverCon = new IloNumExpr[ConNum];
        for(int con=0; con<ConNum; con++){
            coverCon[con] = cplex.prod(0, w[0][0][0]);
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(    AirpTypeLegnumPattern[ap][tp][i][1+j] == yCoverCfPair[con][0]
                            && AirpTypeLegnumPattern[ap][tp][i][1+j+1] == yCoverCfPair[con][1]){
                        coverCon[con] = cplex.sum(coverCon[con], w[ap][tp][i]);
                        break;
                    }
                }
            }
//            rng[2][con] = cplex.addEq(cplex.sum(coverCon[con],wc[con]), 1);
            if(  CopyFlightDepS[yCoverCfPair[con][0]]!=CopyFlightArrS[yCoverCfPair[con][0]]
                    && CopyFlightDepS[yCoverCfPair[con][1]]!=CopyFlightArrS[yCoverCfPair[con][1]]
            ){
                rng[2][con] = cplex.addEq(coverCon[con], 1);
            }else{
                rng[2][con] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }

        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                IloNumExpr gatecap = cplex.prod(0, w[0][0][0]);
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    gatecap = cplex.sum(gatecap, w[ap][tp][i]);
                }
                rng[3][ap*2+tp] = cplex.addGe(GateCap, cplex.prod(0, gatecap));
            }
        }


        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
        cplex.setWarning(null);
        cplex.setOut(null);

        return cplex;
    }

    static double[][] GRLPphase1DualValue(int[] yCoverCf, int[][] yCoverCfPair,
                                          int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightType, int GateTypeDivideline,
                                          int[][][][] AirpTypeLegnumPattern, int[][] AirpTypePatternNum)
            throws IloException {

        int CfNum = CopyFlightOFID.length;
        int ApNum = AirpTypeLegnumPattern.length;
        int ConNum = yCoverCfPair.length;

        IloCplex cplex = new IloCplex();

        //定义变量
        IloNumVar[][][] w = new IloNumVar[ApNum][2][];


        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                String[] wname = new String[AirpTypePatternNum[ap][tp]];
                for(int i=0; i<AirpTypePatternNum[ap][tp]; i++){
                    wname[i]="w"+ap +","+ tp +","+ i;
                }
                w[ap][tp] = cplex.numVarArray(AirpTypePatternNum[ap][tp],0,1, wname);
            }
        }

//        String[] wcname = new String[ConNum];
//        for(int i=0; i<ConNum; i++){
//            wcname[i]="wc"+ i;
//        }
//        IloNumVar[] wc = cplex.numVarArray(ConNum, 0, 1, wcname);

        //目标函数：cost和最小的特例->使用的停机位数尽可能少
//        IloNumExpr obj = cplex.prod(2,cplex.sum(wc));
        IloNumExpr obj = cplex.prod(0,w[0][0][0]);
        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                obj=cplex.sum(obj, cplex.sum(w[ap][tp]));
            }
        }
        cplex.addMinimize(cplex.prod(GateCost, obj));

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[CfNum];
        rng[1] = new IloRange[CfNum];
        rng[2] = new IloRange[ConNum];
        rng[3] = new IloRange[ApNum*2];


        //约束：复制航班的决策与SARP统一  形式1
        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
        for(int cf=0; cf<CfNum; cf++) {
            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
        }
        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
                        int cf = AirpTypeLegnumPattern[ap][tp][i][1+j];
                        if(ap==CopyFlightDepS[cf]){
                            coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
                        }
                        if(ap==CopyFlightArrS[cf]){
                            coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
                        }
                    }
                }
            }
        }
        for(int cf=0; cf<CfNum; cf++) {
            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);

//            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
//                rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//                rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//            }else{
//                rng[0][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//                rng[1][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//            }

        }


        //约束：复制航班的决策与SARP统一  形式2
//        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
//        for(int cf=0; cf<CfNum; cf++){
//            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
//            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
//            for(int ap=0; ap<ApNum; ap++){
//                for(int tp=0; tp<2; tp++){
//                    for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
//                        for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
//                            if(AirpTypeLegnumPattern[ap][tp][i][1+j] == cf){
//                                if(ap==CopyFlightDepS[cf]){
//                                    coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
//                                }
//                                if(ap==CopyFlightArrS[cf]){
//                                    coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
//                                }
//                                break;
//                            }
//                        }
//                    }
//                }
//            }
//            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//        }



        //约束：保证一架飞机进出的是同一个Gate yCoverCfPair[con][2]
        IloNumExpr[] coverCon = new IloNumExpr[ConNum];
        for(int con=0; con<ConNum; con++){
            coverCon[con] = cplex.prod(0, w[0][0][0]);
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(    AirpTypeLegnumPattern[ap][tp][i][1+j] == yCoverCfPair[con][0]
                            && AirpTypeLegnumPattern[ap][tp][i][1+j+1] == yCoverCfPair[con][1]){
                        coverCon[con] = cplex.sum(coverCon[con], w[ap][tp][i]);
                        break;
                    }
                }
            }
//            rng[2][con] = cplex.addEq(cplex.sum(coverCon[con],wc[con]), 1);
            if(  CopyFlightDepS[yCoverCfPair[con][0]]!=CopyFlightArrS[yCoverCfPair[con][0]]
                    && CopyFlightDepS[yCoverCfPair[con][1]]!=CopyFlightArrS[yCoverCfPair[con][1]]
            ){
                rng[2][con] = cplex.addEq(coverCon[con], 1);
            }else{
                rng[2][con] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }

        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                IloNumExpr gatecap = cplex.prod(0, w[0][0][0]);
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    gatecap = cplex.sum(gatecap, w[ap][tp][i]);
                }
                rng[3][ap*2+tp] = cplex.addGe(GateCap, cplex.prod(0, gatecap));
            }
        }


        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
//        cplex.setParam(IloCplex.Param.Threads, 1);
//        cplex.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
//        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.exportModel("GRPLP.lp");
        cplex.solve();

        System.out.println(cplex.getStatus());
        System.out.println("LpObj = "+cplex.getObjValue());

//        double[] solwc = cplex.getValues(wc);
//        for(int i=0; i<solwc.length; i++){
//            if(solwc[i]>0){
//                System.out.println(i+"~"+solwc[i]);
//            }
//        }

        double[][] LpDualValueLpValue = new double[4][];
        for(int i=0; i<4; i++){
            LpDualValueLpValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }


        cplex.end();
        return LpDualValueLpValue;
    }

    static int[][][] GRIP(int[] yCoverCf, int[][] yCoverCfPair,
                          int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightType, int GateTypeDivideline,
                          int[][][][] AirpTypeLegnumPattern, int[][] AirpTypePatternNum)
            throws IloException {

        int CfNum = CopyFlightOFID.length;
        int ApNum = AirpTypeLegnumPattern.length;
        int ConNum = yCoverCfPair.length;

        IloCplex cplex = new IloCplex();

        //定义变量
        IloIntVar[][][] w = new IloIntVar[ApNum][2][];


        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                String[] wname = new String[AirpTypePatternNum[ap][tp]];
                for(int i=0; i<AirpTypePatternNum[ap][tp]; i++){
                    wname[i]="w"+ap +","+ tp +","+ i;
                }
                w[ap][tp] = cplex.intVarArray(AirpTypePatternNum[ap][tp],0,1, wname);
            }
        }

//        String[] wcname = new String[ConNum];
//        for(int i=0; i<ConNum; i++){
//            wcname[i]="wc"+ i;
//        }
//        IloNumVar[] wc = cplex.numVarArray(ConNum, 0, 1, wcname);

        //目标函数：cost和最小的特例->使用的停机位数尽可能少
//        IloNumExpr obj = cplex.prod(2,cplex.sum(wc));
        IloIntExpr obj = cplex.prod(0,w[0][0][0]);
        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                obj=cplex.sum(obj, cplex.sum(w[ap][tp]));
            }
        }
        cplex.addMinimize(obj);

        IloRange[][] rng = new IloRange[3][];
        rng[0] = new IloRange[CfNum];
        rng[1] = new IloRange[CfNum];
        rng[2] = new IloRange[ConNum];


        //约束：复制航班的决策与SARP统一  形式1
        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
        for(int cf=0; cf<CfNum; cf++) {
            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
        }
        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
                        int cf = AirpTypeLegnumPattern[ap][tp][i][1+j];
                        if(ap==CopyFlightDepS[cf]){
                            coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
                        }
                        if(ap==CopyFlightArrS[cf]){
                            coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
                        }
                    }
                }
            }
        }
        for(int cf=0; cf<CfNum; cf++) {
            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);

//            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
//                rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//                rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//            }else{
//                rng[0][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//                rng[1][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//            }

        }


        //约束：复制航班的决策与SARP统一  形式2
//        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
//        for(int cf=0; cf<CfNum; cf++){
//            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
//            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
//            for(int ap=0; ap<ApNum; ap++){
//                for(int tp=0; tp<2; tp++){
//                    for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
//                        for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
//                            if(AirpTypeLegnumPattern[ap][tp][i][1+j] == cf){
//                                if(ap==CopyFlightDepS[cf]){
//                                    coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
//                                }
//                                if(ap==CopyFlightArrS[cf]){
//                                    coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
//                                }
//                                break;
//                            }
//                        }
//                    }
//                }
//            }
//            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//        }



        //约束：保证一架飞机进出的是同一个Gate yCoverCfPair[con][2]
        IloNumExpr[] coverCon = new IloNumExpr[ConNum];
        for(int con=0; con<ConNum; con++){
            coverCon[con] = cplex.prod(0, w[0][0][0]);
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(    AirpTypeLegnumPattern[ap][tp][i][1+j] == yCoverCfPair[con][0]
                            && AirpTypeLegnumPattern[ap][tp][i][1+j+1] == yCoverCfPair[con][1]){
                        coverCon[con] = cplex.sum(coverCon[con], w[ap][tp][i]);
                        break;
                    }
                }
            }
//            rng[2][con] = cplex.addEq(cplex.sum(coverCon[con],wc[con]), 1);
            if(  CopyFlightDepS[yCoverCfPair[con][0]]!=CopyFlightArrS[yCoverCfPair[con][0]]
                    && CopyFlightDepS[yCoverCfPair[con][1]]!=CopyFlightArrS[yCoverCfPair[con][1]]
            ){
                rng[2][con] = cplex.addEq(coverCon[con], 1);
            }else{
                rng[2][con] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }


        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.exportModel("GRPLP.lp");
        cplex.solve();

        System.out.println("GRIpStatus = "+cplex.getStatus());
        System.out.println("GRIpObjValue = "+cplex.getObjValue());

        int[][][] solw = new int[ApNum][2][];
        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                solw[ap][tp] = new int[AirpTypePatternNum[ap][tp]];
                for(int p=0; p<AirpTypePatternNum[ap][tp]; p++){
                    solw[ap][tp][p] = (int) cplex.getValue(w[ap][tp][p]);
                }
            }
        }

        cplex.end();

        return solw;
    }

    static IloCplex GRIP2(int[] yCoverCf, int[][] yCoverCfPair,
                          int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightType, int GateTypeDivideline,
                          int[][][][] AirpTypeLegnumPattern, int[][] AirpTypePatternNum)
            throws IloException {

        int CfNum = CopyFlightOFID.length;
        int ApNum = AirpTypeLegnumPattern.length;
        int ConNum = yCoverCfPair.length;

        IloCplex cplex = new IloCplex();

        //定义变量
        IloIntVar[][][] w = new IloIntVar[ApNum][2][];


        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                String[] wname = new String[AirpTypePatternNum[ap][tp]];
                for(int i=0; i<AirpTypePatternNum[ap][tp]; i++){
                    wname[i]="w"+ap +","+ tp +","+ i;
                }
                w[ap][tp] = cplex.intVarArray(AirpTypePatternNum[ap][tp],0,1, wname);
            }
        }


        //目标函数：cost和最小的特例->使用的停机位数尽可能少
//        IloNumExpr obj = cplex.prod(2,cplex.sum(wc));
        IloIntExpr obj = cplex.prod(0,w[0][0][0]);
        for(int ap=0; ap<ApNum; ap++){
            for(int tp=0; tp<2; tp++){
                obj=cplex.sum(obj, cplex.sum(w[ap][tp]));
            }
        }
        cplex.addMinimize(cplex.prod(GateCost, obj));

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[CfNum];
        rng[1] = new IloRange[CfNum];
        rng[2] = new IloRange[ConNum];
        rng[3] = new IloRange[ApNum*2];


        //约束：复制航班的决策与SARP统一  形式1
        IloNumExpr[][] coverFlightDA = new IloNumExpr[CfNum][2];
        for(int cf=0; cf<CfNum; cf++) {
            coverFlightDA[cf][0] = cplex.prod(0, w[0][0][0]);
            coverFlightDA[cf][1] = cplex.prod(0, w[0][0][0]);
        }
        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]; j++){
                        int cf = AirpTypeLegnumPattern[ap][tp][i][1+j];
                        if(ap==CopyFlightDepS[cf]){
                            coverFlightDA[cf][0] = cplex.sum(coverFlightDA[cf][0], w[ap][tp][i]);
                        }
                        if(ap==CopyFlightArrS[cf]){
                            coverFlightDA[cf][1] = cplex.sum(coverFlightDA[cf][1], w[ap][tp][i]);
                        }
                    }
                }
            }
        }
        for(int cf=0; cf<CfNum; cf++) {
            rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
            rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);

//            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
//                rng[0][cf] = cplex.addEq(coverFlightDA[cf][0], yCoverCf[cf]);
//                rng[1][cf] = cplex.addEq(coverFlightDA[cf][1], yCoverCf[cf]);
//            }else{
//                rng[0][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//                rng[1][cf] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
//            }

        }


        //约束：保证一架飞机进出的是同一个Gate yCoverCfPair[con][2]
        IloNumExpr[] coverCon = new IloNumExpr[ConNum];
        for(int con=0; con<ConNum; con++){
            coverCon[con] = cplex.prod(0, w[0][0][0]);
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                for(int j=0; j<AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(    AirpTypeLegnumPattern[ap][tp][i][1+j] == yCoverCfPair[con][0]
                            && AirpTypeLegnumPattern[ap][tp][i][1+j+1] == yCoverCfPair[con][1]){
                        coverCon[con] = cplex.sum(coverCon[con], w[ap][tp][i]);
                        break;
                    }
                }
            }
//            rng[2][con] = cplex.addEq(cplex.sum(coverCon[con],wc[con]), 1);
            if(  CopyFlightDepS[yCoverCfPair[con][0]]!=CopyFlightArrS[yCoverCfPair[con][0]]
                    && CopyFlightDepS[yCoverCfPair[con][1]]!=CopyFlightArrS[yCoverCfPair[con][1]]
            ){
                rng[2][con] = cplex.addEq(coverCon[con], 1);
            }else{
                rng[2][con] = cplex.addEq(0, cplex.prod(0, w[0][0][0]));
            }

        }

        for(int ap=0; ap<ApNum; ap++) {
            for(int tp=0; tp<2; tp++){
                IloNumExpr gatecap = cplex.prod(0, w[0][0][0]);
                for(int i=0; i< AirpTypePatternNum[ap][tp]; i++){
                    gatecap = cplex.sum(gatecap, w[ap][tp][i]);
                }
                rng[3][ap*2+tp] = cplex.addGe(GateCap, cplex.prod(0, gatecap));
            }
        }

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_15minTiaoTime10\\Data\\9\\GR.lp");

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.setWarning(null);
        cplex.setOut(null);

        return cplex;
    }

    static int[][] forcedCoveredPairInitialPattern(double forcedCoveredPairInitialPatternTimeLim, int GateAirpID, int GateType, int NumberOfInitialPattern, int LegNum, int forcedCoveredCF, int[] forcedCoveredPair,
                                                   int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                                   int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                                   int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT,int[] CopyFlightArrT
    ) throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }

        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //强制覆盖FlightOrignIndex
        if (forcedCoveredCF != -1) {
            cp.addGe(cp.count(flightvars, forcedCoveredCF), 1);
        }

        //强制覆盖forcedCoveredPair??
        IloIntExpr[] PairFlag = cp.intExprArray(maxlegs);
        PairFlag[0] = cp.prod(0, isactual[0]);
        for (int i = 1; i < maxlegs; i++) {
            PairFlag[i] = cp.prod(cp.eq(flightvars[i-1], forcedCoveredPair[0]), cp.eq(flightvars[i], forcedCoveredPair[1]));
            PairFlag[i] = cp.prod(PairFlag[i], isactual[i]);
        }
        cp.addGe(cp.sum(PairFlag),1);



        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }

        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setWarning(null);
        cp.setParameter(IloCP.IntParam.Workers, 12);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit, forcedCoveredPairInitialPatternTimeLim);
        cp.startNewSearch();

        int[][] ActualPattern = new int[NumberOfInitialPattern][maxlegs+1];

//        int findflag = 0;
        for (int n = 0; n < NumberOfInitialPattern; n++) {
            if (cp.next()) {
//                findflag += 1;
                int numisactual = 0;
                for (int i = 0; i < maxlegs; i++) {
                    if (cp.getValue(isactual[i]) != 1) {
                        break;
                    }
                    numisactual +=1;
                    ActualPattern[n][i+1]=(int)cp.getValue(flightvars[i]);
                }
                ActualPattern[n][0] = numisactual;
//                System.out.println(Arrays.toString(ActualPattern[n]));
            }else{
//                System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");

                cp.end();
                return ActualPattern;
            }
        }
//        System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");
        cp.end();
        return ActualPattern;
    }

    static double[] OneBestPattern(double OneBestPatternTimeLim, double[][] LpDualValue, int[][] yCoverCfPair,
                                   int GateAirpID, int GateType, int LegNum, int forcedCoveredCF,
                                   int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                   int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                   int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT,int[] CopyFlightArrT)
            throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }


        IloNumExpr obj = cp.sum(cp.prod(0, isactual[0]), GateCost + LpDualValue[3][GateAirpID*2+GateType]);

        //使用copyf的reduced cost
        double[] cfDepDual = new double[LpDualValue[0].length];
        double[] cfArrDual = new double[LpDualValue[1].length];
        for(int cf=0; cf<CopyFlightArrS.length; cf++){
//            cfDepDual[cf]=LpDualValue[0][cf];
//            cfArrDual[cf]=LpDualValue[1][cf];
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                cfDepDual[cf]=LpDualValue[0][cf];
                cfArrDual[cf]=LpDualValue[1][cf];
            }else{
                cfDepDual[cf]=0;
                cfArrDual[cf]=0;
            }
        }
        for(int i=0; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.sum(cp.prod(cp.element(cfDepDual, flightvars[i]), cp.eq(cp.element(CopyFlightDepS, flightvars[i]), GateAirpID)),
                            cp.prod(cp.element(cfArrDual, flightvars[i]), cp.eq(cp.element(CopyFlightArrS, flightvars[i]), GateAirpID))),
                            cp.ge(isactual[i], 1) )
            );
        }

        //使用copyf pair的reduced cost
        int CfNum = CopyFlightOFID.length;
        double[] pairDual = new double[CfNum*CfNum];
        for(int i=0; i<yCoverCfPair.length; i++){
//            pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            if(CopyFlightDepS[yCoverCfPair[i][0]]!=CopyFlightArrS[yCoverCfPair[i][0]]
                    && CopyFlightDepS[yCoverCfPair[i][1]]!=CopyFlightArrS[yCoverCfPair[i][1]]
            ){
                pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            }
        }
        for(int i=1; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod(cp.element(pairDual, cp.sum(cp.prod(CfNum, flightvars[i-1]), flightvars[i])),
                            cp.ge(isactual[i], 1))
            );
        }

        cp.addMinimize(obj);
        cp.addLe(obj, -epson);


        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }


        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setWarning(null);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,OneBestPatternTimeLim);

        double[] ReducedcostLegnumPattern = new double[2+maxlegs];

        if (cp.solve()) {
            int numisactual = 0;
            for (int i = 0; i < maxlegs; i++) {
                if (cp.getValue(isactual[i]) != 1) {
                    break;
                }
                numisactual +=1;
                ReducedcostLegnumPattern[2+i]= cp.getValue(flightvars[i]);
            }
            ReducedcostLegnumPattern[0] = cp.getValue(obj);
            ReducedcostLegnumPattern[1] = numisactual;

            cp.end();
            return ReducedcostLegnumPattern;
        }

        cp.end();
        return ReducedcostLegnumPattern;
    }

    static double[] phase1OneBestPattern(double[][] LpDualValue, int[][] yCoverCfPair,
                                         int GateAirpID, int GateType, int LegNum, int forcedCoveredCF,
                                         int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                         int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                         int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT,int[] CopyFlightArrT)
            throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }


        IloNumExpr obj = cp.sum(cp.prod(0, isactual[0]), LpDualValue[3][GateAirpID*2+GateType]);

        //使用copyf的reduced cost
        double[] cfDepDual = new double[LpDualValue[0].length];
        double[] cfArrDual = new double[LpDualValue[1].length];
        for(int cf=0; cf<CopyFlightArrS.length; cf++){
//            cfDepDual[cf]=LpDualValue[0][cf];
//            cfArrDual[cf]=LpDualValue[1][cf];
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                cfDepDual[cf]=LpDualValue[0][cf];
                cfArrDual[cf]=LpDualValue[1][cf];
            }else{
                cfDepDual[cf]=0;
                cfArrDual[cf]=0;
            }
        }
        for(int i=0; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.sum(cp.prod(cp.element(cfDepDual, flightvars[i]), cp.eq(cp.element(CopyFlightDepS, flightvars[i]), GateAirpID)),
                            cp.prod(cp.element(cfArrDual, flightvars[i]), cp.eq(cp.element(CopyFlightArrS, flightvars[i]), GateAirpID))),
                            cp.ge(isactual[i], 1) )
            );
        }

        //使用copyf pair的reduced cost
        int CfNum = CopyFlightOFID.length;
        double[] pairDual = new double[CfNum*CfNum];
        for(int i=0; i<yCoverCfPair.length; i++){
//            pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            if(CopyFlightDepS[yCoverCfPair[i][0]]!=CopyFlightArrS[yCoverCfPair[i][0]]
                    && CopyFlightDepS[yCoverCfPair[i][1]]!=CopyFlightArrS[yCoverCfPair[i][1]]
            ){
                pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            }
        }
        for(int i=1; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod(cp.element(pairDual, cp.sum(cp.prod(CfNum, flightvars[i-1]), flightvars[i])),
                            cp.ge(isactual[i], 1))
            );
        }

        cp.addMinimize(obj);
        cp.addLe(obj, -epson);


        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }


        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setWarning(null);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,3);

        double[] ReducedcostLegnumPattern = new double[2+maxlegs];

        if (cp.solve()) {
            int numisactual = 0;
            for (int i = 0; i < maxlegs; i++) {
                if (cp.getValue(isactual[i]) != 1) {
                    break;
                }
                numisactual +=1;
                ReducedcostLegnumPattern[2+i]= cp.getValue(flightvars[i]);
            }
            ReducedcostLegnumPattern[0] = cp.getValue(obj);
            ReducedcostLegnumPattern[1] = numisactual;

            cp.end();
            return ReducedcostLegnumPattern;
        }

        cp.end();
        return ReducedcostLegnumPattern;
    }



    static int[][] NegativeReducedCostPattern(double NegativeReducedCostPatternTimeLim, double[][] LpDualValue, int[][] yCoverCfPair,
                                              int GateAirpID, int GateType, int NumberOfInitialPattern, int LegNum, int forcedCoveredCF,
                                              int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                              int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                              int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT)
            throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }

        IloNumExpr obj = cp.sum(cp.prod(0, isactual[0]), GateCost + LpDualValue[3][GateAirpID*2+GateType]);

        //使用copyf的reduced cost
        double[] cfDepDual = new double[LpDualValue[0].length];
        double[] cfArrDual = new double[LpDualValue[1].length];
        for(int cf=0; cf<CopyFlightArrS.length; cf++){
//            cfDepDual[cf]=LpDualValue[0][cf];
//            cfArrDual[cf]=LpDualValue[1][cf];
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                cfDepDual[cf]=LpDualValue[0][cf];
                cfArrDual[cf]=LpDualValue[1][cf];
            }else{
                cfDepDual[cf]=0;
                cfArrDual[cf]=0;
            }
        }
        for(int i=0; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.sum(cp.prod(cp.element(cfDepDual, flightvars[i]), cp.eq(cp.element(CopyFlightDepS, flightvars[i]), GateAirpID)),
                            cp.prod(cp.element(cfArrDual, flightvars[i]), cp.eq(cp.element(CopyFlightArrS, flightvars[i]), GateAirpID))),
                            cp.ge(isactual[i], 1) )
            );
        }

        //使用copyf pair的reduced cost
        int CfNum = CopyFlightOFID.length;
        double[] pairDual = new double[CfNum*CfNum];
        for(int i=0; i<yCoverCfPair.length; i++){
//            pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            if(CopyFlightDepS[yCoverCfPair[i][0]]!=CopyFlightArrS[yCoverCfPair[i][0]]
                    && CopyFlightDepS[yCoverCfPair[i][1]]!=CopyFlightArrS[yCoverCfPair[i][1]]
            ){
                pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            }
        }
        for(int i=1; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod(cp.element(pairDual, cp.sum(cp.prod(CfNum, flightvars[i-1]), flightvars[i])),
                            cp.ge(isactual[i], 1))
            );
        }

        cp.addLe(obj, -epson);



        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //强制覆盖FlightOrignIndex
        if (forcedCoveredCF != -1) {
            cp.addGe(cp.count(flightvars, forcedCoveredCF), 1);
        }

        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }

        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,NegativeReducedCostPatternTimeLim);
        cp.startNewSearch();

        int[][] ActualPattern = new int[NumberOfInitialPattern][maxlegs+1];

        int findflag = 0;
        for (int n = 0; n < NumberOfInitialPattern; n++) {
            if (cp.next()) {
                findflag += 1;
                int numisactual = 0;
                for (int i = 0; i < maxlegs; i++) {
                    if (cp.getValue(isactual[i]) != 1) {
                        break;
                    }
                    numisactual +=1;
                    ActualPattern[n][i+1]=(int)cp.getValue(flightvars[i]);
                }
                ActualPattern[n][0] = numisactual;
//                System.out.println(Arrays.toString(ActualPattern[n]));
            }else{
//                System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");

                cp.end();
                return ActualPattern;
            }
        }
//        System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");
        cp.end();
        return ActualPattern;
    }

    static int[][] phase1NegativeReducedCostPattern(double[][] LpDualValue, int[][] yCoverCfPair,
                                                    int GateAirpID, int GateType, int NumberOfInitialPattern, int LegNum, int forcedCoveredCF,
                                                    int GateTypeDivideline, int[] AirpTypeCFID, int AirpTypeCFNum,
                                                    int[] FlightType, int[] FlightDepS, int[] FlightArrS,
                                                    int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT)
            throws  IloException {

        int maxlegs = MaxPatternLegNum;
        int TurnTime = 30;

        IloCP cp = new IloCP();

        IloIntVar[] flightvarsIndex = cp.intVarArray(maxlegs, 0, AirpTypeCFNum-1);
        IloIntVar[] isactual = cp.intVarArray(maxlegs, 0, 1);
        IloIntExpr[] flightvars = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsOFID = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvars[i] = cp.element(AirpTypeCFID, flightvarsIndex[i]);
            IloIntExpr flag = cp.sum(cp.prod(2, isactual[i]), -1);
            flightvarsOFID[i] = cp.prod(cp.element(CopyFlightOFID, flightvars[i]), flag);
        }

        IloNumExpr obj = cp.sum(cp.prod(0, isactual[0]), LpDualValue[3][GateAirpID*2+GateType]);

        //使用copyf的reduced cost
        double[] cfDepDual = new double[LpDualValue[0].length];
        double[] cfArrDual = new double[LpDualValue[1].length];
        for(int cf=0; cf<CopyFlightArrS.length; cf++){
//            cfDepDual[cf]=LpDualValue[0][cf];
//            cfArrDual[cf]=LpDualValue[1][cf];
            if(CopyFlightDepS[cf]!=CopyFlightArrS[cf]){
                cfDepDual[cf]=LpDualValue[0][cf];
                cfArrDual[cf]=LpDualValue[1][cf];
            }else{
                cfDepDual[cf]=0;
                cfArrDual[cf]=0;
            }
        }
        for(int i=0; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.sum(cp.prod(cp.element(cfDepDual, flightvars[i]), cp.eq(cp.element(CopyFlightDepS, flightvars[i]), GateAirpID)),
                            cp.prod(cp.element(cfArrDual, flightvars[i]), cp.eq(cp.element(CopyFlightArrS, flightvars[i]), GateAirpID))),
                            cp.ge(isactual[i], 1) )
            );
        }

        //使用copyf pair的reduced cost
        int CfNum = CopyFlightOFID.length;
        double[] pairDual = new double[CfNum*CfNum];
        for(int i=0; i<yCoverCfPair.length; i++){
//            pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            if(CopyFlightDepS[yCoverCfPair[i][0]]!=CopyFlightArrS[yCoverCfPair[i][0]]
                    && CopyFlightDepS[yCoverCfPair[i][1]]!=CopyFlightArrS[yCoverCfPair[i][1]]
            ){
                pairDual[CfNum*yCoverCfPair[i][0]+yCoverCfPair[i][1]] = LpDualValue[2][i];
            }
        }
        for(int i=1; i<maxlegs; i++){
            obj = cp.diff(obj,
                    cp.prod(cp.element(pairDual, cp.sum(cp.prod(CfNum, flightvars[i-1]), flightvars[i])),
                            cp.ge(isactual[i], 1))
            );
        }

        cp.addLe(obj, -epson);



        //同一航班的复制边最多只能选一条...
        if(LegNum!=1){
            for(int f=0; f<FlightType.length; f++){
                if(FlightDepS[f]==GateAirpID||FlightArrS[f]==GateAirpID){
                    if((FlightType[f]<GateTypeDivideline?0:1)==GateType){
                        cp.addLe(cp.count(flightvarsOFID, f),1);
                    }
                }
            }
        }

        //强制覆盖FlightOrignIndex
        if (forcedCoveredCF != -1) {
            cp.addGe(cp.count(flightvars, forcedCoveredCF), 1);
        }

        //LegNum约束
        if(LegNum!=-1){
            cp.addEq(cp.sum(isactual), LegNum);
        }else{
            cp.addGe(cp.sum(isactual), 2);
        }

        //变量关系约束
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.ge(isactual[i - 1], isactual[i]));
        }
        for (int i = 1; i < maxlegs; i++) {
            cp.add(cp.eq(isactual[i], cp.neq(flightvars[i], flightvars[i - 1])));
        }

        IloIntExpr[] flightvarsDepS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsDepT = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrS = cp.intExprArray(maxlegs);
        IloIntExpr[] flightvarsArrT = cp.intExprArray(maxlegs);
        for (int i = 0; i < maxlegs; i++) {
            flightvarsDepS[i] = cp.element(CopyFlightDepS, flightvars[i]);
            flightvarsDepT[i] = cp.element(CopyFlightDepT, flightvars[i]);
            flightvarsArrS[i] = cp.element(CopyFlightArrS, flightvars[i]);
            flightvarsArrT[i] = cp.element(CopyFlightArrT, flightvars[i]);
        }

        //时间衔接约束
        for (int i = 1; i < maxlegs; i++) {
            IloIntExpr BeforeTime = cp.sum(
                    cp.prod(cp.neq(flightvarsDepS[i - 1],flightvarsArrS[i - 1]),
                            cp.prod(cp.eq(flightvarsDepS[i - 1], GateAirpID), cp.sum(flightvarsDepT[i - 1],TurnTime))),
                    cp.prod(cp.eq(flightvarsArrS[i - 1], GateAirpID), cp.sum(flightvarsArrT[i - 1],TurnTime))
            );
            IloIntExpr NowTime = cp.sum(
                    cp.prod(cp.eq(flightvarsDepS[i], GateAirpID), flightvarsDepT[i] ),
                    cp.prod(cp.neq(flightvarsDepS[i],flightvarsArrS[i]),
                            cp.prod(cp.eq(flightvarsArrS[i], GateAirpID), flightvarsArrT[i]))
            );
            cp.add(cp.or(cp.eq(isactual[i], 0),
                    cp.ge(cp.diff(NowTime, BeforeTime), 0)
            ));
        }


        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.DepthFirst);
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Restart);
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.Auto);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);
        cp.startNewSearch();

        int[][] ActualPattern = new int[NumberOfInitialPattern][maxlegs+1];

        int findflag = 0;
        for (int n = 0; n < NumberOfInitialPattern; n++) {
            if (cp.next()) {
                findflag += 1;
                int numisactual = 0;
                for (int i = 0; i < maxlegs; i++) {
                    if (cp.getValue(isactual[i]) != 1) {
                        break;
                    }
                    numisactual +=1;
                    ActualPattern[n][i+1]=(int)cp.getValue(flightvars[i]);
                }
                ActualPattern[n][0] = numisactual;
//                System.out.println(Arrays.toString(ActualPattern[n]));
            }else{
//                System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");

                cp.end();
                return ActualPattern;
            }
        }
//        System.out.println("AirpID:"+GateAirpID +",GateType:"+GateType+",LegNum:"+LegNum+",forcedCoveredCF:"+forcedCoveredCF+",find " +findflag+ " pattern(s)\n");
        cp.end();
        return ActualPattern;
    }

    static double[][] MPLP(  double[][][] BendersOptCutXishu,
                             int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                             int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                             int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloNumVar[][] y = new IloNumVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.numVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloNumVar[] z = cplex.numVarArray(FlightNum, 0, 1, zname);

        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");


        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloNumExpr CancelNum = cplex.prod(1, z[0]);
        for(int f=1; f<FlightNum; f++){
            CancelNum = cplex.sum(CancelNum, z[f]);
        }

        IloNumExpr DelayTime = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                DelayTime = cplex.sum(DelayTime, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][1], y[airc][i]));
            }
        }

        IloNumExpr SwapNum = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                SwapNum = cplex.sum(SwapNum, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][2], y[airc][i]));
            }
        }

        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf = cplex.prod(1, z[f]);
            for(int airc=0; airc<AircNum; airc++){
                for(int i=0; i<AircRouteNum[airc]; i++){
                    for(int j=3; j<3+AircLegnumDlytimeSwapnumRoute[airc][i][0]; j++){
                        if(CopyFlightOFID[AircLegnumDlytimeSwapnumRoute[airc][i][j]] == f){
                            coverf = cplex.sum(coverf, y[airc][i]);
                            break;
                        }
                    }
                }
            }
            rng[0][f] = cplex.addEq(coverf, 1);
        }


        for(int ap = 0; ap< ApNum; ap++){
            for(int slot=0; slot< SlotNum; slot++){
                IloNumExpr apslot = cplex.prod(0, z[0]);
                for(int airc=0; airc<AircNum; airc++){
                    for(int i=0; i<AircRouteNum[airc]; i++){
                        for(int j=3; j<3+AircLegnumDlytimeSwapnumRoute[airc][i][0]; j++){
                            if(     (CopyFlightDepS[AircLegnumDlytimeSwapnumRoute[airc][i][j]]==ap) &&
                                    (CopyFlightDepT[AircLegnumDlytimeSwapnumRoute[airc][i][j]]>= 60*slot) &&
                                    (CopyFlightDepT[AircLegnumDlytimeSwapnumRoute[airc][i][j]]< 60*(slot+1)) &&
                                    (CopyFlightDepS[AircLegnumDlytimeSwapnumRoute[airc][i][j]]!=CopyFlightArrS[AircLegnumDlytimeSwapnumRoute[airc][i][j]])
                            ){
                                apslot = cplex.sum(apslot, y[airc][i]);
                            }
                        }
                    }
                }
//                rng[1][ap*SlotNum+slot] = cplex.addLe(apslot, ApSlotDep[ap][slot]);
                rng[1][ap*SlotNum+slot] = cplex.addGe(cplex.prod(-1,apslot), -ApSlotDep[ap][slot]);
            }
        }


        for(int ap = 0; ap< ApNum; ap++){
            for(int slot=0; slot< SlotNum; slot++){
                IloNumExpr apslot = cplex.prod(0, z[0]);
                for(int airc=0; airc<AircNum; airc++){
                    for(int i=0; i<AircRouteNum[airc]; i++){
                        for(int j=3; j<3+AircLegnumDlytimeSwapnumRoute[airc][i][0]; j++){
                            if(     (CopyFlightArrS[AircLegnumDlytimeSwapnumRoute[airc][i][j]]==ap) &&
                                    (CopyFlightArrT[AircLegnumDlytimeSwapnumRoute[airc][i][j]]>= 60*slot) &&
                                    (CopyFlightArrT[AircLegnumDlytimeSwapnumRoute[airc][i][j]]< 60*(slot+1)) &&
                                    (CopyFlightDepS[AircLegnumDlytimeSwapnumRoute[airc][i][j]]!=CopyFlightArrS[AircLegnumDlytimeSwapnumRoute[airc][i][j]])
                            ){
                                apslot = cplex.sum(apslot, y[airc][i]);
                            }
                        }
                    }
                }
//                rng[2][ap*SlotNum+slot] = cplex.addLe(apslot, ApSlotArr[ap][slot]);
                rng[2][ap*SlotNum+slot] = cplex.addGe(cplex.prod(-1,apslot), -ApSlotArr[ap][slot]);
            }
        }


        for(int airc=0; airc<AircNum; airc++){
            IloNumExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        //double[][][] BendersOptCutXishu
        for(int c=0; c< BendersOptCutXishu.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r=0; r<BendersOptCutXishu[c][airc].length; r++){
                    expr.addTerm(y[airc][r], BendersOptCutXishu[c][airc][r]);
                }
            }
            cplex.addLe(expr,q);
        }


//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
//        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj= " + cplex.getObjValue());

        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.print("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));



        double[][] LpDualValue = new double[4][];
        for(int i=0; i<4; i++){
            LpDualValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }


        cplex.end();
        return LpDualValue;
    }

    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException, IloException {

        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;
        int gateTypeNum = 1;
        int GateTypeDivideline = 0;
        if(DataBase==1){
            ActFlightNum = 24;
            MtsNum = 1;
            AircNum = 4;
            TypeNum = 2;
            ApNum = 7;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 1;
            }
        }else if(DataBase==2){
            ActFlightNum = 52;
            MtsNum = 1;
            AircNum = 8;
            TypeNum = 3;
            ApNum = 9;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
            }else{
                GateTypeDivideline = 1;
            }
        }else if(DataBase==3){
            ActFlightNum = 98;
            MtsNum = 2;
            AircNum = 15;
            TypeNum = 3;
            ApNum = 14;
        }else if(DataBase==4){
            ActFlightNum = 150;
            MtsNum = 2;
            AircNum = 24;
            TypeNum = 4;
            ApNum = 18;
        }else if(DataBase==5){
            ActFlightNum = 198;
            MtsNum = 3;
            AircNum = 32;
            TypeNum = 4;
            ApNum = 17;
        }else if(DataBase==6){
            ActFlightNum = 250;
            MtsNum = 3;
            AircNum = 41;
            TypeNum = 4;
            ApNum = 22;
        }else if(DataBase==7){
            ActFlightNum = 295;
            MtsNum = 3;
            AircNum = 49;
            TypeNum = 5;
            ApNum = 26;
        }else if(DataBase==8){
            ActFlightNum = 359;
            MtsNum = 3;
            AircNum = 61;
            TypeNum = 6;
            ApNum = 28;
        }else if(DataBase==9){
            ActFlightNum = 395;
            MtsNum = 3;
            AircNum = 68;
            TypeNum = 8;
            ApNum = 30;
        }else if(DataBase==10){
            ActFlightNum = 464;
            MtsNum = 3;
            AircNum = 81;
            TypeNum = 11;
            ApNum = 35;
        }

        InputDataReader FlightReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Flight.txt");
        int[][] Flight = FlightReader.readIntArrayArray();
        int FlightNum = Flight.length;

        int[] FlightID = new int[FlightNum];
        int[] FlightDepS = new int[FlightNum];
        int[] FlightArrS = new int[FlightNum];
        int[] FlightDepT = new int[FlightNum];
        int[] FlightArrT = new int[FlightNum];
        int[] FlightType = new int[FlightNum];
        int[] FlightOAID = new int[FlightNum];

        int[] FlightFlyT = new int[FlightNum];
        for(int f=0; f<FlightNum; f++){
            FlightID[f] = Flight[f][0];
            FlightDepS[f] = Flight[f][1];
            FlightArrS[f] = Flight[f][2];
            FlightDepT[f] = Flight[f][3];
            FlightArrT[f] = Flight[f][4];
            FlightType[f] = Flight[f][5];
            FlightOAID[f] = Flight[f][6];

            FlightFlyT[f] = FlightArrT[f] - FlightDepT[f];
        }

        int[][] CopyFlight = new int[ActFlightNum*CopyNum+MtsNum][7];
        int CopyFlightNum = CopyFlight.length;
        int CopyFlag=0;
        for(int f=0; f<Flight.length; f++){
            if(Flight[f][1]!=Flight[f][2]){
                for(int i=0; i<CopyNum; i++){
                    for(int j=0; j<7;j++)
                        CopyFlight[CopyFlag][j] = Flight[f][j];
                    CopyFlight[CopyFlag][3] = Flight[f][3] + OneDelayTime*i;
                    CopyFlight[CopyFlag][4] = Flight[f][4] + OneDelayTime*i;
                    CopyFlag += 1;
                }
            }else{
                for(int j=0; j<7;j++)
                    CopyFlight[CopyFlag][j] = Flight[f][j];
                CopyFlight[CopyFlag][3] = Flight[f][3];
                CopyFlight[CopyFlag][4] = Flight[f][4];
                CopyFlag += 1;
            }
        }

        InputDataReader AircReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Aircraft.txt");
        int[][] Aircraft = AircReader.readIntArrayArray();

        DataReader ApSlotDepReader = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\ApSlotDep.txt");
        DataReader ApSlotArrReader = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\ApSlotArr.txt");
        int[][] ApSlotDep = new int[ApNum][SlotNum];
        int[][] ApSlotArr = new int[ApNum][SlotNum];
        for (int ap = 0; ap < ApNum; ap++) {
            for(int t=0; t<SlotNum; t++){
                ApSlotDep[ap][t] = ApSlotDepReader.next();
                ApSlotArr[ap][t] = ApSlotArrReader.next();
            }
        }

        int[] CopyFlightOFID = new int[CopyFlight.length];
        int[] CopyFlightDepS = new int[CopyFlight.length];
        int[] CopyFlightArrS = new int[CopyFlight.length];
        int[] CopyFlightDepT = new int[CopyFlight.length];
        int[] CopyFlightArrT = new int[CopyFlight.length];
        int[] CopyFlightType = new int[CopyFlight.length];
        int[] CopyFlightOAID = new int[CopyFlight.length];
        int[] CopyFlightFlyT = new int[CopyFlight.length];
        int[] CopyFlightDlyMint = new int[CopyFlight.length];
        int[][] TypeCopyFlightRange= new int[TypeNum][2];
        int[] TypeMtsNum = new int[TypeNum];
        int[] CopyFlightDepTSlot = new int[CopyFlight.length];
        int[] CopyFlightArrTSlot = new int[CopyFlight.length];
        for(int tp=0; tp<TypeNum; tp++){
            TypeCopyFlightRange[tp][0] = CopyFlight.length;
            TypeCopyFlightRange[tp][1] = 0;
        }
        for(int cf=0; cf<CopyFlight.length; cf++){
            CopyFlightOFID[cf] = CopyFlight[cf][0];
            CopyFlightDepS[cf] = CopyFlight[cf][1];
            CopyFlightArrS[cf] = CopyFlight[cf][2];
            CopyFlightDepT[cf] = CopyFlight[cf][3];
            CopyFlightArrT[cf] = CopyFlight[cf][4];
            CopyFlightType[cf] = CopyFlight[cf][5];
            CopyFlightOAID[cf] = CopyFlight[cf][6];
            if(cf<TypeCopyFlightRange[CopyFlightType[cf]][0]){
                TypeCopyFlightRange[CopyFlightType[cf]][0]=cf;
            }
            if(cf>TypeCopyFlightRange[CopyFlightType[cf]][1]){
                TypeCopyFlightRange[CopyFlightType[cf]][1]=cf;
            }
            CopyFlightFlyT[cf] = CopyFlightArrT[cf] - CopyFlightDepT[cf];
            CopyFlightDlyMint[cf] = CopyFlightDepT[cf] - Flight[CopyFlightOFID[cf]][3];
            if(CopyFlightDepS[cf]==CopyFlightArrS[cf]){
                TypeMtsNum[CopyFlightType[cf]] += 1;
            }
            CopyFlightDepTSlot[cf] = (int) CopyFlightDepT[cf]/60;
            CopyFlightArrTSlot[cf] = (int) CopyFlightArrT[cf]/60;
        }

        int[][] TypeMtsCFID = new int[TypeNum][];
        for(int tp=0; tp<TypeNum; tp++){
            TypeMtsCFID[tp] = new int[TypeMtsNum[tp]];
            if(TypeMtsNum[tp]>0){
                int flag = 0;
                for(int cf=TypeCopyFlightRange[tp][0]; cf<=TypeCopyFlightRange[tp][1]; cf++){
                    if(CopyFlightDepS[cf]==CopyFlightArrS[cf]){
                        TypeMtsCFID[tp][flag]=cf;
                        flag+=1;
                        if(flag==TypeMtsNum[tp])
                            break;
                    }
                }
            }
        }

        int[] AircID = new int[AircNum];
        int[] AircType = new int[AircNum];
        int[] AircStartS = new int[AircNum];
        int[] AircEndS = new int[AircNum];
        int[] AircTurnT = new int[AircNum];
        int[] AircMts = new int[AircNum];

        for(int airc=0; airc<AircNum; airc++){
            AircID[airc] = Aircraft[airc][0];
            AircType[airc] = Aircraft[airc][1];
            AircStartS[airc] = Aircraft[airc][2];
            AircEndS[airc] = Aircraft[airc][3];
            AircTurnT[airc] = Aircraft[airc][4];
            AircMts[airc] = Aircraft[airc][5];
        }

        int[] AircRouteNum = new int[AircNum];
        int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];

        DataReader RouteNumR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\RouteNum100.txt");
        DataReader RouteR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Route100.txt");
        for (int airc = 0; airc < AircNum; airc++) {
            AircRouteNum[airc] = RouteNumR.next();
            AircLegnumDlytimeSwapnumRoute[airc] = new int[AircRouteNum[airc]][3+MaxRouteLegNum];
        }
        for (int airc = 0; airc < AircNum; airc++) {
            for (int i = 0; i < AircRouteNum[airc]; i++){
                for(int j=0; j<3+MaxRouteLegNum; j++){
                    AircLegnumDlytimeSwapnumRoute[airc][i][j] = RouteR.next();
                }
            }
        }

        int[][] AirpTypePatternNum = new int[ApNum][2];
        int[][][][] AirpTypeLegnumPattern = new int[ApNum][2][][];
        DataReader InitialPatternNumR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\InitialPatternNum1000~"+(3-GateTypeDivideline)+".txt");
        DataReader InitialPatternR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\InitialPattern1000~"+(3-GateTypeDivideline)+".txt");
        for(int ap=0; ap< ApNum; ap++) {
            for (int tp = 0; tp < 2; tp++) {
                AirpTypePatternNum[ap][tp] = InitialPatternNumR.next();
                AirpTypeLegnumPattern[ap][tp] = new int[AirpTypePatternNum[ap][tp]][1+MaxPatternLegNum];
            }
        }
        for(int ap=0; ap< ApNum; ap++) {
            for (int tp = 0; tp < 2; tp++) {
                for (int i = 0; i < AirpTypePatternNum[ap][tp]; i++) {
                    for(int j=0; j<1+MaxPatternLegNum; j++){
                        AirpTypeLegnumPattern[ap][tp][i][j] = InitialPatternR.next();
                    }
                }
            }
        }



        //机场相关数据导入 各机场各类型停机位是否使用
        int[][][] AirpTypeCFID = new int[ApNum][2][CopyFlightNum];
        int[][] AirpTypeCFNum = new int[ApNum][2];
        int[][] AirpTypeFNum = new int[ApNum][2];
        for(int ap=0; ap<ApNum; ap++){
            for(int cf=0; cf<CopyFlightNum; cf++){
                if((CopyFlightDepS[cf]==ap||CopyFlightArrS[cf]==ap) && CopyFlightType[cf]<GateTypeDivideline){
                    AirpTypeCFID[ap][0][AirpTypeCFNum[ap][0]]=cf;
                    AirpTypeCFNum[ap][0]+=1;
                }
                if((CopyFlightDepS[cf]==ap||CopyFlightArrS[cf]==ap) && CopyFlightType[cf]>=GateTypeDivideline){
                    AirpTypeCFID[ap][1][AirpTypeCFNum[ap][1]]=cf;
                    AirpTypeCFNum[ap][1]+=1;
                }
            }
            for(int f=0; f<FlightNum; f++){
                if((FlightDepS[f]==ap||FlightArrS[f]==ap) && FlightType[f]<GateTypeDivideline){
                    AirpTypeFNum[ap][0]+=1;
                }
                if((FlightDepS[f]==ap||FlightArrS[f]==ap) && FlightType[f]>=GateTypeDivideline){
                    AirpTypeFNum[ap][1]+=1;
                }
            }
        }

        //给定SARP的求解GRP
        int[] y = {74, 100, 26, 41};

        //筛选出y覆盖的CopyFlight和CopyFlightPair
        int[] fltsOpt = new int[FlightNum];
        int[][][] conOpt = new int[AircNum][][];
        for(int i = 0; i<FlightNum; i++){
            fltsOpt[i]=-1;
        }

        int conNum = 0;
        for(int airc=0; airc<AircNum; airc++){
            int[] chooseRoute = AircLegnumDlytimeSwapnumRoute[airc][y[airc]];
            int[][] aircConOpt = new int[chooseRoute[0]-1][2];
            for(int i=0; i<chooseRoute[0]; i++){
                fltsOpt[CopyFlightOFID[chooseRoute[3+i]]] = chooseRoute[3+i];
                if(i<chooseRoute[0]-1){
                    aircConOpt[i][0]= chooseRoute[3+i];
                    aircConOpt[i][1]= chooseRoute[3+i+1];
                }
            }
            conOpt[airc] = aircConOpt;
            conNum += chooseRoute[0] -1;
        }

        int[] yCoverCf = new int[CopyFlightNum];
        for(int f = 0; f<FlightNum; f++){
            if(fltsOpt[f]!=-1) {
                yCoverCf[fltsOpt[f]]=1;
            }
        }

        int[][] yCoverCfPair = new int[conNum][2];
        int conflag = 0;
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<conOpt[airc].length; i++){
                yCoverCfPair[conflag] = conOpt[airc][i];
                conflag += 1;
            }
        }



        //初始化3：为y覆盖的Pair找Initial3PatternNum个Pattern
        int[] yCoverCfPairNum = new int[conNum];
        for(int con=0; con<conNum; con++){
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for (int i = 0; i < AirpTypePatternNum[ap][tp]; i++) {
                for(int j=1; j<= AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(AirpTypeLegnumPattern[ap][tp][i][j]== yCoverCfPair[con][0] && AirpTypeLegnumPattern[ap][tp][i][j+1]== yCoverCfPair[con][1]){
                        yCoverCfPairNum[con] += 1;
                    }
                }
            }
        }
//        for(int con=0; con<conNum; con++) {
//            System.out.println(Arrays.toString(yCoverCfPair[con]));
//        }

//        System.out.println(Arrays.toString(yCoverCfPairNum));

        int Initial3PatternNum = 10;
        for(int p=0; p<yCoverCfPair.length; p++){
            int ap = CopyFlightArrS[yCoverCfPair[p][0]];
            int tp = CopyFlightType[yCoverCfPair[p][0]]<GateTypeDivideline?0:1;
            int[][] AB2;
//            AB2 = forcedCoveredPairInitialPattern(ap, tp, 1, 2, -1, yCoverCfPair[p],
//                    GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
//                    FlightType, FlightDepS, FlightArrS,
//                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT,CopyFlightArrT);
//            for (int i = 0; i < AB2.length; i++) {
//                if (AB2[i][0] != 0) {
//                    AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
//                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
//                    for(int j=0; j<1; j++){
//                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
//                    }
//                    for(int j=1; j<1+AB2[i][0]; j++){
//                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
//                    }
//                    AirpTypePatternNum[ap][tp] += 1;
//                } else {
//                    break;
//                }
//            }
//            if(yCoverCfPairNum[p]< 10){
//                int[][] AB;
//                AB = forcedCoveredPairInitialPattern(ap, tp, Initial3PatternNum, -1, -1, yCoverCfPair[p],
//                        GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
//                        FlightType, FlightDepS, FlightArrS,
//                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT,CopyFlightArrT);
//                for (int i = 0; i < AB.length; i++) {
//                    if (AB[i][0] != 0) {
//                        for(int j=0; j<1; j++){
//                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
//                        }
//                        for(int j=1; j<1+AB[i][0]; j++){
//                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
//                        }
//                        AirpTypePatternNum[ap][tp] += 1;
//                    } else {
//                        break;
//                    }
//                }
//            }
        }

        for(int con=0; con<conNum; con++){
            yCoverCfPairNum[con] = 0;
            int ap = CopyFlightArrS[yCoverCfPair[con][0]];
            int tp = CopyFlightType[yCoverCfPair[con][0]]<GateTypeDivideline?0:1;
            for (int i = 0; i < AirpTypePatternNum[ap][tp]; i++) {
                for(int j=1; j<= AirpTypeLegnumPattern[ap][tp][i][0]-1; j++){
                    if(AirpTypeLegnumPattern[ap][tp][i][j]== yCoverCfPair[con][0] && AirpTypeLegnumPattern[ap][tp][i][j+1]== yCoverCfPair[con][1]){
                        yCoverCfPairNum[con] += 1;
                    }
                }
            }
        }

        //列生成迭代
//        int[][] AirpTypeReducedcostFlag = new int[ApNum][2];
//        int AirpTypeReducedcostFlagSum = 0;
//        for(int ap=0; ap<ApNum; ap++){
//            for(int tp=0; tp<2; tp++){
//                if(AirpTypeFNum[ap][tp]==0) {
//                    AirpTypeReducedcostFlag[ap][tp] = 1;
//                    AirpTypeReducedcostFlagSum += 1;
//                }
//            }
////            System.out.println(Arrays.toString(AirpTypeReducedcostFlag[ap]));
//        }
//        int loop = 0;
//        int loopLim = 10;
//        IloCplex timecplex= new IloCplex();
//        double t0 = timecplex.getCplexTime();
//        while(AirpTypeReducedcostFlagSum<2*ApNum ){
//            double t1 = timecplex.getCplexTime();
//            loop+=1;
//            System.out.println("loop" + loop);
//            System.out.println("AirpTypeReducedCost >0 Num = "+AirpTypeReducedcostFlagSum);
//            System.out.println("Solve Lp:");
//
//            double[][] LpDualValue = GRLP(yCoverCf, yCoverCfPair,
//                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
//                    AirpTypeLegnumPattern, AirpTypePatternNum);
//
//            for(int ap=0; ap<ApNum; ap++) {
//                for(int tp = 0; tp < 2; tp++) {
//                    if(AirpTypeReducedcostFlag[ap][tp] == 1){
//                        continue;
//                    }
//                    double[] ReducedcostLegnumPattern = OneBestPattern(LpDualValue, yCoverCfPair,
//                            ap, tp, -1, -1, GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
//                            FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
//                    System.out.println("At Airp :" + ap +", Type :" +tp + ", ReducedcostCost:" + ReducedcostLegnumPattern[0]);
//                    if(ReducedcostLegnumPattern[0]>=-epson){
//                        AirpTypeReducedcostFlag[ap][tp] = 1;
//                        AirpTypeReducedcostFlagSum += 1;
//                        continue;
//                    }
//                    AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
//                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
//                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = (int) ReducedcostLegnumPattern[1];
//                    for(int i=1; i<1+AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0]; i++){
//                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][i] = (int) ReducedcostLegnumPattern[1+i];
//                    }
//                    AirpTypePatternNum[ap][tp] += 1;
//
//                    int[][] AB;
//                    AB = NegativeReducedCostPattern(LpDualValue, yCoverCfPair, ap, tp, 100, -1, -1,
//                            GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
//                            FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
//
//                    for (int i = 0; i < AB.length; i++) {
//                        if (AB[i][0] != 0) {
//                            AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
//                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
//                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = AB[i][0];
//                            for(int j=1; j<1+AB[i][0]; j++){
//                                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
//                            }
//                            AirpTypePatternNum[ap][tp] += 1;
//                        } else {
//                            break;
//                        }
//                    }
//                }
//            }
//            System.out.println("After Column Generation SP");
//            System.out.println("AirpTypeReducedCost >0 Num = "+ AirpTypeReducedcostFlagSum);
//            System.out.println("Loop use time "+(timecplex.getCplexTime()-t1)+"\n");
//        }
//        System.out.println("Total use time "+(timecplex.getCplexTime()-t0));
//        System.out.println("IPObj: ");
//        GRIP(yCoverCf, yCoverCfPair,
//                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
//                AirpTypeLegnumPattern, AirpTypePatternNum);


        GRIP(yCoverCf, yCoverCfPair,
                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                AirpTypeLegnumPattern, AirpTypePatternNum);
        System.out.println(" ");

        IloCplex GRIPcplex = GRIP2(yCoverCf, yCoverCfPair,
                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                AirpTypeLegnumPattern, AirpTypePatternNum);

        GRIPcplex.solve();
        System.out.println(GRIPcplex.getStatus());
        if(GRIPcplex.getStatus().equals(IloCplex.Status.Optimal))
            System.out.println("yes optimal");

        System.out.println(GRIPcplex.getObjValue());
        GRIPcplex.end();




//        double[][] LpDualValue = GRLP(yCoverCf, yCoverCfPair,
//                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
//                AirpTypeLegnumPattern, AirpTypePatternNum);
//
////        int[] AircRouteNum = new int[AircNum];
////        int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];
//        double[][]  yXishu = new double[AircNum][];
//        for(int airc=0; airc<AircNum; airc++){
//            yXishu[airc] = new double[AircRouteNum[airc]];
//            for(int r=0; r<AircRouteNum[airc]; r++){
//                for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
//                    int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
//                    yXishu[airc][r] += (LpDualValue[0][i1]+LpDualValue[1][i1]);
//
//                    if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
//                        int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
//                        for(int p=0; p<yCoverCfPair.length; p++){
//                            int[] Pair = yCoverCfPair[p];
//                            if(i1==Pair[0] && i2==Pair[1]){
//                                yXishu[airc][r] += LpDualValue[2][p];
//                                break;
//                            }
//                        }
//                    }
//                }
//            }
//        }






//        double PandingBLhs = 0;
//        for(int airc=0; airc<AircNum; airc++){
//            PandingBLhs += (yXishu[airc][y[airc]]);
//        }
//        System.out.println("PandingBLhs = "+ PandingBLhs);
//
//        double PandingBRhs = 0;
//        for(int ap=0; ap<ApNum; ap++) {
//            for (int tp = 0; tp < 2; tp++) {
//                if (AirpTypeReducedcostFlag[ap][tp] == 1) {
//                    continue;
//                }
//                double[] ReducedcostLegnumPattern = OneBestPattern(LpDualValue, yCoverCfPair,
//                        ap, tp, -1, -1, GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
//                        FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
//                System.out.println("At Airp :" + ap + ", Type :" + tp + ", ReducedcostCost:" + ReducedcostLegnumPattern[0]);
//                if(Math.max(-ReducedcostLegnumPattern[0],0)>PandingBRhs){
//                    PandingBRhs = Math.max(-ReducedcostLegnumPattern[0],0);
//                }
//            }
//        }
//        System.out.println("PandingBRhs = "+ PandingBRhs);
//
//
//        double[][][] BendersOptCutXishu = new double[0][][];
//        BendersOptCutXishu = Arrays.copyOf(BendersOptCutXishu,BendersOptCutXishu.length+1);
//        BendersOptCutXishu[BendersOptCutXishu.length-1] = new double[AircNum][];
//        for(int airc=0; airc<AircNum; airc++){
//            BendersOptCutXishu[BendersOptCutXishu.length-1][airc] = new double[AircRouteNum[airc]];
//            for(int r=0; r<AircRouteNum[airc]; r++){
//                BendersOptCutXishu[BendersOptCutXishu.length-1][airc][r] = yXishu[airc][r];
//            }
//        }
//
//
//
//        MPLP(BendersOptCutXishu,
//        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
//        AircLegnumDlytimeSwapnumRoute, AircRouteNum,
//        Flight, ApSlotDep, ApSlotArr);
//
//        BendersOptCutXishu = Arrays.copyOf(BendersOptCutXishu,BendersOptCutXishu.length+1);
//        BendersOptCutXishu[BendersOptCutXishu.length-1] = new double[AircNum][];
//        for(int airc=0; airc<AircNum; airc++){
//            BendersOptCutXishu[BendersOptCutXishu.length-1][airc] = new double[AircRouteNum[airc]];
//            for(int r=0; r<AircRouteNum[airc]; r++){
//                BendersOptCutXishu[BendersOptCutXishu.length-1][airc][r] = yXishu[airc][r];
//            }
//        }
//        MPLP(BendersOptCutXishu,
//                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
//                AircLegnumDlytimeSwapnumRoute, AircRouteNum,
//                Flight, ApSlotDep, ApSlotArr);














    }






}

