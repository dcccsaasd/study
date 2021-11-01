package SACG_5minTiaoTime;

import ilog.concert.*;
import ilog.cp.IloCP;
import ilog.cplex.IloCplex;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

public class SAR {
    static int DataBase = 5;
    static double epson = 1e-6;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 5;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);

    static int OneCancelCost =  200;
    static int MintDelayCost = 1;
    static int OneSwapCost = 0;
    static int GateCost = 30;

    static int MaxRouteLegNum = 10;
    static int GateCap = 100;


    static double[][] SARPLP(int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
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

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum));
        cplex.addMinimize(TotalCost);

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

    static double SARPLPv(int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
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

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum));
        cplex.addMinimize(TotalCost);

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

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        double LpOptValue = cplex.getObjValue();
//        System.out.println("LpObj= " + LpOptValue);


        cplex.end();
        return LpOptValue;
    }

    static int[] SARPIP(int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                        int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                        int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloIntExpr CancelNum = cplex.prod(1, z[0]);
        for(int f=1; f<FlightNum; f++){
            CancelNum = cplex.sum(CancelNum, z[f]);
        }

        IloIntExpr DelayTime = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                DelayTime = cplex.sum(DelayTime, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][1], y[airc][i]));
            }
        }

        IloIntExpr SwapNum = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                SwapNum = cplex.sum(SwapNum, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][2], y[airc][i]));
            }
        }

        IloIntExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum));
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloIntExpr coverf = cplex.prod(1, z[f]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        System.out.println("IpStatus = " + cplex.getStatus());
        System.out.println("IpObj = " + cplex.getObjValue());

        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
        System.out.println("SwapNum= "+cplex.getValue(SwapNum));

        int[] AircChoose = new int[AircNum];
        for(int i=0; i<AircNum; i++){
            AircChoose[i] = -1;
        }

        for(int airc=0; airc<AircNum; airc++){
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    AircChoose[airc] = j;
                }
            }
        }


        cplex.end();
        return AircChoose;
    }

    static double SARPIPv(int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                          int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                          int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloIntExpr CancelNum = cplex.prod(1, z[0]);
        for(int f=1; f<FlightNum; f++){
            CancelNum = cplex.sum(CancelNum, z[f]);
        }

        IloIntExpr DelayTime = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                DelayTime = cplex.sum(DelayTime, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][1], y[airc][i]));
            }
        }

        IloIntExpr SwapNum = cplex.prod(0, y[0][0]);
        for(int airc=0; airc<AircNum; airc++){
            for(int i=0; i<AircRouteNum[airc]; i++){
                SwapNum = cplex.sum(SwapNum, cplex.prod(AircLegnumDlytimeSwapnumRoute[airc][i][2], y[airc][i]));
            }
        }

        IloIntExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum));
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloIntExpr coverf = cplex.prod(1, z[f]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        double IpOptValue = cplex.getObjValue();
//        System.out.println("IpObj = " + cplex.getObjValue());


        cplex.end();
        return IpOptValue;
    }

    static double[] OneBestRoute(int AircID,
                                 int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                 int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                 int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                 int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                 int[] TypeCopyFlightRange, int[] TypeMtsCFID,  double[][] LpDualValue)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);

        cp.addMinimize(obj);

//        cp.addLe(obj, -epson);//加

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);


        double[] ReducedcostLegnumDlytimeSwapnumRoute = new double[4+MaxLegNum];
        if(cp.solve()){
//            System.out.println("\n"+cp.getStatus());
            ReducedcostLegnumDlytimeSwapnumRoute[0] =  cp.getObjValue();
            ReducedcostLegnumDlytimeSwapnumRoute[1] =  cp.getValue(Legnum);
            ReducedcostLegnumDlytimeSwapnumRoute[2] =  cp.getValue(Dlytime);
            ReducedcostLegnumDlytimeSwapnumRoute[3] =  cp.getValue(Swapnum);
            for(int i=0;i<ReducedcostLegnumDlytimeSwapnumRoute[1];i++){
                ReducedcostLegnumDlytimeSwapnumRoute[i+4] =  cp.getValue(flightvars[i]);
            }
            cp.end();
            return ReducedcostLegnumDlytimeSwapnumRoute;
        }

        cp.end();
        return ReducedcostLegnumDlytimeSwapnumRoute;
    }


    static double[] OneBestRoute2(int AircID,  double[][] LpDualValue,
                                  double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                  int AllSAROptSolFea2GRlength, int AllSAROptSolInfea2GRlength,
                                  double[] AllGRIPOptVal,double L,
                                  int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                  int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                  int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                  int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                  int[] TypeCopyFlightRange, int[] TypeMtsCFID)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);



        //Benders Cuts 的约束
        for(int c=0; c<AllGRLpDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpDualValue[c][0], flightvars[i]), cp.element(AllGRLpDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[c].length; p++){
                    int[] Pair = AllyCoverCfPair[c][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[4][c], cut));
        }


        //LLcuts约束的cost
        for(int c=0; c<AllSAROptSolFea2GRlength; c++) {
            obj = cp.sum(obj, -(AllGRIPOptVal[c]-L)*LpDualValue[5][c]);
        }


        //No-good cuts约束的cost
        for(int c=0; c<AllSAROptSolInfea2GRlength; c++) {
            obj = cp.sum(obj, -LpDualValue[6][c]);
        }





        cp.addMinimize(obj);

//        cp.addLe(obj, -epson);//加

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);


        double[] ReducedcostLegnumDlytimeSwapnumRoute = new double[4+MaxLegNum];
        if(cp.solve()){
//            System.out.println("\n"+cp.getStatus());
            ReducedcostLegnumDlytimeSwapnumRoute[0] =  cp.getObjValue();
            ReducedcostLegnumDlytimeSwapnumRoute[1] =  cp.getValue(Legnum);
            ReducedcostLegnumDlytimeSwapnumRoute[2] =  cp.getValue(Dlytime);
            ReducedcostLegnumDlytimeSwapnumRoute[3] =  cp.getValue(Swapnum);
            for(int i=0;i<ReducedcostLegnumDlytimeSwapnumRoute[1];i++){
                ReducedcostLegnumDlytimeSwapnumRoute[i+4] =  cp.getValue(flightvars[i]);
            }
            cp.end();
            return ReducedcostLegnumDlytimeSwapnumRoute;
        }

        cp.end();
        return ReducedcostLegnumDlytimeSwapnumRoute;
    }

    static double[] OneBestRouteGRphase1(int AircID,  double[][] LpDualValue,
                                         double[][][] AllGRLpOptDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                         int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR,double[][][] AllGRLpPhase1InfeaDualValue,
                                         double[] AllGRIPOptVal,double L,
                                         int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                         int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                         int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                         int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                         int[] TypeCopyFlightRange, int[] TypeMtsCFID)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);



        //Benders 最优割 的cost
        for(int c=0; c<AllGRLpOptDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpOptDualValue[c][0], flightvars[i]), cp.element(AllGRLpOptDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[AllSAROptSolFea2GR[c]].length; p++){
                    int[] Pair = AllyCoverCfPair[AllSAROptSolFea2GR[c]][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpOptDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[4][c], cut));
        }


        //LLcuts约束的cost
        for(int c=0; c<AllSAROptSolFea2GR.length; c++) {
            obj = cp.sum(obj, -(AllGRIPOptVal[c]-L)*LpDualValue[5][c]);
        }


        //No-good cuts约束的cost
        for(int c=0; c<AllSAROptSolInfea2GR.length; c++) {
            obj = cp.sum(obj, -LpDualValue[6][c]);
        }

        //Benders 可行割 的cost
        for(int c=0; c<AllGRLpPhase1InfeaDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpPhase1InfeaDualValue[c][0], flightvars[i]), cp.element(AllGRLpPhase1InfeaDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[AllSAROptSolInfea2GR[c]].length; p++){
                    int[] Pair = AllyCoverCfPair[AllSAROptSolInfea2GR[c]][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpPhase1InfeaDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[7][c], cut));
        }





        cp.addMinimize(obj);

//        cp.addLe(obj, -epson);//加

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 1);//加速
//        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);


        double[] ReducedcostLegnumDlytimeSwapnumRoute = new double[4+MaxLegNum];
        if(cp.solve()){
//            System.out.println("\n"+cp.getStatus());
            ReducedcostLegnumDlytimeSwapnumRoute[0] =  cp.getObjValue();
            ReducedcostLegnumDlytimeSwapnumRoute[1] =  cp.getValue(Legnum);
            ReducedcostLegnumDlytimeSwapnumRoute[2] =  cp.getValue(Dlytime);
            ReducedcostLegnumDlytimeSwapnumRoute[3] =  cp.getValue(Swapnum);
            for(int i=0;i<ReducedcostLegnumDlytimeSwapnumRoute[1];i++){
                ReducedcostLegnumDlytimeSwapnumRoute[i+4] =  cp.getValue(flightvars[i]);
            }
            cp.end();
            return ReducedcostLegnumDlytimeSwapnumRoute;
        }

        cp.end();
        return ReducedcostLegnumDlytimeSwapnumRoute;
    }

    static int[][] NegativeReducedCostRoute(int AircID,
                                            int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                            int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                            int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                            int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                            int[] TypeCopyFlightRange, int[] TypeMtsCFID,
                                            double[][] LpDualValue, double Reducedcost,int NumberOfRoute)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);

//        cp.addLe(obj, Reducedcost+epson);
        cp.addLe(obj, -epson);

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
//        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 8);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);

        cp.startNewSearch();

        int[][] LegnumDlytimeSwapnumRoute = new int[NumberOfRoute][3+MaxLegNum];
        int findflag = 0;
        for(int n=0; n<NumberOfRoute; n++){
            if(cp.next()){
                findflag += 1;
                LegnumDlytimeSwapnumRoute[n][0] = (int) cp.getValue(Legnum);
                LegnumDlytimeSwapnumRoute[n][1] = (int) cp.getValue(Dlytime);
                LegnumDlytimeSwapnumRoute[n][2] = (int) cp.getValue(Swapnum);
                for(int i=0;i<MaxLegNum;i++){
                    LegnumDlytimeSwapnumRoute[n][i+3] = (int) cp.getValue(flightvars[i]);
                }
            }else{
//                System.out.println("AircID: "+AircID+" find " +findflag+ " route(s)");

                cp.end();
                return LegnumDlytimeSwapnumRoute;
            }
        }

        cp.end();
        return LegnumDlytimeSwapnumRoute;
    }

    static int[][] NegativeReducedCostRoute2(int AircID,  double[][] LpDualValue,
                                             double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                             int AllSAROptSolFea2GRlength, int AllSAROptSolInfea2GRlength,
                                             double[] AllGRIPOptVal,double L,
                                             int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                             int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                             int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                             int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                             int[] TypeCopyFlightRange, int[] TypeMtsCFID, int NumberOfRoute)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);



        //Benders Cuts 的约束
        for(int c=0; c<AllGRLpDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpDualValue[c][0], flightvars[i]), cp.element(AllGRLpDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[c].length; p++){
                    int[] Pair = AllyCoverCfPair[c][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[4][c], cut));
        }


        //LLcuts约束的cost
        for(int c=0; c<AllSAROptSolFea2GRlength; c++) {
            obj = cp.sum(obj, (AllGRIPOptVal[c]-L)*LpDualValue[5][c]);
        }


        //No-good cuts约束的cost
        for(int c=0; c<AllSAROptSolInfea2GRlength; c++) {
            obj = cp.sum(obj, -LpDualValue[6][c]);
        }

//        cp.addLe(obj, Reducedcost+epson);
        cp.addLe(obj, -epson);

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
//        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 12);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,2);

        cp.startNewSearch();

        int[][] LegnumDlytimeSwapnumRoute = new int[NumberOfRoute][3+MaxLegNum];
        int findflag = 0;
        for(int n=0; n<NumberOfRoute; n++){
            if(cp.next()){
                findflag += 1;
                LegnumDlytimeSwapnumRoute[n][0] = (int) cp.getValue(Legnum);
                LegnumDlytimeSwapnumRoute[n][1] = (int) cp.getValue(Dlytime);
                LegnumDlytimeSwapnumRoute[n][2] = (int) cp.getValue(Swapnum);
                for(int i=0;i<MaxLegNum;i++){
                    LegnumDlytimeSwapnumRoute[n][i+3] = (int) cp.getValue(flightvars[i]);
                }
            }else{
//                System.out.println("AircID: "+AircID+" find " +findflag+ " route(s)");

                cp.end();
                return LegnumDlytimeSwapnumRoute;
            }
        }

        cp.end();
        return LegnumDlytimeSwapnumRoute;
    }

    static int[][] NegativeReducedCostRouteGRphase1(int AircID,  double[][] LpDualValue,
                                                    double[][][] AllGRLpOptDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                                    int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR, double[][][] AllGRLpPhase1InfeaDualValue,
                                                    double[] AllGRIPOptVal,double L,
                                                    int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS,
                                                    int[] CopyFlightDepT, int[] CopyFlightArrT, int[] CopyFlightDepTSlot, int[] CopyFlightArrTSlot,
                                                    int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                                    int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                                    int[] TypeCopyFlightRange, int[] TypeMtsCFID, int NumberOfRoute)throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;
        int SlotNum = 24;

        IloCP cp = new IloCP();

        int[] lb = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            lb[i] = TypeCopyFlightRange[0];
        }
        int[] ub = new int[MaxLegNum];
        for(int i=0; i<MaxLegNum; i++){
            ub[i] = TypeCopyFlightRange[1];
        }

        IloIntVar[] flightvars = cp.intVarArray(MaxLegNum, lb, ub);
        IloIntVar[] isactual = cp.intVarArray(MaxLegNum,0, 1);

        IloIntVar Legnum = cp.intVar(0, MaxLegNum);
        IloIntVar Dlytime = cp.intVar(0, IloCP.IntMax);
        IloIntVar Swapnum = cp.intVar(0, MaxLegNum);

        //Cost=延误费用+交换费用
        IloNumExpr obj = cp.sum(cp.prod(MintDelayCost,Dlytime), cp.prod(OneSwapCost,Swapnum));

        //CopyFlightDual的cost
        double[] CopyFlightDual = new double[CopyFlightOFID.length];
        for(int i=0; i<CopyFlightOFID.length; i++){
            if(CopyFlightDepS[i]!=CopyFlightArrS[i]){
                CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
            }else{
                CopyFlightDual[i] = 0;
            }
//            CopyFlightDual[i] = LpDualValue[0][CopyFlightOFID[i]];
        }
        for(int i=0; i<MaxLegNum; i++){
            obj = cp.diff(obj,
                    cp.prod( cp.element(CopyFlightDual, flightvars[i]), cp.ge(isactual[i], 1))
            );
        }

        //DepSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightDepTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[1], apslot), flag )
            );
        }

        //ArrSlot的cost
        for(int i=0; i<MaxLegNum; i++){
            IloIntExpr ap = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr slot = cp.element(CopyFlightArrTSlot, flightvars[i]);
            IloIntExpr apslot = cp.sum(cp.prod(SlotNum, ap), slot);
            IloIntExpr iDepS = cp.element(CopyFlightDepS, flightvars[i]);
            IloIntExpr iArrS = cp.element(CopyFlightArrS, flightvars[i]);
            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), cp.neq(iDepS, iArrS));
//            IloIntExpr flag = cp.prod(cp.ge(isactual[i], 1), 1);
            obj = cp.sum(obj,
                    cp.prod( cp.element(LpDualValue[2], apslot), flag )
            );
        }

        //飞机的cost
        obj = cp.sum(obj, LpDualValue[3][AircID]);



        //Benders 最优割 的cost
        for(int c=0; c<AllGRLpOptDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpOptDualValue[c][0], flightvars[i]), cp.element(AllGRLpOptDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[AllSAROptSolFea2GR[c]].length; p++){
                    int[] Pair = AllyCoverCfPair[AllSAROptSolFea2GR[c]][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpOptDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[4][c], cut));
        }


        //LLcuts约束的cost
        for(int c=0; c<AllSAROptSolFea2GR.length; c++) {
            obj = cp.sum(obj, (AllGRIPOptVal[c]-L)*LpDualValue[5][c]);
        }


        //No-good cuts约束的cost
        for(int c=0; c<AllSAROptSolInfea2GR.length; c++) {
            obj = cp.sum(obj, -LpDualValue[6][c]);
        }

        //Benders 可行割 的cost
        for(int c=0; c<AllGRLpPhase1InfeaDualValue.length; c++){
            IloNumExpr cut = cp.prod(0, isactual[0]);
            for(int i=0; i<MaxLegNum; i++){
                cut = cp.sum(cut,
                        cp.prod( cp.sum(cp.element(AllGRLpPhase1InfeaDualValue[c][0], flightvars[i]), cp.element(AllGRLpPhase1InfeaDualValue[c][1], flightvars[i])),
                                cp.ge(isactual[i], 1))
                );
            }
            for(int i=0; i<MaxLegNum-1; i++){
                for(int p=0; p<AllyCoverCfPair[AllSAROptSolInfea2GR[c]].length; p++){
                    int[] Pair = AllyCoverCfPair[AllSAROptSolInfea2GR[c]][p];
                    IloNumExpr i1ArrS = cp.element(CopyFlightArrS, flightvars[i]);
                    IloNumExpr i2DepS = cp.element(CopyFlightDepS, flightvars[i+1]);

                    IloNumExpr flag =
                            cp.prod(cp.ge(cp.sum(cp.eq(i1ArrS,Pair[0]), cp.eq(i2DepS,Pair[1])), 2),
                                    cp.ge(cp.sum(isactual[i], isactual[i+1]), 2));

                    cut = cp.sum(cut, cp.prod(AllGRLpPhase1InfeaDualValue[c][2][p], flag));
                }

            }
            obj = cp.sum(obj, cp.prod(LpDualValue[7][c], cut));
        }




//        cp.addLe(obj, Reducedcost+epson);
        cp.addLe(obj, -epson);

        //起点、终点固定机场
        cp.addEq(cp.element(CopyFlightDepS, flightvars[0]), AircStartS);
//        cp.addEq(cp.element(CopyFlightArrS, flightvars[MaxLegNum-1]), AircEndS);

        //维修约束
        if(AircMts == -1){
            if(TypeMtsCFID.length > 0){
                for(int i=0; i< TypeMtsCFID.length; i++ ) {
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }
        if(AircMts != -1){
            for(int i=0; i< TypeMtsCFID.length; i++ ) {
                if(CopyFlightOFID[TypeMtsCFID[i]] == AircMts){
                    cp.addGe(cp.count(flightvars, TypeMtsCFID[i]),1);
                }else{
                    cp.addEq(cp.count(flightvars, TypeMtsCFID[i]),0);
                }
            }
        }

        //变量关系约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.ge( isactual[i-1] ,isactual[i] ) );
        }
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.eq( isactual[i] ,cp.neq(flightvars[i], flightvars[i-1]) ) );
        }

        //时间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.ge( cp.element(CopyFlightDepT, flightvars[i]) ,
                            cp.sum(cp.element(CopyFlightArrT, flightvars[i-1]), AircTurnT ) ) ));
        }

        //空间衔接约束
        for(int i = 1; i<MaxLegNum; i++){
            cp.add( cp.or( cp.eq(isactual[i], 0),
                    cp.eq( cp.element(CopyFlightDepS, flightvars[i]) , cp.element(CopyFlightArrS, flightvars[i-1]) ) ));
        }

        //飞行时间约束
        IloIntExpr FlyTimeActual = cp.prod(cp.element(CopyFlightFlyT, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            FlyTimeActual = cp.sum(FlyTimeActual, cp.prod(cp.element(CopyFlightFlyT, flightvars[i]), isactual[i]));
        }
        cp.add( cp.le( FlyTimeActual, MaxFlyTime));

        //执行航班数约束
        IloIntExpr LegnumActual = cp.prod(1, isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            LegnumActual = cp.sum(LegnumActual, isactual[i]);
        }
        cp.add( cp.eq( Legnum, LegnumActual));
        cp.add(cp.ge(Legnum, 2));

        //延误时间约束
        IloIntExpr DlytimeActual = cp.prod(cp.element(CopyFlightDlyMint, flightvars[0]), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            DlytimeActual = cp.sum(DlytimeActual, cp.prod(cp.element(CopyFlightDlyMint, flightvars[i]), isactual[i]));
        }
        cp.add( cp.eq( Dlytime, DlytimeActual));

        //交换次数约束
        IloIntExpr SwapnumActual = cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[0]), AircID), isactual[0]);
        for(int i=1; i<MaxLegNum; i++){
            SwapnumActual = cp.sum(SwapnumActual, cp.prod( cp.neq(cp.element(CopyFlightOAID, flightvars[i]), AircID), isactual[i]));
        }
        cp.add( cp.eq( Swapnum, SwapnumActual));

        cp.setParameter(IloCP.IntParam.LogVerbosity, IloCP.ParameterValues.Quiet);
        cp.setParameter(IloCP.IntParam.Workers, 12);//加速
        cp.setParameter(IloCP.IntParam.SearchType, IloCP.ParameterValues.MultiPoint);
        cp.setParameter(IloCP.DoubleParam.TimeLimit,5);

        cp.startNewSearch();

        int[][] LegnumDlytimeSwapnumRoute = new int[NumberOfRoute][3+MaxLegNum];
        int findflag = 0;
        for(int n=0; n<NumberOfRoute; n++){
            if(cp.next()){
                findflag += 1;
                LegnumDlytimeSwapnumRoute[n][0] = (int) cp.getValue(Legnum);
                LegnumDlytimeSwapnumRoute[n][1] = (int) cp.getValue(Dlytime);
                LegnumDlytimeSwapnumRoute[n][2] = (int) cp.getValue(Swapnum);
                for(int i=0;i<MaxLegNum;i++){
                    LegnumDlytimeSwapnumRoute[n][i+3] = (int) cp.getValue(flightvars[i]);
                }
            }else{
//                System.out.println("AircID: "+AircID+" find " +findflag+ " route(s)");

                cp.end();
                return LegnumDlytimeSwapnumRoute;
            }
        }

        cp.end();
        return LegnumDlytimeSwapnumRoute;
    }


    static double[][] SARLP( double[][][] BendersOptCutXishu,
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

        IloNumExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloNumExpr[] AircDelayTime = new IloNumExpr[AircNum];
        IloNumExpr[] AircSwapNum = new IloNumExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloNumExpr DelayTime = cplex.sum(AircDelayTime);
        IloNumExpr SwapNum  = cplex.sum(AircSwapNum);


        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
            IloNumExpr coverairc = cplex.sum(y[airc]);
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        //double[][][] BendersOptCutXishu
        for(int c=0; c< BendersOptCutXishu.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                expr.addTerms(y[airc], BendersOptCutXishu[c][airc]);
            }
            cplex.addLe(cplex.diff(expr,q),0);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
//        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
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

    static double[][] SARLP2( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
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


        IloRange[][] rng = new IloRange[5][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpDualValue.length];

        IloNumExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloNumExpr[] AircDelayTime = new IloNumExpr[AircNum];
        IloNumExpr[] AircSwapNum = new IloNumExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloNumExpr DelayTime = cplex.sum(AircDelayTime);
        IloNumExpr SwapNum  = cplex.sum(AircSwapNum);


        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
            IloNumExpr coverairc = cplex.sum(y[airc]);
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }


//        for(int airc=0; airc<AircNum; airc++){
//            expr.addTerms(y[airc], BendersOptCutXishu[c][airc]);
//        }


        for(int c=0; c< AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), 0);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
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



        double[][] LpDualValue = new double[5][];
        for(int i=0; i<5; i++){
            LpDualValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }


        cplex.end();
        return LpDualValue;
    }

    static double[][] SARLP3( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                              int[][] AllSAROptSol, double[] AllGRIPOptVal,double L,
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


        IloRange[][] rng = new IloRange[6][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpDualValue.length];
        rng[5] = new IloRange[AllSAROptSol.length];

        IloNumExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloNumExpr[] AircDelayTime = new IloNumExpr[AircNum];
        IloNumExpr[] AircSwapNum = new IloNumExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloNumExpr DelayTime = cplex.sum(AircDelayTime);
        IloNumExpr SwapNum  = cplex.sum(AircSwapNum);


        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
            IloNumExpr coverairc = cplex.sum(y[airc]);
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }


//        for(int airc=0; airc<AircNum; airc++){
//            expr.addTerms(y[airc], BendersOptCutXishu[c][airc]);
//        }


        for(int c=0; c< AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), 0);
        }


        for(int c=0; c<AllSAROptSol.length; c++){
            double GRIPOptVal = AllGRIPOptVal[c];
            int[]  SAROptSol = AllSAROptSol[c];
            double Xishu = L- GRIPOptVal;
            double Changshu = -1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
//            cplex.addGe(cplex.diff(q, expr), Changshu);
            rng[5][c] =  cplex.addGe(cplex.prod(cplex.diff(q, expr), 1/Xishu), Changshu/Xishu);
        }





//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
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



        double[][] LpDualValue = new double[6][];
        for(int i=0; i<6; i++){
            LpDualValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }


        cplex.end();
        return LpDualValue;
    }

    static double[][] SARLP4( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                              int[][] AllSAROptSol, int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR,
                              double[] AllGRIPOptVal,double L,
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


        IloRange[][] rng = new IloRange[7][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpDualValue.length];
        rng[5] = new IloRange[AllSAROptSolFea2GR.length];
        rng[6] = new IloRange[AllSAROptSolInfea2GR.length];

        IloNumExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloNumExpr[] AircDelayTime = new IloNumExpr[AircNum];
        IloNumExpr[] AircSwapNum = new IloNumExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloNumExpr DelayTime = cplex.sum(AircDelayTime);
        IloNumExpr SwapNum  = cplex.sum(AircSwapNum);


        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
            IloNumExpr coverairc = cplex.sum(y[airc]);
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }


        //Benders最优割
        for(int c=0; c< AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), -OptCutChangshu);
        }

        // Lcut
        for(int i=0; i<AllSAROptSolFea2GR.length; i++){
            int c = AllSAROptSolFea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];
            double GRIPOptVal = AllGRIPOptVal[i];
            System.out.println(GRIPOptVal);


            double Xishu = L- GRIPOptVal;
            double Changshu = 0;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
            rng[5][i] =  cplex.addGe(cplex.diff(q, expr), Changshu);
//            rng[5][i] =  cplex.addGe(cplex.prod(cplex.diff(q, expr), 1/Xishu), Changshu/Xishu);
        }


        //No-good cut
        for(int i=0; i<AllSAROptSolInfea2GR.length; i++){
            int c = AllSAROptSolInfea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];

            double Changshu = 1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += (-1);
                }
            }

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(1, y[airc][r]);
                    }else{
                        expr.addTerm(-1, y[airc][r]);
                    }
                }
            }
            rng[6][i] =  cplex.addGe(expr, Changshu);
        }





//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
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



        double[][] LpDualValueLpValue = new double[8][];
        for(int i=0; i<7; i++){
            LpDualValueLpValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }
        LpDualValueLpValue[7] = new double[1];
        LpDualValueLpValue[7][0] = cplex.getObjValue();

        cplex.end();
        return LpDualValueLpValue;
    }

    static double[][] SARLPGRphase1( double[][][] AllGRLpOptDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                     int[][] AllSAROptSol, int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR, double[][][] AllGRLpPhase1InfeaDualValue,
                                     double[] AllGRIPOptVal,double L,
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


        IloRange[][] rng = new IloRange[8][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpOptDualValue.length];
        rng[5] = new IloRange[AllSAROptSolFea2GR.length];
        rng[6] = new IloRange[AllSAROptSolInfea2GR.length];
        rng[7] = new IloRange[AllGRLpPhase1InfeaDualValue.length];

        IloNumExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloNumExpr[] AircDelayTime = new IloNumExpr[AircNum];
        IloNumExpr[] AircSwapNum = new IloNumExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloNumExpr DelayTime = cplex.sum(AircDelayTime);
        IloNumExpr SwapNum  = cplex.sum(AircSwapNum);


        IloNumExpr TotalCostq = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCostq);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
            IloNumExpr coverairc = cplex.sum(y[airc]);
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }


        //Benders最优割
        for(int c=0; c< AllGRLpOptDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpOptDualValue[c][0][i1]+AllGRLpOptDualValue[c][1][i1]);
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpOptDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpOptDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), -OptCutChangshu);
        }

        // LLcut
        for(int i=0; i<AllSAROptSolFea2GR.length; i++){
            int c = AllSAROptSolFea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];
            double GRIPOptVal = AllGRIPOptVal[i];
            System.out.println(GRIPOptVal);


            double Xishu = L- GRIPOptVal;
            double Changshu = 0;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
            rng[5][i] =  cplex.addGe(cplex.diff(q, expr), Changshu);
//            rng[5][i] =  cplex.addGe(cplex.prod(cplex.diff(q, expr), 1/Xishu), Changshu/Xishu);
        }


        //No-good cut
        for(int i=0; i<AllSAROptSolInfea2GR.length; i++){
            int c = AllSAROptSolInfea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];

            double Changshu = 1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += (-1);
                }
            }

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(1, y[airc][r]);
                    }else{
                        expr.addTerm(-1, y[airc][r]);
                    }
                }
            }
            rng[6][i] =  cplex.addGe(expr, Changshu);
        }

        //Benders可行割
        for(int c=0; c< AllGRLpPhase1InfeaDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpPhase1InfeaDualValue[c][0][i1]+AllGRLpPhase1InfeaDualValue[c][1][i1]);
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpPhase1InfeaDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpPhase1InfeaDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
            rng[7][c] =  cplex.addGe(cplex.diff(0, expr), -OptCutChangshu);
        }





//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        cplex.exportModel("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\SARP.lp");
        System.out.println("LpStatus = " + cplex.getStatus());
        System.out.println("LpObj = " + cplex.getObjValue());

        double[][] LpDualValueLpValue = new double[9][];
        for(int i=0; i<8; i++){
            LpDualValueLpValue[i] = cplex.getDuals(rng[i]);
//            System.out.println(Arrays.toString(LpDualValue[i]));
        }
        LpDualValueLpValue[8] = new double[1];
        LpDualValueLpValue[8][0] = cplex.getObjValue();

        cplex.end();
        return LpDualValueLpValue;
    }



    static double[] SARIP( double[][][] BendersOptCutXishu,
                           int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                           int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                           int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        for(int c=0; c< BendersOptCutXishu.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                expr.addTerms(y[airc], BendersOptCutXishu[c][airc]);
            }
            IloRange cons = cplex.addLe(cplex.diff(expr,q),0);
//            cplex.addLazyConstraint(cons);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

//        System.out.println("IpStatus = " + cplex.getStatus());
        System.out.println("SARIpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));

        int[] AircChoose = new int[AircNum];
        for(int i=0; i<AircNum; i++){
            AircChoose[i] = -1;
        }

        for(int airc=0; airc<AircNum; airc++){
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    AircChoose[airc] = j;
                }
            }
        }

        double[] yq = new double[AircNum+1];
        for(int airc=0; airc<AircNum; airc++){
            yq[airc] = AircChoose[airc];
        }
        yq[AircNum] = cplex.getValue(q);


        cplex.end();
        return yq;
    }

    static double[] SARIP2( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                            int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                            int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                            int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");
//        IloIntVar q = cplex.intVar(0, Integer.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        for(int c=0; c<AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }

                    expr.addTerm(Xishu, y[airc][r]);
                }
            }
            IloRange cons = cplex.addLe(cplex.diff(expr,q), 0);
//            cplex.addLazyConstraint(cons);
        }

//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        System.out.println("IpStatus = " + cplex.getStatus());
        System.out.println("SARIpObj = " + cplex.getObjValue());

//        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));

        int[] AircChoose = new int[AircNum];
        for(int i=0; i<AircNum; i++){
            AircChoose[i] = -1;
        }

        for(int airc=0; airc<AircNum; airc++){
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    AircChoose[airc] = j;
                }
            }
        }

        double[] yq = new double[AircNum+1];
        for(int airc=0; airc<AircNum; airc++){
            yq[airc] = AircChoose[airc];
        }
        yq[AircNum] = cplex.getValue(q);


        cplex.end();
        return yq;
    }

    static double[] SARIP3( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                            int[][] AllSAROptSol, double[] AllGRIPOptVal, double L,
                            int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                            int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                            int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");
//        IloIntVar q = cplex.intVar(0, Integer.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[4][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        for(int c=0; c<AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }

                    expr.addTerm(Xishu, y[airc][r]);
                }
            }
            IloRange cons = cplex.addLe(cplex.diff(expr,q), 0);
//            cplex.addLazyConstraint(cons);
        }


        for(int c=0; c<AllSAROptSol.length; c++){
            double GRIPOptVal = AllGRIPOptVal[c];
            int[]  SAROptSol = AllSAROptSol[c];
            double Xishu = L- GRIPOptVal;
            double Changshu = -1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
            cplex.addGe(cplex.diff(q, expr), Changshu);
        }


//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        System.out.println("SARIpStatus = " + cplex.getStatus());
        System.out.println("SARIpObj = " + cplex.getObjValue());
        System.out.println("q = " + cplex.getValue(q));

//        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));

        int[] AircChoose = new int[AircNum];
        for(int i=0; i<AircNum; i++){
            AircChoose[i] = -1;
        }

        for(int airc=0; airc<AircNum; airc++){
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    AircChoose[airc] = j;
                }
            }
        }

        double[] yq = new double[AircNum+1];
        for(int airc=0; airc<AircNum; airc++){
            yq[airc] = AircChoose[airc];
        }
        yq[AircNum] = cplex.getValue(q);


        cplex.end();
        return yq;
    }

    static double[] SARIP4( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                            int[][] AllSAROptSol, int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR,
                            double[] AllGRIPOptVal,double L,
                            int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                            int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                            int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");
//        IloIntVar q = cplex.intVar(0, Integer.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[7][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpDualValue.length];
        rng[5] = new IloRange[AllSAROptSolFea2GR.length];
        rng[6] = new IloRange[AllSAROptSolInfea2GR.length];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        for(int c=0; c<AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }

                    expr.addTerm(Xishu, y[airc][r]);
                }
            }
            IloRange cons = cplex.addLe(cplex.diff(expr,q), 0);
//            cplex.addLazyConstraint(cons);
        }


        for(int i=0; i<AllSAROptSolFea2GR.length; i++){
            int c = AllSAROptSolFea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];
            double GRIPOptVal = AllGRIPOptVal[i];


            double Xishu = L- GRIPOptVal;
            double Changshu = -1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
//            cplex.addGe(cplex.diff(q, expr), Changshu);
            cplex.addGe(cplex.diff(q, expr), Changshu);
        }



        for(int i=0; i<AllSAROptSolInfea2GR.length; i++){
            int c = AllSAROptSolInfea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];

            double Changshu = 1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += (-1);
                }
            }

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(1, y[airc][r]);
                    }else{
                        expr.addTerm(-1, y[airc][r]);
                    }
                }
            }
            rng[6][i] =  cplex.addGe(expr, Changshu);
        }


//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        System.out.println("SARIpStatus = " + cplex.getStatus());
        System.out.println("SARIpObj = " + cplex.getObjValue());
        System.out.println("q = " + cplex.getValue(q));

//        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));

        int[] AircChoose = new int[AircNum];
        for(int i=0; i<AircNum; i++){
            AircChoose[i] = -1;
        }

        for(int airc=0; airc<AircNum; airc++){
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    AircChoose[airc] = j;
                }
            }
        }

        double[] yq = new double[AircNum+1];
        for(int airc=0; airc<AircNum; airc++){
            yq[airc] = AircChoose[airc];
        }
        yq[AircNum] = cplex.getValue(q);


        cplex.end();
        return yq;
    }

    static double[] SARIP5( double[][][] AllGRLpOptDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                            int[][] AllSAROptSol, int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR,
                            double[] AllGRIPOptVal,double L,
                            int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                            int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                            int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[7][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpOptDualValue.length];
        rng[5] = new IloRange[AllSAROptSolFea2GR.length];
        rng[6] = new IloRange[AllSAROptSolInfea2GR.length];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        //Benders最优割
        for(int c=0; c< AllGRLpOptDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpOptDualValue[c][0][i1]+AllGRLpOptDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpOptDualValue[c][0][i1]+AllGRLpOptDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpOptDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpOptDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), -OptCutChangshu);
        }

        // LLcut
        for(int i=0; i<AllSAROptSolFea2GR.length; i++){
            int c = AllSAROptSolFea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];
            double GRIPOptVal = AllGRIPOptVal[i];
            System.out.println(GRIPOptVal);


            double Xishu = L- GRIPOptVal;
            double Changshu = 0;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
            rng[5][i] =  cplex.addGe(cplex.diff(q, expr), Changshu);
//            rng[5][i] =  cplex.addGe(cplex.prod(cplex.diff(q, expr), 1/Xishu), Changshu/Xishu);
        }


        //No-good cut
        for(int i=0; i<AllSAROptSolInfea2GR.length; i++){
            int c = AllSAROptSolInfea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];

            double Changshu = 1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += (-1);
                }
            }

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(1, y[airc][r]);
                    }else{
                        expr.addTerm(-1, y[airc][r]);
                    }
                }
            }
            rng[6][i] =  cplex.addGe(expr, Changshu);
        }


//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        double[] yqobj = new double[AircNum+2];
        System.out.println("SARIpStatus = " + cplex.getStatus());

        for(int airc=0; airc<AircNum; airc++){
            yqobj[airc] = -1;
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    yqobj[airc] = j;
                    break;
                }
            }
        }

        yqobj[AircNum] = cplex.getValue(q);
        System.out.println("q = " + yqobj[AircNum]);
        yqobj[AircNum+1] = cplex.getObjValue();
        System.out.println("SARIpObj = " + yqobj[AircNum+1]);
        System.out.println("CancelNum = "+ Math.round(cplex.getValue(CancelNum)));
        System.out.println("DelayTime = "+ Math.round(cplex.getValue(DelayTime)));
        System.out.println("SwapNum = "+ Math.round(cplex.getValue(SwapNum)));

//        for(int f=0; f<FlightNum; f++){
//            if(cplex.getValue(z[f])==1){
//                System.out.println("Flight"+f+"is cancelled");
//            }
//        }

        cplex.end();
        return yqobj;
    }


    static double[] SARIPGRphase1( double[][][] AllGRLpDualValue, int[][] AllyCoverCf, int[][][] AllyCoverCfPair,
                                   int[][] AllSAROptSol, int[] AllSAROptSolFea2GR, int[] AllSAROptSolInfea2GR, double[][][] AllGRLpPhase1InfeaDualValue,
                                   double[] AllGRIPOptVal,double L,
                                   int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                                   int[][][] AircLegnumDlytimeSwapnumRoute, int[] AircRouteNum,
                                   int[][] Flight, int[][] ApSlotDep, int[][] ApSlotArr)throws IloException {
        int FlightNum = Flight.length;
        int AircNum = AircRouteNum.length;
        int ApNum = ApSlotDep.length;
        int SlotNum = ApSlotDep[0].length;

        IloCplex cplex = new IloCplex();

        IloIntVar[][] y = new IloIntVar[AircNum][];
        for(int airc=0; airc<AircNum; airc++){
            String[] yname = new String[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                yname[i]="y"+airc +","+ i;
            }
            y[airc] = cplex.intVarArray(AircRouteNum[airc], 0, 1, yname);
        }

        String[] zname = new String[FlightNum];
        for(int f=0; f<FlightNum; f++){
            zname[f]="z"+ f;
        }
        IloIntVar[] z = cplex.intVarArray(FlightNum, 0, 1, zname);
        IloNumVar q = cplex.numVar(0, Double.MAX_VALUE, "q");
//        IloIntVar q = cplex.intVar(0, Integer.MAX_VALUE, "q");

        IloRange[][] rng = new IloRange[8][];
        rng[0] = new IloRange[FlightNum];
        rng[1] = new IloRange[ApNum*SlotNum];
        rng[2] = new IloRange[ApNum*SlotNum];
        rng[3] = new IloRange[AircNum];
        rng[4] = new IloRange[AllGRLpDualValue.length];
        rng[5] = new IloRange[AllSAROptSolFea2GR.length];
        rng[6] = new IloRange[AllSAROptSolInfea2GR.length];
        rng[7] = new IloRange[AllGRLpPhase1InfeaDualValue.length];

        IloIntExpr CancelNum = cplex.sum(z);

        int[][] RouteDelayTime = new int[AircNum][];
        int[][] RouteSwapNum = new int[AircNum][];
        IloIntExpr[] AircDelayTime = new IloIntExpr[AircNum];
        IloIntExpr[] AircSwapNum = new IloIntExpr[AircNum];
        for(int airc=0; airc<AircNum; airc++){
            RouteDelayTime[airc] = new int[AircRouteNum[airc]];
            RouteSwapNum[airc] = new int[AircRouteNum[airc]];
            for(int i=0; i<AircRouteNum[airc]; i++){
                RouteDelayTime[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][1];
                RouteSwapNum[airc][i] = AircLegnumDlytimeSwapnumRoute[airc][i][2];
            }
            AircDelayTime[airc] = cplex.scalProd(RouteDelayTime[airc], y[airc]);
            AircSwapNum[airc] = cplex.scalProd(RouteSwapNum[airc], y[airc]);
        }
        IloIntExpr DelayTime = cplex.sum(AircDelayTime);
        IloIntExpr SwapNum  = cplex.sum(AircSwapNum);

        IloNumExpr TotalCost = cplex.sum(cplex.prod(OneCancelCost, CancelNum),
                cplex.prod(MintDelayCost, DelayTime), cplex.prod(OneSwapCost, SwapNum), q);
        cplex.addMinimize(TotalCost);

        for(int f=0; f<FlightNum; f++){
            IloNumExpr coverf;
            if(Flight[f][1]!=Flight[f][2]){
                coverf = cplex.prod(1, z[f]);
            }else{
                coverf = cplex.prod(0, z[f]);
            }
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
                IloIntExpr apslot = cplex.prod(0, z[0]);
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
            IloIntExpr coverairc = cplex.prod(1, y[airc][0]);
            for(int i =1; i<AircRouteNum[airc]; i++){
                coverairc = cplex.sum(coverairc, y[airc][i]);
            }
//            rng[3][airc] = cplex.addLe(coverairc, 1);
            rng[3][airc] = cplex.addGe(cplex.prod(-1,coverairc), -1);
        }

        //Benders最优割
        for(int c=0; c< AllGRLpDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                        for(int cf=0; cf<AllyCoverCf[c].length; cf++){
//                            if(i1 == AllyCoverCf[c][cf]){
//                                Xishu += (AllGRLpDualValue[c][0][i1]+AllGRLpDualValue[c][1][i1]);
//                                break;
//                            }
//                        }
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[4][c] =  cplex.addGe(cplex.diff(q, expr), -OptCutChangshu);
        }

        // Lcut
        for(int i=0; i<AllSAROptSolFea2GR.length; i++){
            int c = AllSAROptSolFea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];
            double GRIPOptVal = AllGRIPOptVal[i];
            System.out.println(GRIPOptVal);


            double Xishu = L- GRIPOptVal;
            double Changshu = 0;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += 1;
                }
            }
            Changshu = Changshu*Xishu + GRIPOptVal;

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(Xishu, y[airc][r]);
                    }else{
                        expr.addTerm(-Xishu, y[airc][r]);
                    }
                }
            }
//            cplex.addLe(cplex.diff(expr, q), -Changshu);
            rng[5][i] =  cplex.addGe(cplex.diff(q, expr), Changshu);
//            rng[5][i] =  cplex.addGe(cplex.prod(cplex.diff(q, expr), 1/Xishu), Changshu/Xishu);
        }


        //No-good cut
        for(int i=0; i<AllSAROptSolInfea2GR.length; i++){
            int c = AllSAROptSolInfea2GR[i];
            int[]  SAROptSol = AllSAROptSol[c];

            double Changshu = 1;
            for(int airc=0; airc<AircNum; airc++){
                if(SAROptSol[airc]!=-1){
                    Changshu += (-1);
                }
            }

            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++) {
                for (int r = 0; r < AircRouteNum[airc]; r++) {
                    if(r!=SAROptSol[airc]){
                        expr.addTerm(1, y[airc][r]);
                    }else{
                        expr.addTerm(-1, y[airc][r]);
                    }
                }
            }
            rng[6][i] =  cplex.addGe(expr, Changshu);
        }

        //Benders可行割
        for(int c=0; c< AllGRLpPhase1InfeaDualValue.length; c++){
            IloLinearNumExpr expr = cplex.linearNumExpr();
            for(int airc=0; airc<AircNum; airc++){
                for(int r =0; r<AircRouteNum[airc];r++){
                    double Xishu = 0;
                    for(int i=3; i<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]; i++){
                        int i1 = AircLegnumDlytimeSwapnumRoute[airc][r][i];
                        Xishu += (AllGRLpPhase1InfeaDualValue[c][0][i1]+AllGRLpPhase1InfeaDualValue[c][1][i1]);
                        if( i1<3+AircLegnumDlytimeSwapnumRoute[airc][r][0]-1){
                            int i2 = AircLegnumDlytimeSwapnumRoute[airc][r][i+1];
                            for(int p=0; p<AllyCoverCfPair[c].length; p++){
                                int[] Pair = AllyCoverCfPair[c][p];
                                if(i1==Pair[0] && i2==Pair[1]){
                                    Xishu += AllGRLpPhase1InfeaDualValue[c][2][p];
                                    break;
                                }
                            }
                        }
                    }
                    expr.addTerm(Xishu, y[airc][r]);
                }
            }

            double OptCutChangshu = 0;
            for(int ap=0; ap<ApNum; ap++) {
                for(int tp = 0; tp < 2; tp++) {
                    OptCutChangshu += AllGRLpPhase1InfeaDualValue[c][3][ap*2+tp];
                }
            }
            OptCutChangshu = OptCutChangshu * GateCap;
//            rng[4][c] =  cplex.addLe(cplex.diff(expr, q), 0);
            rng[7][c] =  cplex.addGe(cplex.diff(0, expr), -OptCutChangshu);
        }


//        cplex.setParam(IloCplex.Param.Preprocessing.Presolve, false);
        cplex.setWarning(null);
        cplex.setOut(null);

//        cplex.setParam(IloCplex.Param.WorkMem, 50000);
//        cplex.setParam(IloCplex.Param.Threads, 1);
        cplex.solve();

        double[] yqobj = new double[AircNum+2];
        System.out.println("SARIpStatus = " + cplex.getStatus());

        for(int airc=0; airc<AircNum; airc++){
            yqobj[airc] = -1;
            for(int j=0; j<y[airc].length; j++){
                if(cplex.getValue(y[airc][j])!=0){
                    yqobj[airc] = j;
                    break;
                }
            }
        }

        yqobj[AircNum] = cplex.getValue(q);
        System.out.println("q = " + yqobj[AircNum]);
        yqobj[AircNum+1] = cplex.getObjValue();
        System.out.println("SARIpObj = " + yqobj[AircNum+1]);
        System.out.println("CancelNum = "+(int)cplex.getValue(CancelNum));
        System.out.println("DelayTime = "+(int)cplex.getValue(DelayTime));
        System.out.println("SwapNum = "+(int)cplex.getValue(SwapNum));

//        double[] solz = cplex.getValues(z);
//        System.out.println("z = " + Arrays.toString(solz));


//        for(int Airc=0; Airc<AircNum; Airc++){
//            System.out.println(Arrays.toString(cplex.getValues(y[Airc])));
//        }
//        System.out.println("CancelNum= "+ cplex.getValue(CancelNum));
//        System.out.print(", Cancel FlightID= ");
//        for(int i=0; i<solz.length; i++){
//            if(solz[i] > 0){
//                System.out.print(i+" ");
//            }
//        }
//        System.out.println(" ");
//        System.out.println("DelayTime= "+cplex.getValue(DelayTime));
//        System.out.println("SwapNum= "+cplex.getValue(SwapNum));




        cplex.end();
        return yqobj;
    }

    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException, IloException {
        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;
        int Initial1RouteNum = 1000;
        int NegativeReducedCostRouteNum = 5*DataBase;
        if(DataBase==1){
            ActFlightNum = 24;
            MtsNum = 1;
            AircNum = 4;
            TypeNum = 2;
            ApNum = 7;
        }else if(DataBase==2){
            ActFlightNum = 52;
            MtsNum = 1;
            AircNum = 8;
            TypeNum = 3;
            ApNum = 9;
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
//        System.out.println("DataBase = "+DataBase+", Initial1RouteNum = "+Initial1RouteNum+", NegativeReducedCostRouteNum = "+NegativeReducedCostRouteNum);

        InputDataReader FlightReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Flight.txt");
        int[][] Flight = FlightReader.readIntArrayArray();
        int FlightNum = Flight.length;

        int[][] CopyFlight = new int[ActFlightNum*CopyNum+MtsNum][7];
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

        InputDataReader AircReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Aircraft.txt");
        int[][] Aircraft = AircReader.readIntArrayArray();

        DataReader ApSlotDepReader = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\ApSlotDep.txt");
        DataReader ApSlotArrReader = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\ApSlotArr.txt");
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

//        int[] AircRouteNum = new int[AircNum];
//        int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];
//
//
//        DataReader InitialRouteNumR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\InitialRouteNum"+Initial1RouteNum+".txt");
//        DataReader InitialRouteR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\InitialRoute"+Initial1RouteNum+".txt");
//        for (int airc = 0; airc < AircNum; airc++) {
//            AircRouteNum[airc] = InitialRouteNumR.next();
//            AircLegnumDlytimeSwapnumRoute[airc] = new int[AircRouteNum[airc]][3+MaxRouteLegNum];
//        }
//        for (int airc = 0; airc < AircNum; airc++) {
//            for (int i = 0; i < AircRouteNum[airc]; i++){
//                for(int j=0; j<3+MaxRouteLegNum; j++){
//                    AircLegnumDlytimeSwapnumRoute[airc][i][j] = InitialRouteR.next();
//                }
//            }
//        }
//
//
//        int[] AircReducedCostFlag = new int[AircNum];
//        for(int airc=0; airc<AircNum; airc++){
//            AircReducedCostFlag[airc] = 0;
//        }


        double[][] Jilv = new double[0][];


        for(int a=1; a<=1; a++){
            for(int b=1;b<=1; b++){
                Initial1RouteNum = 1000*a;
                NegativeReducedCostRouteNum = 1000*b;
                System.out.println("\n------------------------------------------------");
                System.out.println("SARdatabase"+DataBase+"."+Initial1RouteNum+"."+NegativeReducedCostRouteNum+" - 626");
                System.out.println("DataBase = "+DataBase+", Initial1RouteNum = "+Initial1RouteNum+", NegativeReducedCostRouteNum = "+NegativeReducedCostRouteNum);

                int[] AircRouteNum = new int[AircNum];
                int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];
                DataReader InitialRouteNumR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRouteNum"+Initial1RouteNum+".txt");
                DataReader InitialRouteR = new DataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRoute"+Initial1RouteNum+".txt");
                for (int airc = 0; airc < AircNum; airc++) {
                    AircRouteNum[airc] = InitialRouteNumR.next();
                    AircLegnumDlytimeSwapnumRoute[airc] = new int[AircRouteNum[airc]][3+MaxRouteLegNum];
                }
                for (int airc = 0; airc < AircNum; airc++) {
                    for (int i = 0; i < AircRouteNum[airc]; i++){
                        for(int j=0; j<3+MaxRouteLegNum; j++){
                            AircLegnumDlytimeSwapnumRoute[airc][i][j] = InitialRouteR.next();
                        }
                    }
                }
                int[] AircReducedCostFlag = new int[AircNum];
                for(int airc=0; airc<AircNum; airc++){
                    AircReducedCostFlag[airc] = 0;
                }

                //列生成迭代
                int loop = 0;
                int loopLim = 100;
                IloCplex timecplex= new IloCplex();
                double t0 = timecplex.getCplexTime();
                while(Arrays.stream(AircReducedCostFlag).sum()<AircNum && loop<loopLim){
                    double t1 = timecplex.getCplexTime();
                    loop+=1;
                    System.out.println("loop" + loop);
                    System.out.println("AircReducedCost >0 Num = "+Arrays.stream(AircReducedCostFlag).sum()+"/"+AircNum);
                    System.out.println("Solve Lp:");
                    double[][] LpDualValue = SARPLP(CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                            AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                            Flight, ApSlotDep, ApSlotArr);

                    for(int airc=0; airc<AircNum; airc++){
                        if(AircReducedCostFlag[airc]==1){
                            continue;
                        }
                        double[] ReducedcostLegnumDlytimeSwapnumRoute =
                                OneBestRoute(airc,
                                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS,
                                        CopyFlightDepT, CopyFlightArrT, CopyFlightDepTSlot, CopyFlightArrTSlot,
                                        CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                                        AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                                        TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]], LpDualValue);
                        System.out.println("At AircNum :" + airc+", ReducedcostCost:" + ReducedcostLegnumDlytimeSwapnumRoute[0]);
                        if(ReducedcostLegnumDlytimeSwapnumRoute[0]>=-epson){
                            AircReducedCostFlag[airc]=1;
                            continue;
                        }
                        AircLegnumDlytimeSwapnumRoute[airc] = Arrays.copyOf(AircLegnumDlytimeSwapnumRoute[airc],AircRouteNum[airc]+1);
                        AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]] = new int[3+MaxRouteLegNum];
                        for(int i=0; i<3+MaxRouteLegNum; i++){
                            AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][i] = (int) ReducedcostLegnumDlytimeSwapnumRoute[1+i];
                        }
                        AircRouteNum[airc] += 1;

                        int[][] AB;
                        AB = NegativeReducedCostRoute(airc,
                                CopyFlightOFID, CopyFlightDepS, CopyFlightArrS,
                                CopyFlightDepT, CopyFlightArrT, CopyFlightDepTSlot, CopyFlightArrTSlot,
                                CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                                AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                                TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]],
                                LpDualValue, ReducedcostLegnumDlytimeSwapnumRoute[0], NegativeReducedCostRouteNum);
                        for (int i = 0; i < AB.length; i++) {
                            if (AB[i][0] != 0) {
                                AircLegnumDlytimeSwapnumRoute[airc] = Arrays.copyOf(AircLegnumDlytimeSwapnumRoute[airc],AircRouteNum[airc]+1);
                                AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]] = new int[3+MaxRouteLegNum];
                                for(int j=0; j<3+AB[i][0]; j++){
                                    AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][j] = AB[i][j];
                                }
                                AircRouteNum[airc] += 1;
                            } else {
                                break;
                            }
                        }

                    }
                    System.out.println("After Column Generation SP");
                    System.out.println("AircReducedCost >0 Num = "+Arrays.stream(AircReducedCostFlag).sum());
                    System.out.println("Loop use time "+(timecplex.getCplexTime()-t1)+"\n");
                }
                double iteTime = timecplex.getCplexTime()-t0;
                System.out.println("Total use time "+iteTime);




                int[] AircChoose = SARPIP(CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                        AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                        Flight, ApSlotDep, ApSlotArr);
                System.out.println(Arrays.toString(AircChoose));
                System.out.println("------------------------------------------------\n");



//                AircLegnumDlytimeSwapnumRoute[airc] = Arrays.copyOf(AircLegnumDlytimeSwapnumRoute[airc],AircRouteNum[airc]+1);
//                AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]] = new int[3+MaxRouteLegNum];
//                for(int i=0; i<3+MaxRouteLegNum; i++){
//                    AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][i] = (int) ReducedcostLegnumDlytimeSwapnumRoute[1+i];
//                }
                Jilv = Arrays.copyOf(Jilv,Jilv.length+1);
                Jilv[Jilv.length-1] = new double[6];
                Jilv[Jilv.length-1][0] = Initial1RouteNum;
                Jilv[Jilv.length-1][1] = NegativeReducedCostRouteNum;
                Jilv[Jilv.length-1][2] = SARPLPv(CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                        AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                        Flight, ApSlotDep, ApSlotArr);
                Jilv[Jilv.length-1][3] = SARPIPv(CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                        AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                        Flight, ApSlotDep, ApSlotArr);
                Jilv[Jilv.length-1][4] = loop;
                Jilv[Jilv.length-1][5] = iteTime;


//                FileWriter RouteNum510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\RouteNum"+1000+".txt");
//                for(int airc=0; airc< Aircraft.length; airc++){
//                    RouteNum510FW.write(AircRouteNum[airc]+"\n");
//                }
//                RouteNum510FW.close();
//
//                //AircLegnumDlytimeSwapnumRoute
//                FileWriter Route510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Route"+1000+".txt");
//                for(int airc=0; airc< Aircraft.length; airc++){
//                    for (int i = 0; i < AircRouteNum[airc]; i++) {
//                        for(int j = 0; j<MaxRouteLegNum+3-1; j++){
//                            Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][j]+"\t");
//                        }
//                        Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][MaxRouteLegNum+3-1]+"\n");
//                    }
//                }
//                Route510FW.close();

//
            }
        }
        for(int i=0; i<Jilv.length; i++){
            System.out.println((i+1)+"\t"+(int)Jilv[i][0]+"\t"+(int)Jilv[i][1]+"\t"+Jilv[i][2]+"\t"+Jilv[i][3]+"\t"+(int)Jilv[i][4]+"\t"+Jilv[i][5]+"\t");
        }



//        int[] AircChoose = SARPIP(CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
//                AircLegnumDlytimeSwapnumRoute, AircRouteNum,
//                Flight, ApSlotDep, ApSlotArr);
//        System.out.println(Arrays.toString(AircChoose));

//        FileWriter RouteNum510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\RouteNum510.txt");
//        for(int airc=0; airc< Aircraft.length; airc++){
//            RouteNum510FW.write(AircRouteNum[airc]+"\n");
//        }
//        RouteNum510FW.close();
//
//        //AircLegnumDlytimeSwapnumRoute
//        FileWriter Route510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG\\Data\\"+DataBase+"\\Route510.txt");
//        for(int airc=0; airc< Aircraft.length; airc++){
//            for (int i = 0; i < AircRouteNum[airc]; i++) {
//                for(int j = 0; j<MaxRouteLegNum+3-1; j++){
//                    Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][j]+"\t");
//                }
//                Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][MaxRouteLegNum+3-1]+"\n");
//            }
//        }
//        Route510FW.close();
    }



}

