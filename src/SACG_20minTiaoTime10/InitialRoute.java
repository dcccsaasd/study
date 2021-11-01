package SACG_20minTiaoTime10;

import ilog.concert.IloException;
import ilog.concert.*;
import ilog.cp.IloCP;
import ilog.cplex.IloCplex;

import java.io.IOException;
import java.io.FileWriter;
import java.util.Arrays;

public class InitialRoute {
    static int DataBase = 10;
    static double epson = 1e-6;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 20;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);

    static int OneCancelCost = 200;
    static int MintDelayCost = 1;
    static int OneSwapCost = 0;
    static int GateCost = 30;

    static int MaxRouteLegNum = 10;
    static int MaxPatternLegNum = 12;
    static int GateCap = 100;


    static int[][] Intial1Route(int AircID, int Initial1RouteNum, int forcedCoveredOFID,
                                int[] CopyFlightOFID, int[] CopyFlightDepS, int[] CopyFlightArrS, int[] CopyFlightDepT, int[] CopyFlightArrT,
                                int[] CopyFlightType, int[] CopyFlightOAID, int[] CopyFlightFlyT, int[] CopyFlightDlyMint,
                                int AircType, int AircStartS, int AircEndS, int AircTurnT, int AircMts,
                                int[] TypeCopyFlightRange, int[] TypeMtsCFID
    )throws IloException {

        int MaxLegNum = MaxRouteLegNum;
        int MaxFlyTime = 600;

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


        //强制覆盖 forcedCoveredOFID
        if(forcedCoveredOFID != -1){
            IloIntExpr[] flightvarsOrignIndex = cp.intExprArray(MaxLegNum);
            for(int i=0; i<MaxLegNum; i++){
                flightvarsOrignIndex[i] = cp.element(CopyFlightOFID, flightvars[i]);
            }
            cp.addGe(cp.count(flightvarsOrignIndex, forcedCoveredOFID), 1 );
        }

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
        cp.setParameter(IloCP.DoubleParam.TimeLimit,60);

//        cp.solve();
//        System.out.println(cp.getStatus());

        cp.startNewSearch();
        System.out.println("AircID: "+AircID+", forcedCoveredFlightOrignIndex: "+forcedCoveredOFID);

        int[][] LegnumDlytimeSwapnumRoute = new int[Initial1RouteNum][3+MaxLegNum];
        int findflag = 0;
        for(int n=0; n<Initial1RouteNum; n++){
            if(cp.next()){
                findflag += 1;
                LegnumDlytimeSwapnumRoute[n][0] = (int) cp.getValue(Legnum);
                LegnumDlytimeSwapnumRoute[n][1] = (int) cp.getValue(Dlytime);
                LegnumDlytimeSwapnumRoute[n][2] = (int) cp.getValue(Swapnum);
                for(int i=0;i<MaxLegNum;i++){
                    LegnumDlytimeSwapnumRoute[n][i+3] = (int) cp.getValue(flightvars[i]);
//                    System.out.print((int)cp.getValue(flightvars[i])+" ");
                }
//                System.out.println(" ");
            }else{
                System.out.println("AircID: "+AircID+" find " +findflag+ " route(s)");

                cp.end();
//                for(int i=0; i<LegnumDlytimeSwapnumRoute.length; i++)
//                    System.out.println(Arrays.toString(LegnumDlytimeSwapnumRoute[i]));
                return LegnumDlytimeSwapnumRoute;
            }
        }


        cp.end();
//        for(int i=0; i<LegnumDlytimeSwapnumRoute.length; i++)
//            System.out.println(Arrays.toString(LegnumDlytimeSwapnumRoute[i]));
        return LegnumDlytimeSwapnumRoute;
    }

    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException, IloException {
        InputDataReader FlightReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\Flight.txt");
        int[][] Flight = FlightReader.readIntArrayArray();

        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;

        int seqFlag = 0;
        int gateTypeNum = 2;
        int GateTypeDivideline = 0;
        int BigloopLim = 100;
        int SAloopLim = 100;
        int GRloopLim = 100;
        int SARLpValueChongfuNumLim = 5;
        int GRLpValueChongfuNumLim = 5;
        double SAL=0;
        int InitialRouteSize = 0;
        int InitialPatternNum = 1000;
        int Initial3PatternNum = 0;
        if(DataBase==1){
            ActFlightNum = 24;
            MtsNum = 1;
            AircNum = 4;
            TypeNum = 2;
            ApNum = 7;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 15.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 1;
                SAL = 17.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 100;
            Initial3PatternNum = 1000;
        }else if(DataBase==2){
            ActFlightNum = 52;
            MtsNum = 1;
            AircNum = 8;
            TypeNum = 3;
            ApNum = 9;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 35.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 1;
                SAL = 36.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 400;
            Initial3PatternNum = 1000;
        }else if(DataBase==3){
            ActFlightNum = 98;
            MtsNum = 2;
            AircNum = 15;
            TypeNum = 3;
            ApNum = 14;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 34.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 1;
                SAL = 70.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 1200;
            Initial3PatternNum = 1000;
        }else if(DataBase==4){
            ActFlightNum = 150;
            MtsNum = 2;
            AircNum = 24;
            TypeNum = 4;
            ApNum = 18;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 34.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 1;
                SAL = 110.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 1200;
            Initial3PatternNum = 1000;
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

        InputDataReader AircReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\Aircraft.txt");
        int[][] Aircraft = AircReader.readIntArrayArray();

//        InputDataReader ApSlotDepReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\ApSlotDep.txt");
//        int[][] ApSlotDep = ApSlotDepReader.readIntArrayArray();
//
//        InputDataReader ApSlotArrReader = new InputDataReader("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\ApSlotArr.txt");
//        int[][] ApSlotArr = ApSlotArrReader.readIntArrayArray();

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


        int[] AircID = new int[Aircraft.length];
        int[] AircType = new int[Aircraft.length];
        int[] AircStartS = new int[Aircraft.length];
        int[] AircEndS = new int[Aircraft.length];
        int[] AircTurnT = new int[Aircraft.length];
        int[] AircMts = new int[Aircraft.length];

        for(int airc=0; airc<Aircraft.length; airc++){
            AircID[airc] = Aircraft[airc][0];
            AircType[airc] = Aircraft[airc][1];
            AircStartS[airc] = Aircraft[airc][2];
            AircEndS[airc] = Aircraft[airc][3];
            AircTurnT[airc] = Aircraft[airc][4];
            AircMts[airc] = Aircraft[airc][5];
        }


        int Initial1RouteNum;
        for(int a=1; a<=1; a++){
            Initial1RouteNum = 1000*a;



            int[][][] AircLegnumDlytimeSwapnumRoute = new int[Aircraft.length][100000][3+MaxRouteLegNum];
            int[] AircRouteNum = new int[Aircraft.length];
            int[] FlightCoverNum = new int[Flight.length];

            //初始化1
//        int Initial1RouteNum = 1000;
            for(int airc=0; airc< Aircraft.length; airc++){
                int[][] AB;
                AB = Intial1Route(airc, Initial1RouteNum, -1,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                        CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                        AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                        TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]]);
                for (int i = 0; i < AB.length; i++) {
                    if (AB[i][0] != 0) {
                        for(int j=0; j<3; j++){
                            AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][j] = AB[i][j];
                        }
                        for(int j=3; j<3+AB[i][0]; j++){
                            AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][j] = AB[i][j];
                            FlightCoverNum[CopyFlightOFID[AB[i][j]]] += 1;
                        }
                        AircRouteNum[airc] += 1;
                    } else {
                        break;
                    }
                }
            }

            System.out.println("初始化1未被覆盖原航班：");
            for(int f=0; f< Flight.length; f++){
                if(FlightCoverNum[f]==0){
                    System.out.print(f+"\t");
                }
            }
            System.out.println(" ");


            //初始化2
            int Initial2RouteNum = 10;
            for(int f=0; f<Flight.length; f++){
                if(FlightCoverNum[f]==0){
                    for(int airc=0; airc< Aircraft.length; airc++){
                        if(Flight[f][5]==AircType[airc]){
                            int[][] AB;
                            AB = Intial1Route(airc, Initial2RouteNum, f,
                                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                                    CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                                    AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                                    TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]]);
                            for (int i = 0; i < AB.length; i++) {
                                if (AB[i][0] != 0) {
                                    AircRouteNum[airc] += 1;
                                    for(int j=0; j<3; j++){
                                        AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][j] = AB[i][j];
                                    }
                                    for(int j=3; j<3+AB[i][0]; j++){
                                        AircLegnumDlytimeSwapnumRoute[airc][AircRouteNum[airc]][j] = AB[i][j];
                                        FlightCoverNum[CopyFlightOFID[AB[i][j]]] += 1;
                                    }
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            System.out.println("初始化2未被覆盖原航班：");
            for(int f=0; f< Flight.length; f++){
                if(FlightCoverNum[f]==0){
                    System.out.print(f+"\t");
                }
            }
            System.out.println(" ");

            FileWriter InitialRouteNumFW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialRouteNum"+Initial1RouteNum+".txt");
            for(int airc=0; airc< Aircraft.length; airc++){
                InitialRouteNumFW.write(AircRouteNum[airc]+"\n");
            }
            InitialRouteNumFW.close();

            //AircLegnumDlytimeSwapnumRoute
            FileWriter InitialRouteFW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_20minTiaoTime10\\Data\\"+DataBase+"\\InitialRoute"+Initial1RouteNum+".txt");
            for(int airc=0; airc< Aircraft.length; airc++){
                for (int i = 0; i < AircRouteNum[airc]; i++) {
                    for(int j = 0; j<MaxRouteLegNum+3-1; j++){
                        InitialRouteFW.write(AircLegnumDlytimeSwapnumRoute[airc][i][j]+"\t");
                    }
                    InitialRouteFW.write(AircLegnumDlytimeSwapnumRoute[airc][i][MaxRouteLegNum+3-1]+"\n");
                }
            }
            InitialRouteFW.close();







        }








    }
}

