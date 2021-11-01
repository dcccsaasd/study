package SACG_5minTiaoTime;

import ilog.concert.*;
import ilog.cp.IloCP;
import ilog.cplex.IloCplex;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class SAGR {
    static double epson = 1e-6;

    static int MaxDelayTime = 120;
    static int OneDelayTime = 5;
    static int CopyNum = (int) (MaxDelayTime/OneDelayTime +1);

    static int OneCancelCost = 200;
    static int MintDelayCost = 1;
    static int OneSwapCost = 0;
    static int GateCost = 30;

    static int MaxRouteLegNum = 10;
    static int MaxPatternLegNum = 12;
    static int GateCap = 100;

    static void TT(int DataBase, double forcedCoveredPairInitialPatternTimeLim, double OneBestPatternTimeLim, double NegativeReducedCostPatternTimeLim, double TotalTimeLim) throws IOException, InputDataReader.InputDataReaderException, IloException {
        /*定义各数据集相关参数*/
        int ActFlightNum = 0;
        int MtsNum = 0;
        int AircNum = 0;
        int TypeNum = 0;
        int ApNum = 0;
        int SlotNum = 24;

        int seqFlag = 0;
        int gateTypeNum = 2;
        int GateTypeDivideline = 0;
        int BigloopLim = 10;
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
                InitialRouteSize = 100;
                Initial3PatternNum = 1000;
            }else{
                GateTypeDivideline = 1;
                SAL = 17.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
                InitialRouteSize = 100;
                Initial3PatternNum = 1000;
            }
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
                InitialRouteSize = 400;
                Initial3PatternNum = 800;
            }else{
                GateTypeDivideline = 1;
                SAL = 36.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
                InitialRouteSize = 400;
                Initial3PatternNum = 1000;
            }
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
                InitialRouteSize = 1200;
                Initial3PatternNum = 1000;
            }else{
                GateTypeDivideline = 1;
                SAL = 77.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
                InitialRouteSize = 1200;
                Initial3PatternNum = 1000;
            }
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
                GateTypeDivideline = 2;
                SAL = 110.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
                InitialRouteSize = 1200;
                Initial3PatternNum = 1000;
            }
        }else if(DataBase==5){
            ActFlightNum = 198;
            MtsNum = 3;
            AircNum = 32;
            TypeNum = 4;
            ApNum = 17;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 34.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 2;
                SAL = 170.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
                InitialRouteSize = 1000;
                Initial3PatternNum = 1000;
            }
        }else if(DataBase==6){
            ActFlightNum = 250;
            MtsNum = 3;
            AircNum = 41;
            TypeNum = 4;
            ApNum = 22;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 34.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 2;
                SAL = 185.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 1000;
            Initial3PatternNum = 1000;
        }else if(DataBase==7){
            ActFlightNum = 295;
            MtsNum = 3;
            AircNum = 49;
            TypeNum = 5;
            ApNum = 26;
            if(gateTypeNum == 1){
                GateTypeDivideline = TypeNum;
                SAL = 34.000001;

                SAL = Math.floor(SAL)*GateCost+1e-6;
            }else{
                GateTypeDivideline = 2;
                SAL = 198.000001;
                SAL = Math.floor(SAL)*GateCost+1e-6;
            }
            InitialRouteSize = 1000;
            Initial3PatternNum = 1000;
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

        System.out.println("DataBase = "+DataBase);
        System.out.println("OneDelayTime = " +OneDelayTime);
        System.out.println("gateTypeNum = " + gateTypeNum);
        System.out.println("SAL = " + SAL);
        System.out.println("InitialRouteNum =  "+InitialRouteSize);
        System.out.println("InitialPatternNum = " + InitialPatternNum);
        System.out.println("Initial3PatternNum = " + Initial3PatternNum);
        System.out.println("forcedCoveredPairInitialPatternTimeLim = " + forcedCoveredPairInitialPatternTimeLim);
        System.out.println("OneBestPatternTimeLim = " + OneBestPatternTimeLim);
        System.out.println("NegativeReducedCostPatternTimeLim = " + NegativeReducedCostPatternTimeLim);
        System.out.println("TotalTimeLim = " + TotalTimeLim);

        /*航班数据导入*/
        InputDataReader FlightReader = new InputDataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Flight.txt");
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

        /*构造复制航班*/
        int CopyFlightNum = ActFlightNum*CopyNum+MtsNum;
        int[][] CopyFlight = new int[CopyFlightNum][7];
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
        int[] CopyFlightOFID = new int[CopyFlightNum];
        int[] CopyFlightDepS = new int[CopyFlightNum];
        int[] CopyFlightArrS = new int[CopyFlightNum];
        int[] CopyFlightDepT = new int[CopyFlightNum];
        int[] CopyFlightArrT = new int[CopyFlightNum];
        int[] CopyFlightType = new int[CopyFlightNum];
        int[] CopyFlightOAID = new int[CopyFlightNum];
        int[] CopyFlightFlyT = new int[CopyFlightNum];
        int[] CopyFlightDlyMint = new int[CopyFlightNum];
        int[][] TypeCopyFlightRange= new int[TypeNum][2];
        int[] TypeMtsNum = new int[TypeNum];
        int[] CopyFlightDepTSlot = new int[CopyFlightNum];
        int[] CopyFlightArrTSlot = new int[CopyFlightNum];
        for(int tp=0; tp<TypeNum; tp++){
            TypeCopyFlightRange[tp][0] = CopyFlightNum;
            TypeCopyFlightRange[tp][1] = 0;
        }
        for(int cf=0; cf<CopyFlightNum; cf++){
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
        /*统计各机型维修的复制航班编号*/
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

        /*飞机数据导入*/
        InputDataReader AircReader = new InputDataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Aircraft.txt");
        int[][] Aircraft = AircReader.readIntArrayArray();
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

        /*时隙数据导入*/
        DataReader ApSlotDepReader = new DataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\ApSlotDep.txt");
        DataReader ApSlotArrReader = new DataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\ApSlotArr.txt");
        int[][] ApSlotDep = new int[ApNum][SlotNum];
        int[][] ApSlotArr = new int[ApNum][SlotNum];
        for (int ap = 0; ap < ApNum; ap++) {
            for(int t=0; t<SlotNum; t++){
                ApSlotDep[ap][t] = ApSlotDepReader.next();
                ApSlotArr[ap][t] = ApSlotArrReader.next();
            }
        }


        int[] AircRouteNum = new int[AircNum];
        int[][][] AircLegnumDlytimeSwapnumRoute = new int[AircNum][][];
        /*初始飞机路线导入*/
        String InitialRouteNumRString;
        String InitialRouteRString;

//        if(seqFlag == 0){
//            InitialRouteNumRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\RouteNum"+InitialRouteSize+".txt";
//            InitialRouteRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Route"+InitialRouteSize+".txt";
//        }else{
//            InitialRouteNumRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRouteNum"+InitialRouteSize+".txt";
//            InitialRouteRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRoute"+InitialRouteSize+".txt";
//        }

//        InitialRouteNumRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRouteNum"+InitialRouteSize+".txt";
//        InitialRouteRString = "D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialRoute"+InitialRouteSize+".txt";
        InitialRouteNumRString = "D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\RouteNum"+InitialRouteSize+".txt";
        InitialRouteRString = "D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\Route"+InitialRouteSize+".txt";
        DataReader InitialRouteNumR = new DataReader(InitialRouteNumRString);
        DataReader InitialRouteR = new DataReader(InitialRouteRString);

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


        int[][] AirpTypePatternNum = new int[ApNum][2];
        int[][][][] AirpTypeLegnumPattern = new int[ApNum][2][][];
        /*初始机位Pattern导入*/
        DataReader InitialPatternNumR = new DataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialPatternNum"+InitialPatternNum+"~"+gateTypeNum+".txt");
        DataReader InitialPatternR = new DataReader("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\InitialPattern"+InitialPatternNum+"~"+gateTypeNum+".txt");
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
        //机场相关数据处理 各机场各类型停机位是否使用
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


        int Bigloop = 0;
        double LB = -Double.MAX_VALUE;
        double UB = Double.MAX_VALUE;
        double UBGRobj = Double.MAX_VALUE;

        int[] y = new int[AircNum];


        int[][] AllyCoverCf = new int[0][];
        int[][][] AllyCoverCfPair = new int[0][][];
        int[][] AllSAROptSol =  new int[0][];//只记录每架飞机选择的路线编号，没选的话为-1
        int[] AllSAROptSolFea2GR =  new int[0];
        int[] AllSAROptSolInfea2GR =  new int[0];
        double[] AllGRIPOptVal = new double[0];
        double[][][] AllGRLpOptDualValue = new double[0][][];

        double[][][] AllGRLpPhase1InfeaDualValue = new double[0][][];

        IloCplex Totaltimecplex = new IloCplex();
        double Totalt0 = Totaltimecplex.getCplexTime();

        if(seqFlag == 1){
            System.out.println("\n---------------------------------");
            System.out.println("seqFlag == 1");
            BigloopLim = 0;

            double[] ySARqSARIPobj = SAR.SARIP5(AllGRLpOptDualValue, AllyCoverCf, AllyCoverCfPair,
                    AllSAROptSol,  AllSAROptSolFea2GR, AllSAROptSolInfea2GR, AllGRIPOptVal, SAL,
                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                    AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                    Flight, ApSlotDep, ApSlotArr);

            double SARIPobj = ySARqSARIPobj[AircNum+1];


            if(SARIPobj>LB){
                LB = SARIPobj;
                System.out.println("SARq>LB, update LB = " + LB);
            }
            System.out.println(" ");



            for(int airc=0; airc<AircNum; airc++){
                y[airc] = (int) ySARqSARIPobj[airc];
            }
            System.out.println("Airc choose route's index = "+Arrays.toString(y));


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

            int[] yCoverCf01 = new int[CopyFlightNum];
            int[] yCoverCf = new int[0];
            for(int f = 0; f<FlightNum; f++){
                if(fltsOpt[f]!=-1) {
                    yCoverCf01[fltsOpt[f]]=1;
                    yCoverCf = Arrays.copyOf(yCoverCf, yCoverCf.length+1);
                    yCoverCf[yCoverCf.length-1] = fltsOpt[f];
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

            System.out.print("Need find "+(yCoverCfPair.length)+" pairs: ");
            int jinduFlag = 0;
            for(int p=0; p<yCoverCfPair.length; p++) {
                System.out.print(jinduFlag+" ");
                jinduFlag += 1;

                int[] Pair1 = yCoverCfPair[p];

                int ap = CopyFlightArrS[Pair1[0]];
                int tp = CopyFlightType[Pair1[0]] < GateTypeDivideline ? 0 : 1;
                AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp], AirpTypePatternNum[ap][tp] + 1);
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1 + MaxPatternLegNum];
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = 2;
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][1] = Pair1[0];
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][2] = Pair1[1];
                AirpTypePatternNum[ap][tp] += 1;

                int[][] AB2 = GR.forcedCoveredPairInitialPattern(forcedCoveredPairInitialPatternTimeLim, ap, tp, Initial3PatternNum, -1, -1, yCoverCfPair[p],
                        GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                        FlightType, FlightDepS, FlightArrS,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
                for (int i = 0; i < AB2.length; i++) {
                    if (AB2[i][0] != 0) {
//                        System.out.println(Arrays.toString(AB2[i]));
                        AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp], AirpTypePatternNum[ap][tp] + 1);
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1 + MaxPatternLegNum];
                        for (int j = 0; j < 1; j++) {
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
                        }
                        for (int j = 1; j < 1 + AB2[i][0]; j++) {
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
                        }
                        AirpTypePatternNum[ap][tp] += 1;
                    } else {
                        break;
                    }
                }
            }
            System.out.println(" ");


            IloCplex GRIPcplex = GR.GRIP2(yCoverCf01, yCoverCfPair,
                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                    AirpTypeLegnumPattern, AirpTypePatternNum);
            GRIPcplex.solve();

            if(GRIPcplex.getStatus().equals(IloCplex.Status.Infeasible)){
                System.out.println("!GRIP is Infeasible");
                GRIPcplex.end();
            }else{
                double GRobj = GRIPcplex.getObjValue();
                System.out.println("GRIPobj = "+GRobj);
                GRIPcplex.end();


                if((SARIPobj+GRobj) < UB){
                    UB = SARIPobj+GRobj;
                    System.out.println("(SARIPobj-SARq+GRobj)<UB, update UB = " + UB);
                }
                if((SARIPobj+GRobj) <= UB && GRobj< UBGRobj){
                    UBGRobj = GRobj;
                }
            }
        }



        //Bigloop--------------------------------
        while (Bigloop<BigloopLim && (Totaltimecplex.getCplexTime()-Totalt0) < TotalTimeLim) {
            System.out.println("---------------------------------");
            Bigloop +=1 ;
            System.out.println("Bigloop"+Bigloop+"\n");
            System.out.println("SARLP Column Generation:");
            //SA列生成迭代
            int SAloop = 0;
            int[] AircReducedCostFlag = new int[AircNum];
            for(int airc=0; airc<AircNum; airc++){
                AircReducedCostFlag[airc] = 0;
            }
            IloCplex SAtimecplex= new IloCplex();
            double SARt0 = SAtimecplex.getCplexTime();
            double SARLpValue = 1e6;
            int SRLpValueChongfuNum = 0;

            while(Arrays.stream(AircReducedCostFlag).sum()<AircNum && SAloop<SAloopLim){
                SAloop+=1;

                System.out.println("SALPloop" + SAloop);

                double[][] LpDualValueLpValue = SAR.SARLPGRphase1(AllGRLpOptDualValue, AllyCoverCf, AllyCoverCfPair,
                        AllSAROptSol,  AllSAROptSolFea2GR, AllSAROptSolInfea2GR, AllGRLpPhase1InfeaDualValue,
                        AllGRIPOptVal, SAL,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                        AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                        Flight, ApSlotDep, ApSlotArr);

                double LpValue = LpDualValueLpValue[LpDualValueLpValue.length-1][0];
                if(Math.abs(LpValue-SARLpValue)>epson){
                    SARLpValue = LpValue;
                    SRLpValueChongfuNum = 0;
                }else{
                    SRLpValueChongfuNum += 1;
                    if(SRLpValueChongfuNum>=SARLpValueChongfuNumLim){
                        break;
                    }
                }

                double[][] LpDualValue = new double[LpDualValueLpValue.length-1][];
                for(int i=0; i<4; i++){
                    LpDualValue[i] = new double[LpDualValueLpValue[i].length];
                    for(int j=0; j<LpDualValueLpValue[i].length; j++){
                        LpDualValue[i][j] = LpDualValueLpValue[i][j];
                    }
                }

                if(AllGRLpOptDualValue.length>0){
                    LpDualValue[4] = new double[LpDualValueLpValue[4].length];
                    for(int j=0; j<LpDualValueLpValue[4].length; j++){
                        LpDualValue[4][j] = LpDualValueLpValue[4][j];
                    }
                }
                if(AllSAROptSolFea2GR.length>0){
                    LpDualValue[5] = new double[LpDualValueLpValue[5].length];
                    for(int j=0; j<LpDualValueLpValue[5].length; j++){
                        LpDualValue[5][j] = LpDualValueLpValue[5][j];
                    }
                }
                if(AllSAROptSolInfea2GR.length>0){
                    LpDualValue[6] = new double[LpDualValueLpValue[6].length];
                    for(int j=0; j<LpDualValueLpValue[6].length; j++){
                        LpDualValue[6][j] = LpDualValueLpValue[6][j];
                    }
                }
                if(AllGRLpPhase1InfeaDualValue.length>0){
                    LpDualValue[7] = new double[LpDualValueLpValue[7].length];
                    for(int j=0; j<LpDualValueLpValue[7].length; j++){
                        LpDualValue[7][j] = LpDualValueLpValue[7][j];
                    }
                }

                System.out.print(AircNum + " aircraft need pricing : ");
                for(int airc=0; airc<AircNum; airc++){
                    System.out.print(airc+" ");
                    if(AircReducedCostFlag[airc]==1){
                        continue;
                    }

                    double[] ReducedcostLegnumDlytimeSwapnumRoute =
                            SAR.OneBestRouteGRphase1(airc, LpDualValue, AllGRLpOptDualValue, AllyCoverCf, AllyCoverCfPair,
                                    AllSAROptSolFea2GR, AllSAROptSolInfea2GR, AllGRLpPhase1InfeaDualValue,
                                    AllGRIPOptVal, SAL,
                                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS,
                                    CopyFlightDepT, CopyFlightArrT, CopyFlightDepTSlot, CopyFlightArrTSlot,
                                    CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                                    AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                                    TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]]);

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
                    AB = SAR.NegativeReducedCostRouteGRphase1(airc, LpDualValue, AllGRLpOptDualValue, AllyCoverCf, AllyCoverCfPair,
                            AllSAROptSolFea2GR, AllSAROptSolInfea2GR, AllGRLpPhase1InfeaDualValue,
                            AllGRIPOptVal, SAL,
                            CopyFlightOFID, CopyFlightDepS, CopyFlightArrS,
                            CopyFlightDepT, CopyFlightArrT, CopyFlightDepTSlot, CopyFlightArrTSlot,
                            CopyFlightType, CopyFlightOAID, CopyFlightFlyT, CopyFlightDlyMint,
                            AircType[airc], AircStartS[airc], AircEndS[airc], AircTurnT[airc], AircMts[airc],
                            TypeCopyFlightRange[AircType[airc]], TypeMtsCFID[AircType[airc]], 1000);
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
                System.out.println(" ");
                System.out.println("AircReducedCost >0 Num = "+Arrays.stream(AircReducedCostFlag).sum()+"/"+AircNum+"\n");
            }
            System.out.println("SARLP CG use time "+(SAtimecplex.getCplexTime()-SARt0)+"\n");
            SAtimecplex.end();



//            FileWriter RouteNum510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\betterRouteNum"+1000+".txt");
//            for(int airc=0; airc< Aircraft.length; airc++){
//                RouteNum510FW.write(AircRouteNum[airc]+"\n");
//            }
//            RouteNum510FW.close();
//
//            //AircLegnumDlytimeSwapnumRoute
//            FileWriter Route510FW = new FileWriter("D:\\idea_javaproject\\idea_javaproject\\src\\SACG_5minTiaoTime\\Data\\"+DataBase+"\\betterRoute"+1000+".txt");
//            for(int airc=0; airc< Aircraft.length; airc++){
//                for (int i = 0; i < AircRouteNum[airc]; i++) {
//                    for(int j = 0; j<MaxRouteLegNum+3-1; j++){
//                        Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][j]+"\t");
//                    }
//                    Route510FW.write(AircLegnumDlytimeSwapnumRoute[airc][i][MaxRouteLegNum+3-1]+"\n");
//                }
//            }
//            Route510FW.close();


            double[] ySARqSARIPobj = SAR.SARIP5(AllGRLpOptDualValue, AllyCoverCf, AllyCoverCfPair,
                    AllSAROptSol,  AllSAROptSolFea2GR, AllSAROptSolInfea2GR, AllGRIPOptVal, SAL,
                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT,
                    AircLegnumDlytimeSwapnumRoute, AircRouteNum,
                    Flight, ApSlotDep, ApSlotArr);

            double SARq = ySARqSARIPobj[AircNum];
            double SARIPobj = ySARqSARIPobj[AircNum+1];

            if(SARIPobj>LB){
                LB = SARIPobj;
                System.out.println("SARq>LB, update LB = " + LB);
            }
            System.out.println(" ");

            if((UB-LB)/UB<=epson){
                System.out.println("\n---------------------------------");
                System.out.println("LB = "+LB+", UB= "+UB);
                System.out.println("Satisfy stop criterion and break;");
                break;
            }


            for(int airc=0; airc<AircNum; airc++){
                y[airc] = (int) ySARqSARIPobj[airc];
            }
            System.out.println("Airc choose route's index = "+Arrays.toString(y));

            int SameRouteFlag = 0;
            for(int i =0; i<AllSAROptSol.length; i++){
                int SameNum = 0;
                for(int j=0; j<AircNum; j++){
                    if(AllSAROptSol[i][j]!=y[j]){
                        break;
                    }
                    if(AllSAROptSol[i][j]==y[j]){
                        SameNum +=1;
                    }
                }
                if(SameNum == AircNum){
                    System.out.println("Same Airc routes!!!");
                    SameRouteFlag = 1;
                    break;
                }
            }

            if(SameRouteFlag == 1){
                break;
            }

            //更新 AllSAROptSol
            AllSAROptSol = Arrays.copyOf(AllSAROptSol, AllSAROptSol.length+1);
            AllSAROptSol[AllSAROptSol.length-1] = new int[AircNum];
            for(int airc=0; airc<AircNum; airc++){
                AllSAROptSol[AllSAROptSol.length-1][airc] = y[airc];
            }

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

//                System.out.println(Arrays.toString(chooseRoute));
            }

            int[] yCoverCf01 = new int[CopyFlightNum];
            int[] yCoverCf = new int[0];
            for(int f = 0; f<FlightNum; f++){
                if(fltsOpt[f]!=-1) {
                    yCoverCf01[fltsOpt[f]]=1;
                    yCoverCf = Arrays.copyOf(yCoverCf, yCoverCf.length+1);
                    yCoverCf[yCoverCf.length-1] = fltsOpt[f];
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

            int[] NoNeedPair = new int[conNum];
            for(int con=0; con<conNum; con++){
                int[] Pair1 = yCoverCfPair[con];

                int ap = CopyFlightArrS[Pair1[0]];
                int tp = CopyFlightType[Pair1[0]] < GateTypeDivideline ? 0 : 1;
                AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp], AirpTypePatternNum[ap][tp] + 1);
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1 + MaxPatternLegNum];
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = 2;
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][1] = Pair1[0];
                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][2] = Pair1[1];
                AirpTypePatternNum[ap][tp] += 1;


                int outFlag = 0;
                for(int i=0; i<AllyCoverCfPair.length; i++){
                    for(int j=0; j<AllyCoverCfPair[i].length; j++){
                        int[] Pair2 = AllyCoverCfPair[i][j];
                        if(Pair1[0]==Pair2[0]&&Pair1[1]==Pair2[1]){
                            outFlag = 1;
                            NoNeedPair[con]=1;
                            break;
                        }
                    }
                    if(outFlag == 1){
                        break;
                    }
                }
            }


            //初始化3：为y覆盖的Pair找Initial3PatternNum个Pattern
            System.out.print("Need find "+(yCoverCfPair.length - Arrays.stream(NoNeedPair).sum())+" pairs: ");
            int jinduFlag = 0;
            for(int p=0; p<yCoverCfPair.length; p++) {
                if(NoNeedPair[p]==1){
                    continue;
                }

                System.out.print(jinduFlag+" ");
                jinduFlag += 1;

                int[] Pair = yCoverCfPair[p];
                int ap = CopyFlightArrS[Pair[0]];
                int tp = CopyFlightType[Pair[0]] < GateTypeDivideline ? 0 : 1;

                int[][] AB2 = GR.forcedCoveredPairInitialPattern(forcedCoveredPairInitialPatternTimeLim, ap, tp, Initial3PatternNum, -1, -1, yCoverCfPair[p],
                        GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                        FlightType, FlightDepS, FlightArrS,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
                for (int i = 0; i < AB2.length; i++) {
                    if (AB2[i][0] != 0) {
                        AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp], AirpTypePatternNum[ap][tp] + 1);
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1 + MaxPatternLegNum];
                        for (int j = 0; j < 1; j++) {
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
                        }
                        for (int j = 1; j < 1 + AB2[i][0]; j++) {
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB2[i][j];
                        }
                        AirpTypePatternNum[ap][tp] += 1;
                    } else {
                        break;
                    }
                }
            }
            System.out.println("\n");


            //GR列生成迭代

            IloCplex GRtimecplex= new IloCplex();
            double GRt0 = GRtimecplex.getCplexTime();


            //可行性
            int GRphase1loop = 0;
            System.out.println("GRLPphase1 Column Generation:");
            int[][] phase1AirpTypeReducedcostFlag = new int[ApNum][2];
            int phase1AirpTypeReducedcostFlagSum = 0;
            for(int ap=0; ap<ApNum; ap++){
                for(int tp=0; tp<2; tp++){
                    if(AirpTypeFNum[ap][tp]==0) {
                        phase1AirpTypeReducedcostFlag[ap][tp] = 1;
                        phase1AirpTypeReducedcostFlagSum += 1;
                    }
                }
            }


            int GRphase1ContinueFlag = 0;
            while (true){
                GRphase1loop+=1;
                System.out.println("GRphase1loop" + GRphase1loop);

                IloCplex GRLPphase1cplex = GR.GRLPphase1(yCoverCf01, yCoverCfPair,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                        AirpTypeLegnumPattern, AirpTypePatternNum);
                GRLPphase1cplex.solve();

                if(!(GRLPphase1cplex.getStatus().equals(IloCplex.Status.Infeasible))){
                    System.out.println("GRLPphase1 is feasible！");
                    GRLPphase1cplex.end();
                    System.out.println("GRLPphase1 CG use time "+(GRtimecplex.getCplexTime()-GRt0));
                    System.out.println("Having used time "+(Totaltimecplex.getCplexTime()-Totalt0)+"\n");
                    break;
                }

                GRLPphase1cplex.end();
                System.out.println("GRLPphase1 is infeasible for the current patterns！");

                double[][] phase1DualValue = GR.GRLPphase1DualValue(yCoverCf01, yCoverCfPair,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                        AirpTypeLegnumPattern, AirpTypePatternNum);

                System.out.print(2*ApNum + " gates need pricing: ");
                for(int ap=0; ap<ApNum; ap++) {
                    for(int tp = 0; tp < 2; tp++) {
                        System.out.print((2*ap+tp) + " ");
                        if(phase1AirpTypeReducedcostFlag[ap][tp] == 1){
                            continue;
                        }
                        double[] ReducedcostLegnumPattern = GR.phase1OneBestPattern(phase1DualValue, yCoverCfPair,
                                ap, tp, -1, -1, GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                                FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);
                        if(ReducedcostLegnumPattern[0]>=-epson){
                            phase1AirpTypeReducedcostFlag[ap][tp] = 1;
                            phase1AirpTypeReducedcostFlagSum += 1;
                            continue;
                        }
                        AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = (int) ReducedcostLegnumPattern[1];
                        for(int i=1; i<1+AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0]; i++){
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][i] = (int) ReducedcostLegnumPattern[1+i];
                        }
                        AirpTypePatternNum[ap][tp] += 1;

                        //负检验数对应列
                        int[][] AB;
                        AB = GR.phase1NegativeReducedCostPattern(phase1DualValue, yCoverCfPair, ap, tp, 100, -1, -1,
                                GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                                FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);

                        for (int i = 0; i < AB.length; i++) {
                            if (AB[i][0] != 0) {
                                AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
                                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
                                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = AB[i][0];
                                for(int j=1; j<1+AB[i][0]; j++){
                                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
                                }
                                AirpTypePatternNum[ap][tp] += 1;
                            } else {
                                break;
                            }
                        }
                    }
                }

                if(phase1AirpTypeReducedcostFlagSum==2*ApNum){
                    IloCplex GRLPphase1cplex2 = GR.GRLPphase1(yCoverCf01, yCoverCfPair,
                            CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                            AirpTypeLegnumPattern, AirpTypePatternNum);
                    GRLPphase1cplex2.solve();

                    if(!(GRLPphase1cplex2.getStatus().equals(IloCplex.Status.Infeasible))){
                        System.out.println("GRLPphase1 is feasible！");
                        GRLPphase1cplex2.end();
                        System.out.println("GRLPphase1 CG use time "+(GRtimecplex.getCplexTime()-GRt0));
                        System.out.println("Having used time "+(Totaltimecplex.getCplexTime()-Totalt0)+"\n");
                        break;
                    }
                    GRLPphase1cplex2.end();
                    System.out.println("GRLPphase1 is infeasible for the all patterns！");

                    GRphase1ContinueFlag = 1;

                    AllSAROptSolInfea2GR = Arrays.copyOf(AllSAROptSolInfea2GR, AllSAROptSolInfea2GR.length+1);
                    AllSAROptSolInfea2GR[AllSAROptSolInfea2GR.length-1] = AllSAROptSol.length-1;
                    System.out.println("LB = "+LB+", UB= "+UB);

                    //可作为 可行割 Benders子问题对偶问题的极方向
                    AllGRLpPhase1InfeaDualValue = Arrays.copyOf(AllGRLpPhase1InfeaDualValue, AllGRLpPhase1InfeaDualValue.length+1);
                    AllGRLpPhase1InfeaDualValue[AllGRLpPhase1InfeaDualValue.length-1] = new double[4][];
                    for(int i=0; i<2; i++){
                        AllGRLpPhase1InfeaDualValue[AllGRLpPhase1InfeaDualValue.length-1][i] = new double[CopyFlightNum];
                    }
                    AllGRLpPhase1InfeaDualValue[AllGRLpPhase1InfeaDualValue.length-1][2] = new double[yCoverCfPair.length];
                    AllGRLpPhase1InfeaDualValue[AllGRLpPhase1InfeaDualValue.length-1][3] = new double[ApNum*2];
                    for(int i=0; i<4; i++){
                        for(int j=0; j<phase1DualValue[i].length; j++){
                            AllGRLpPhase1InfeaDualValue[AllGRLpPhase1InfeaDualValue.length-1][i][j] = phase1DualValue[i][j];
                        }
                    }

                    //更新 AllyCoverCf、AllyCoverCfPair
                    AllyCoverCf = Arrays.copyOf(AllyCoverCf, AllyCoverCf.length+1);
                    AllyCoverCf[AllyCoverCf.length-1] = new int[yCoverCf.length];
                    for(int i=0; i<yCoverCf.length; i++){
                        AllyCoverCf[AllyCoverCf.length-1][i] = yCoverCf[i];
                    }
                    AllyCoverCfPair = Arrays.copyOf(AllyCoverCfPair, AllyCoverCfPair.length+1);
                    AllyCoverCfPair[AllyCoverCfPair.length-1] = new int[yCoverCfPair.length][2];
                    for(int i=0; i<yCoverCfPair.length; i++){
                        for(int j=0; j<2; j++){
                            AllyCoverCfPair[AllyCoverCfPair.length-1][i][j] = yCoverCfPair[i][j];
                        }
                    }

                    System.out.println("Having used time "+(Totaltimecplex.getCplexTime()-Totalt0));
                    System.out.println("---------------------------------\n");
                    break;
                }

                System.out.println(" ");
                System.out.println("phase1AirpTypeReducedCost >0 Num = "+ phase1AirpTypeReducedcostFlagSum+"/"+2*ApNum+"\n");
            }

            if(GRphase1ContinueFlag==1){
                continue;
            }


            //最优性
            int[][] AirpTypeReducedcostFlag = new int[ApNum][2];
            int AirpTypeReducedcostFlagSum = 0;
            for(int ap=0; ap<ApNum; ap++){
                for(int tp=0; tp<2; tp++){
                    if(AirpTypeFNum[ap][tp]==0) {
                        AirpTypeReducedcostFlag[ap][tp] = 1;
                        AirpTypeReducedcostFlagSum += 1;
                    }
                }
            }

            double[][] GRLpOptDualValue = new double[4][];
            for(int i =0; i<2; i++){
                GRLpOptDualValue[i] = new double[CopyFlightNum];
            }
            GRLpOptDualValue[2] = new double[yCoverCfPair.length];
            GRLpOptDualValue[3] = new double[ApNum*2];


            int GRloop = 0;
            double GRLpValue = 1e6;
            int GRLpValueChongfuNum = 0;
            System.out.println("GRLP Column Generation:");
            while(AirpTypeReducedcostFlagSum<2*ApNum && GRloop<GRloopLim){
                GRloop+=1;
                System.out.println("GRloop" + GRloop);

                double[][] LpOptDualValueLpValue = GR.GRLP(yCoverCf01, yCoverCfPair,
                        CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                        AirpTypeLegnumPattern, AirpTypePatternNum);

                double LpValue = LpOptDualValueLpValue[4][0];
                if(Math.abs(LpValue-GRLpValue)>epson){
                    GRLpValue = LpValue;
                    GRLpValueChongfuNum = 0;
                }else{
                    GRLpValueChongfuNum += 1;
                    if(GRLpValueChongfuNum>=GRLpValueChongfuNumLim){
                        break;
                    }
                }

                double[][] LpOptDualValue = new double[4][];
                for(int i=0; i<4; i++){
                    LpOptDualValue[i] = new double[LpOptDualValueLpValue[i].length];
                    for(int j=0; j<LpOptDualValueLpValue[i].length; j++){
                        LpOptDualValue[i][j] = LpOptDualValueLpValue[i][j];
                    }
                }

                for(int i=0; i<4; i++){
                    for(int j=0; j<LpOptDualValue[i].length; j++){
                        GRLpOptDualValue[i][j] = LpOptDualValue[i][j];
                    }
                }

                System.out.print(2*ApNum + " gates need pricing: ");
                for(int ap=0; ap<ApNum; ap++) {
                    for(int tp = 0; tp < 2; tp++) {
                        System.out.print((2*ap+tp) + " ");
                        if(AirpTypeReducedcostFlag[ap][tp] == 1){
                            continue;
                        }
                        double[] ReducedcostLegnumPattern = GR.OneBestPattern(OneBestPatternTimeLim, LpOptDualValue, yCoverCfPair,
                                ap, tp, -1, -1, GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                                FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);

                        if(ReducedcostLegnumPattern[0]>=-epson){
                            AirpTypeReducedcostFlag[ap][tp] = 1;
                            AirpTypeReducedcostFlagSum += 1;
                            continue;
                        }

                        AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
                        AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = (int) ReducedcostLegnumPattern[1];
                        for(int i=1; i<1+AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0]; i++){
                            AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][i] = (int) ReducedcostLegnumPattern[1+i];
                        }
                        AirpTypePatternNum[ap][tp] += 1;

                        int[][] AB;
                        AB = GR.NegativeReducedCostPattern(NegativeReducedCostPatternTimeLim, LpOptDualValue, yCoverCfPair, ap, tp, 100, -1, -1,
                                GateTypeDivideline, AirpTypeCFID[ap][tp], AirpTypeCFNum[ap][tp],
                                FlightType, FlightDepS, FlightArrS, CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightDepT, CopyFlightArrT);

                        for (int i = 0; i < AB.length; i++) {
                            if (AB[i][0] != 0) {
                                AirpTypeLegnumPattern[ap][tp] = Arrays.copyOf(AirpTypeLegnumPattern[ap][tp],AirpTypePatternNum[ap][tp]+1);
                                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]] = new int[1+MaxPatternLegNum];
                                AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][0] = AB[i][0];
                                for(int j=1; j<1+AB[i][0]; j++){
                                    AirpTypeLegnumPattern[ap][tp][AirpTypePatternNum[ap][tp]][j] = AB[i][j];
                                }
                                AirpTypePatternNum[ap][tp] += 1;
                            } else {
                                break;
                            }
                        }
                    }
                }
                System.out.println(" ");
                System.out.println("AirpTypeReducedCost >0 Num = "+ AirpTypeReducedcostFlagSum+"/"+2*ApNum+"\n");

            }
            System.out.println("GRLP CG use time "+(GRtimecplex.getCplexTime()-GRt0)+"\n");
            GRtimecplex.end();


            //计算GRobj
            IloCplex GRIPcplex = GR.GRIP2(yCoverCf01, yCoverCfPair,
                    CopyFlightOFID, CopyFlightDepS, CopyFlightArrS, CopyFlightType, GateTypeDivideline,
                    AirpTypeLegnumPattern, AirpTypePatternNum);
            GRIPcplex.solve();

            if(GRIPcplex.getStatus().equals(IloCplex.Status.Infeasible)){
                System.out.println("! GRIP is Infeasible");
                AllSAROptSolInfea2GR = Arrays.copyOf(AllSAROptSolInfea2GR, AllSAROptSolInfea2GR.length+1);
                AllSAROptSolInfea2GR[AllSAROptSolInfea2GR.length-1] = AllSAROptSol.length-1;
                System.out.println("LB = "+LB+", UB= "+UB);
                GRIPcplex.end();
            }else{
                AllSAROptSolFea2GR = Arrays.copyOf(AllSAROptSolFea2GR, AllSAROptSolFea2GR.length+1);
                AllSAROptSolFea2GR[AllSAROptSolFea2GR.length-1] = AllSAROptSol.length-1;

                double GRobj = GRIPcplex.getObjValue();
                System.out.println("GRIPobj = "+GRobj);

                //更新AllGROptVal
                AllGRIPOptVal = Arrays.copyOf(AllGRIPOptVal, AllGRIPOptVal.length+1);
                AllGRIPOptVal[AllGRIPOptVal.length-1] = GRobj;

                if((SARIPobj-SARq+GRobj) < UB){
                    UB = SARIPobj-SARq+GRobj;
                    System.out.println("(SARIPobj+GRobj)<UB, update UB = " + UB);
                }
                if((SARIPobj-SARq+GRobj) <= UB && GRobj< UBGRobj){
                    UBGRobj = GRobj;
                }

                System.out.println(" ");
                System.out.println("LB = "+LB+", UB= "+UB);
                System.out.println(" ");
                if((UB-LB)/UB<=epson){
                    System.out.println("\n---------------------------------");
                    System.out.println("LB = "+LB+", UB= "+UB);
                    System.out.println("Satisfy stop criterion and break;\n");
                    GRIPcplex.end();
                    break;
                }
                GRIPcplex.end();
            }


            //Benders Opt Cuts
            AllGRLpOptDualValue = Arrays.copyOf(AllGRLpOptDualValue, AllGRLpOptDualValue.length+1);
            AllGRLpOptDualValue[AllGRLpOptDualValue.length-1] = new double[4][];
            for(int i=0; i<2; i++){
                AllGRLpOptDualValue[AllGRLpOptDualValue.length-1][i] = new double[CopyFlightNum];
            }
            AllGRLpOptDualValue[AllGRLpOptDualValue.length-1][2] = new double[yCoverCfPair.length];
            AllGRLpOptDualValue[AllGRLpOptDualValue.length-1][3] = new double[ApNum*2];
            for(int i=0; i<4; i++){
                for(int j=0; j<GRLpOptDualValue[i].length; j++){
                    AllGRLpOptDualValue[AllGRLpOptDualValue.length-1][i][j] = GRLpOptDualValue[i][j];
                }
            }

            //更新 AllyCoverCf、AllyCoverCfPair
            AllyCoverCf = Arrays.copyOf(AllyCoverCf, AllyCoverCf.length+1);
            AllyCoverCf[AllyCoverCf.length-1] = new int[yCoverCf.length];
            for(int i=0; i<yCoverCf.length; i++){
                AllyCoverCf[AllyCoverCf.length-1][i] = yCoverCf[i];
            }
            AllyCoverCfPair = Arrays.copyOf(AllyCoverCfPair, AllyCoverCfPair.length+1);
            AllyCoverCfPair[AllyCoverCfPair.length-1] = new int[yCoverCfPair.length][2];
            for(int i=0; i<yCoverCfPair.length; i++){
                for(int j=0; j<2; j++){
                    AllyCoverCfPair[AllyCoverCfPair.length-1][i][j] = yCoverCfPair[i][j];
                }
            }

            System.out.println("Having used time "+(Totaltimecplex.getCplexTime()-Totalt0));
            System.out.println("---------------------------------\n");

        }

        System.out.println("\n---------------------------------");
        System.out.println("Stop information: ");
        System.out.println("SAL = "+SAL);
        System.out.println("Stop at Bigloop: "+Bigloop);
        System.out.println("Total use time "+(Totaltimecplex.getCplexTime()-Totalt0));
        Totaltimecplex.end();
        System.out.println("LB = "+LB+", UB= "+UB);
        System.out.println("SARcost = "+(UB-UBGRobj));
        System.out.println("GRgateNum = "+UBGRobj/GateCost);











    }

    public static void main(String[] args) throws IOException, InputDataReader.InputDataReaderException, IloException{
        int DataBase = 6;
        double forcedCoveredPairInitialPatternTimeLim = 4;
        double OneBestPatternTimeLim = 3;
        double NegativeReducedCostPatternTimeLim = 2;
        double TotalTimeLim = 1800;

//        System.out.println("------------");
//        TT(5, 5, 3, 2, 1800);
//        System.out.println("------------\n\n\n");

        for(int i=2; i<=5; i++){
            for(int j=1; j<=3; j++){
                for(int k=1; k<=2; k++){
                    OutConsole.write("D:\\IDEA\\works\\src\\SACG_5minTiaoTime\\5\\database5 1000 "+i+" "+j+" "+k+".txt");
                    System.out.println("------------");
                    TT(5, i, j, k, 1800);
                    System.out.println("------------\n\n\n");
                }
            }
        }

    }
}

