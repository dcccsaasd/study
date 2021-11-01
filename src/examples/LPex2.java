package examples;/* --------------------------------------------------------------------------
 * File: LPex2.java
 * Version 12.9.0
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2019. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 ***********************从事先建立好的模型文件中导入模型****************************
 * (1)ROWS: 每一行，包括目标函数与约束条件
 * (1 N 表示自由行， obj是对目标函数的命名，可以任意取
 * (2 L 表示该行小于等于， c1是对改行的命名，可以任意取名
 * (3 G 表示该行大于等于
 * (4 E 表示该行等于
 * (2)其中 MARK0000 ‘MARKER’ ‘INTORG’ MARK0001 ‘MARKER’ ‘INTEND’ 分别表示整数变量的起止
 * (3)Bounds: 表示各变量的上界或下界
 * (1 LO 表示下界
 * (2 UP 表示上界
 * (3 FX 表示该变量固定值
 * (4 FR 表示改变量的范围为(?∞,∞)
 * (5 MI 表示下界为负无穷
 * (6 PL 表示上界为正无穷
 * (7 MPS 变量默认的范围为[0,∞)
 * LPex2.java - Reading in and optimizing an LP problem
 *
 *
 * To run this example, command line arguments are required.
 * i.e.,   java LPex2   filename   method
 * where
 *     filename is the name of the file, with .mps, .lp, or .sav extension
 *     method   is the optimization method
 *                 o          default
 *                 p          primal simplex
 *                 d          dual   simplex
 *                 h          barrier with crossover
 *                 b          barrier without crossover
 *                 n          network with dual simplex cleanup
 *                 s          sifting
 *                 c          concurrent optimization
 * Example:
 *     java LPex2  example.mps  o
 */

import ilog.concert.*;
import ilog.cplex.*;


public class LPex2 {
   static void usage() {
      System.out.println("usage:  LPex2 <filename> <method>");
      System.out.println("          o       default");
      System.out.println("          p       primal simplex");
      System.out.println("          d       dual   simplex");
      System.out.println("          h       barrier with crossover");//有交叉的内点法
      System.out.println("          b       barrier without crossover");//无交叉的内点法
      System.out.println("          n       network with dual simplex cleanup");
      System.out.println("          s       sifting");
      System.out.println("          c       concurrent optimization");//并行
   }

   public static void main(String[] args) {
      if ( args.length != 2 ) {
         usage();
         return;
      }
      try {
         // Create the modeler/solver object
         IloCplex cplex = new IloCplex();

         // Evaluate command line option and set optimization method accordingly.
         switch ( args[1].charAt(0) ) {
         case 'o': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Auto);
                   break;
         case 'p': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Primal);
                   break;
         case 'd': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Dual);
                   break;
         case 'h': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Barrier);
                   break;
         case 'b': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Barrier);
                   cplex.setParam(IloCplex.Param.Barrier.Crossover,
                                  IloCplex.Algorithm.None);
                   break;
         case 'n': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Network);
                   break;
         case 's': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Sifting);
                   break;
         case 'c': cplex.setParam(IloCplex.Param.RootAlgorithm,
                                  IloCplex.Algorithm.Concurrent);
                   break;
         default:  usage();
                   return;
         }

         // Read model from file with name args[0] into cplex optimizer object
         cplex.importModel(args[0]);

         cplex.exportModel("lpex2.lp");
         // Solve the model and display the solution if one was found
         if ( cplex.solve() ) {
            System.out.println("Solution status = " + cplex.getStatus());
            System.out.println("Solution value  = " + cplex.getObjValue());

            // Access the IloLPMatrix object that has been read from a file in
            // order to access variables which are the columns of the LP.  The
            // method importModel() guarantees that exactly one IloLPMatrix
            // object will exist, which is why no tests or iterators are
            // needed in the following line of code.
            IloLPMatrix lp = (IloLPMatrix)cplex.LPMatrixIterator().next();

            double[] x = cplex.getValues(lp);
            for (int j = 0; j < x.length; ++j)
               System.out.println("Variable " + j + ": Value = " + x[j]);

            try {     // basis may not exist
               IloCplex.BasisStatus[] cstat =
                     cplex.getBasisStatuses(lp.getNumVars());
               for (int j = 0; j < x.length; ++j)
                  System.out.println("Variable " + j +
                                     ": Basis Status = " + cstat[j]);
            }
            catch (Exception e) {
            }

            System.out.println("Maximum bound violation = " +
                   cplex.getQuality(IloCplex.QualityType.MaxPrimalInfeas));
         }
         cplex.end();
      }
      catch (IloException e) {
         System.err.println("Concert exception caught: " + e);
      }
   }
}
