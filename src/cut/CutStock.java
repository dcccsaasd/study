/* --------------------------------------------------------------------------
 * File: CutStock.java
 * Version 12.6.3  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2015. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 */

package cut;

import ilog.concert.*;
import ilog.cplex.*;
import java.io.*;

class CutStock {
   static double RC_EPS = 1.0e-6;
   
   // Data of the problem
   static double   _rollWidth;//木材长度
   static double[] _size;//木材需求尺寸
   static double[] _amount;//需求量
  //函数功能：从dat文件中读取数据
   static void readData(String fileName)// 读取数据
                         throws IOException,
                                InputDataReader.InputDataReaderException {
      //声明对象reader
      InputDataReader reader = new InputDataReader(fileName);
      //获取数据与数据量    
      _rollWidth = reader.readDouble();
      System.out.println(_rollWidth);
      _size      = reader.readDoubleArray();//由定义的readDoubleArray()方法可知，一次的读取从“[”开始，以“]”结束
      _amount    = reader.readDoubleArray();
      for (int i = 0; i <_size.length; i++) {
    	  System.out.print(_size[i]+" ");
	}
      System.out.println();//输出换行
      for (int i = 0; i <_amount.length; i++) {
    	  System.out.print(_amount[i]+" ");
	}
      System.out.println();
   }
   //输出共切了多少根17英尺的木材以及每种切法用了多少木材
   static void report1(IloCplex cutSolver, IloNumVarArray Cut, IloRange[] Fill) 
                         throws IloException {
      System.out.println();
      System.out.println("Using " + cutSolver.getObjValue() + " rolls");//输出共切了多少个17英尺的木材
    
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) {
         System.out.println("  Cut" + j + " = " +
                            cutSolver.getValue(Cut.getElement(j)));//输出每种切法用了多少木材
      }
      System.out.println();
      
      for (int i = 0; i < Fill.length; i++) 
         System.out.println("  Fill" + i + " = " + cutSolver.getDual(Fill[i]));//输出每种木材所需的切法数除以个数
      System.out.println();
   }
 //输出影子价格以及新的切法
   static void report2(IloCplex patSolver, IloNumVar[] Use)
                         throws IloException {
      System.out.println();
      System.out.println("Reduced cost is " + patSolver.getObjValue());//输出影子价格
      
      System.out.println();
      if (patSolver.getObjValue() <= -RC_EPS) {
         for (int i = 0; i < Use.length; i++) 
            System.out.println("  Use" + i + " = "
                               + patSolver.getValue(Use[i]));//输出新的切法
         System.out.println();
      }
   }
 ////输出最优切法所需木材数以及每种切法所需木材数
   static void report3(IloCplex cutSolver, IloNumVarArray Cut) 
                         throws IloException {
      System.out.println();
      System.out.println("Best integer solution uses " + 
                         cutSolver.getObjValue() + " rolls");//输出最优切法所需木材数
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) 
         System.out.println("  Cut" + j + " = " + 
                            cutSolver.getValue(Cut.getElement(j)));//输出每种切法所需木材数
   }
   //统计变量值与变量数量
   static class IloNumVarArray {
      int _num           = 0;
      IloNumVar[] _array = new IloNumVar[32];//建模变量通过定义类型为IloNumVar的变量构造为类IloNumVar的对象,cplex相关包
    //数组不够就增加成两倍长度
      void add(IloNumVar ivar) {
         if ( _num >= _array.length ) {
            IloNumVar[] array = new IloNumVar[2 * _array.length];
            System.arraycopy(_array, 0, array, 0, _num);
            _array = array;
         }
         _array[_num++] = ivar;
      }

      IloNumVar getElement(int i) { return _array[i]; }
      int       getSize()         { return _num; }
   }

   public static void main(String[] args ) {
      String datafile = "data/cutstock.dat";
      try {
         if (args.length > 0)
            datafile = args[0];
         /// CUTTING-OPTIMIZATION PROBLEM ///
         readData(datafile);//读入数据文件    
         IloCplex cutSolver = new IloCplex();//建立Cplex参数
         IloObjective RollsUsed = cutSolver.addMinimize();//最小值问题
         IloRange[]   Fill = new IloRange[_amount.length];//建立cplex需求量约束数组
         //添加约束
         for (int f = 0; f < _amount.length; f++ ) {
        	 /*
        	  * 将每种长度木材需求量的约束条件加入到cplex的约束数组中
        	  * 约束条件为：_amount[f]<=Fill[f]
        	  */
            Fill[f] = cutSolver.addRange(_amount[f], Double.MAX_VALUE);//定义了各个约束的上下界
         }
       //定义IloNumVarArray类型的链表
         IloNumVarArray Cut = new IloNumVarArray();
       
         int nWdth = _size.length;
         //System.out.println("nWdth="+Fill[1]);
         for (int j = 0; j < nWdth; j++) {
        	/* 
        	 * 1.两个column部分创建一个新的变量并加入到函数RollsUsed和Fill[j]中
        	 * 变量在函数中的系数分别是1.0和_rollWidth/_size[j]
        	 * 2.numVar函数部分表示创建一个范围为( 0.0, Double.MAX_VALUE)的变量
        	 * 并将变量加入到column函数中的函数RollsUsed和Fill[j]中
        	 * 3.下式可改写为注释部分两行代码
        	 */
            Cut.add(cutSolver.numVar(cutSolver.column(RollsUsed, 1.0).and(
                                     cutSolver.column(Fill[j],
                                                      (int)(_rollWidth/_size[j]))),
                                     0.0, Double.MAX_VALUE));//每种木材所需的切法数除以个数
            
//       IloColumn col = cutSolver.column(RollsUsed, 1.0).and(cutSolver.column(Fill[j],
//                       (int)(_rollWidth/_size[j])));
//       Cut.add((cutSolver.numVar(col,0.0, Double.MAX_VALUE)));
         }
         //选择控制器
         cutSolver.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
         cutSolver.exportModel("xx.lp");
       
        /// 单纯形法计算///
       /// PATTERN-GENERATION PROBLEM ///
         IloCplex patSolver = new IloCplex();
         IloObjective ReducedCost = patSolver.addMinimize();//影子价格
         /*
          * numVarArray函数定义数组类型变量取值范围，将数字中所有元素的范围都确定
          * 其中nWdth表示数组长度，0表示下界，double.Max_Value表示上界,int表示数据类型
          */
         IloNumVar[] Use = patSolver.numVarArray(nWdth, 
                                                 0., Double.MAX_VALUE, 
                                                 IloNumVarType.Int);
         /*
          * scalProd表示将数组中对应的元素相乘，标量乘法
          * addRange表示添加patSolver数组约束，约束范围是
          * -Double.MAX_VALUE<patSolver.scalProd(_size, Use)<= _rollWidth
          */
         patSolver.addRange(-Double.MAX_VALUE, 
                            patSolver.scalProd(_size, Use),
                            _rollWidth);

         /// 列生成过程 ///
       /// COLUMN-GENERATION PROCEDURE ///
         double[] newPatt = new double[nWdth];
         for (;;) {
            //列生成法计算
            cutSolver.solve();
            //输出共切了多少根17英尺的木材以及每种切法用了多少木材
            report1(cutSolver, Cut, Fill);

          /// 找到以及加入一个新的基变量 ///
            //计算对偶问题的值
            double[] price = cutSolver.getDuals(Fill);
            /*
             * 此表达式计算影子价值(reduce cost)
             * 函数diff表示计算两个数之间的差值
             * setExpr函数表示把diff计算的这个值放入ReduceCost中
             */
            ReducedCost.setExpr(patSolver.diff(1.,
                                               patSolver.scalProd(Use, price)));
          
            patSolver.solve();
          //输出影子价格以及新的切法
            report2 (patSolver, Use);
          
            if ( patSolver.getObjValue() > -RC_EPS )
               break;
          
            newPatt = patSolver.getValues(Use);
          //更新数组
            IloColumn column = cutSolver.column(RollsUsed, 1.);
            for ( int p = 0; p < newPatt.length; p++ )
               column = column.and(cutSolver.column(Fill[p], newPatt[p]));
          //加入新的切割方法
            Cut.add( cutSolver.numVar(column, 0., Double.MAX_VALUE) );
         }
       
         for ( int i = 0; i < Cut.getSize(); i++ ) {
            cutSolver.add(cutSolver.conversion(Cut.getElement(i),
                                               IloNumVarType.Int));
         }
       
         cutSolver.solve();
       //输出最优切法所需木材数以及每种切法所需木材数
         report3 (cutSolver, Cut);
         System.out.println("Solution status: " + cutSolver.getStatus());       
         cutSolver.end();
         patSolver.end();
      }
      catch ( IloException exc ) {
         System.err.println("Concert exception '" + exc + "' caught");
      }
      catch (IOException exc) {
         System.err.println("Error reading file " + datafile + ": " + exc);
      }
      catch (InputDataReader.InputDataReaderException exc ) {
         System.err.println(exc);
      }
   }
}


/* Example Input file:
17
[3,5,9]
[25,20,15]
*/
