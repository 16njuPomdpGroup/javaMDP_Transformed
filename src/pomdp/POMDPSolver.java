package pomdp;

import pomdp.algorithms.pointbased.NewMDPIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.Logger;

public class POMDPSolver {

	public static void main(String[] args) {
		String sPath = "Models/"; // model路径
		String sModelName = "hallway"; // model名
		// String sModelName = "tagAvoid";
		String sMethodName = "MDPT"; // 算法名
		Logger.getInstance().setOutput(true); // 允许输出
		Logger.getInstance().setSilent(false); // 允许输出到控制台
		try {
			String sOutputDir = "logs/POMDPSolver"; // 输出路径
			String sFileName = sModelName + "_" + sMethodName + ".txt"; // 输出文件名
			Logger.getInstance().setOutputStream(sOutputDir, sFileName);
		} catch (Exception e) {
			System.err.println(e);
		}

		POMDP pomdp = null;
		try {
			pomdp = new POMDP();
			pomdp.load(sPath + sModelName + ".POMDP"); // 载入POMDP模型

		} catch (Exception e) {
			Logger.getInstance().logln(e);
			e.printStackTrace();
			System.exit(0);
		}

		try {
			NewMDPIteration iteration = new NewMDPIteration(pomdp);
			iteration.newIteration(pomdp); // 执行算法

		} catch (Exception e) {
			Logger.getInstance().logln(e);
			e.printStackTrace();
			System.exit(0);
		}
	}
}
