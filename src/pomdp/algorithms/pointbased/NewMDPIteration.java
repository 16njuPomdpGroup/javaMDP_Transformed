package pomdp.algorithms.pointbased;

import java.util.Iterator;

import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateVector;
import pomdp.utilities.Expander;
import pomdp.utilities.SparseTabularFunction;
import pomdp.utilities.datastructures.Function;
import pomdp.utilities.distance.L1Distance;

public class NewMDPIteration extends ValueIteration {

	protected Iterator<BeliefState> m_itCurrentIterationPoints;
	protected boolean m_bSingleValueFunction = true;
	protected boolean m_bRandomizedActions;
	protected double m_dFilteredADR = 0.0;

	public BeliefStateVector<BeliefState> vBeliefPoints; // 闭包代表点集合
	public int maxAction[]; // 保存代表点所对应的动作

	protected Function m_fTransition;
	protected Function m_fReward;

	protected int m_cStates;
	protected int m_cActions;

	public static final double GAMA = 0.95; // 折扣因子
	public static final double EPSILON = 0.5; // 闭包半径
	public static final int MAX_SIZE = 80; // 最大闭包个数
	public static final int MAX_LEVEL = 10; // 探索点的最深层数

	/**
	 * 两个构造函数
	 */
	public NewMDPIteration(POMDP pomdp) {
		super(pomdp);

		m_fTransition = null;
		m_fReward = null;
		m_cStates = 0;
		m_cActions = 0;
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = true;
	}

	public NewMDPIteration(POMDP pomdp, boolean bRandomizedActionExpansion) {
		super(pomdp);

		m_fTransition = null;
		m_fReward = null;
		m_cStates = 0;
		m_cActions = 0;
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = bRandomizedActionExpansion;
	}

	@Override
	public int getAction(BeliefState bsCurrent) {
		return m_vValueFunction.getBestAction(bsCurrent);
	}

	/**
	 * 执行算法的函数
	 */
	public void newIteration(POMDP pomdp) {
		// 点集扩张，生成闭包
		Expander expander = new Expander(pomdp);
		// vBeliefPoints = expander.expand(EPSILON, MAX_SIZE, MAX_LEVEL);
		vBeliefPoints = expander.expandRBFS(EPSILON, MAX_SIZE, MAX_LEVEL);

		proveClosure();

		m_cStates = vBeliefPoints.size();
		m_cActions = pomdp.getActionCount();
		// 初始化Function类
		initDynamicsFunctions();

		// 生成闭包间的转移概率，并存入m_fTransition中
		// 生成动作的立即回报值，并存入m_fReward中
		setTransitionsAndRewards(pomdp);

		// MDP求解
		MDPSolver(pomdp);

		for (int i = 0; i < vBeliefPoints.size(); i++) {
			System.out.println("i = " + i + ", action = " + maxAction[i]);
		}

		// 计算ADR，在某个闭包内的信念点，动作就取闭包中心点的动作
		testADR(pomdp);

	}

	/**
	 * 验证获得的闭包是否有重叠
	 */
	private void proveClosure() {
		L1Distance distancer = new L1Distance();
		for (BeliefState bs1 : vBeliefPoints) {
			for (BeliefState bs2 : vBeliefPoints) {
				if (vBeliefPoints.indexOf(bs1) == vBeliefPoints.indexOf(bs2)) {
					continue;
				}
				if (distancer.distance(bs1, bs2) < EPSILON) {
					System.out.println("闭包相交！");
					return;
				}
			}
		}
		return;
	}

	/**
	 * 计算ADR
	 */
	private void testADR(POMDP pomdp) {
		System.out.println("ADR = " + pomdp.computeAverageDiscountedReward(500, 100, true, this));
	}

	/**
	 * 根据所给信念点，寻找所有闭包，在闭包半径内返回中心点对应的动作
	 */
	public int getBestAction(BeliefState bs) {
		L1Distance distancer = new L1Distance();

		for (int i = 0; i < m_cStates; i++) {
			if (distancer.distance(bs, vBeliefPoints.get(i)) <= EPSILON) {
				return maxAction[i];
			}
		}
		return 0;
	}

	/**
	 * MDP求解算法
	 */
	private void MDPSolver(POMDP pomdp) {
		int cStates = vBeliefPoints.size();
		int cActions = m_cActions;
		double gama = GAMA;
		double delta = 0.0;
		double epsilon = 0.001; // epsilon为任何状态的值函数最大误差，设置为ADR的千分之一或百分之一
		// 值函数的数组
		double[] utility = new double[cStates]; // U
		double[] utilityTemp = new double[cStates]; // U'
		for (int i = 0; i < cStates; i++) {
			utilityTemp[i] = 0.0;
		}
		maxAction = new int[cStates];
		for (int i = 0; i < cStates; i++) {
			maxAction[i] = 0;
		}

		// 验证delta(s')是否为1
		// for (int i = 0; i < cStates; i++) {
		// for (int j = 0; j < cActions; j++) {
		// double sum = 0.0;
		// for (int k = 0; k < cStates; k++) {
		// if (i == k)
		// continue;
		// sum += m_fTransition.valueAt(i, j, k);
		// if (sum > 0.0)
		// System.out.println("sum = " + sum);
		// }
		// }
		// }

		do {
			for (int i = 0; i < cStates; i++) {
				utility[i] = utilityTemp[i];
			}
			delta = 0.0;
			// 遍历所有s
			for (int i = 0; i < cStates; i++) {
				double maxReward = 0.0;
				// 寻找能使值函数最大的动作a
				for (int j = 0; j < cActions; j++) {
					double transitionReward = 0.0;
					// 累加期望和，i是s，j是a，k是s'
					for (int k = 0; k < cStates; k++) {
						transitionReward = transitionReward + m_fTransition.valueAt(i, j, k) * utility[k];
						// if(m_fTransition.valueAt(i, j, k) > 0.0) System.out.println("Transition = " +
						// m_fTransition.valueAt(i, j, k));
					}
					transitionReward = transitionReward * gama;
					transitionReward = transitionReward + m_fReward.valueAt(i, j);
					// if(m_fReward.valueAt(i, j) > 0.0) System.out.println("Reward = " +
					// m_fReward.valueAt(i, j));
					// System.out.println("transitionReward = " + transitionReward);
					if (transitionReward > maxReward) {
						maxAction[i] = j; // 更替最佳动作
						maxReward = transitionReward;
					}
				}
				utilityTemp[i] = maxReward;
				// 赋值给delta
				delta = Math.max(delta, Math.abs(utilityTemp[i] - utility[i]));

				System.out.println("delta = " + delta);
			}
		} while (delta >= epsilon * (1 - gama) * gama);
		for (int i = 0; i < cStates; i++) {
			System.out.print("utility[" + i + "] = " + utility[i] + " ");
		}
		System.out.println();
	}

	/**
	 * 生成闭包间的转移概率，并存入m_fTransition中 生成动作的立即回报值，并存入m_fReward中
	 */
	private void setTransitionsAndRewards(POMDP pomdp) {
		int actionCount = pomdp.getActionCount();
		int observationCount = pomdp.getObservationCount();
		L1Distance distancer = new L1Distance();
		BeliefState beginNext = null;

		System.out.println("actionCount = " + actionCount + ", observationCount = " + observationCount);

		for (BeliefState begin : vBeliefPoints) {
			for (BeliefState end : vBeliefPoints) {
				// 起始闭包和终止闭包不能是同一个闭包
				if (vBeliefPoints.indexOf(begin) == vBeliefPoints.indexOf(end)) {
					continue;
				}
				for (int iAction = 0; iAction < actionCount; ++iAction) {
					// 相同action的转移概率进行叠加
					double transitionSumGivenA = 0.0;
					// 回报值叠加
					double rewardSumGivenA = 0.0;
					for (int iObservation = 0; iObservation < observationCount; ++iObservation) {
						beginNext = begin.nextBeliefState(iAction, iObservation);
						// 注意beginNext有可能是null，所以要增加一个判断
						if (beginNext != null && distancer.distance(beginNext, end) <= EPSILON) {
							transitionSumGivenA += begin.probabilityOGivenA(iAction, iObservation);
							rewardSumGivenA += pomdp.immediateReward(begin, iAction);
						}
					}
					if (transitionSumGivenA > 0.0) {
						// System.out.println("transitionSumGivenA = " + transitionSumGivenA);
						// 设置T(s,a,s')
						m_fTransition.setValue(vBeliefPoints.indexOf(begin), iAction, vBeliefPoints.indexOf(end),
								transitionSumGivenA);
						// 设置R(s,a)
						if (rewardSumGivenA > 0.0) {
							// System.out.println("rewardSumGivenA = " + rewardSumGivenA);
							m_fReward.setValue(vBeliefPoints.indexOf(begin), iAction, rewardSumGivenA);
						}
					}
				}
			}
		}
	}

	/**
	 * 2018-4-29 调试用函数，输出信念点的概率分布
	 */
	@SuppressWarnings("unused")
	private void outputBsPro(BeliefState bs) {
		int nonZeroStates = (int) bs.size();
		System.out.println("此点的非零概率分布为：(共有 " + nonZeroStates + " 个)");
		for (int i = 0; i < nonZeroStates; i++) {
			System.out.print(bs.valueAt(i) + " ");
		}
		System.out.println();
	}

	/**
	 * 2018-4-29 调试用函数，输出信念点向量中下标为index的信念点的概率分布
	 */
	@SuppressWarnings("unused")
	private void outputBsProIndex(int index) {
		int nonZeroStates = (int) vBeliefPoints.get(index).size();
		System.out.println("下标为 " + index + " 的点的非零概率分布为：(共有 " + nonZeroStates + " 个)");
		for (int i = 0; i < nonZeroStates; i++) {
			System.out.print(vBeliefPoints.get(index).valueAt(i) + " ");
		}
		System.out.println();
	}

	/**
	 * 根据状态、动作、观察值的数量初始化Function类，主要是初始化Function类里面的数据结构的长度
	 */
	private void initDynamicsFunctions() {
		int[] aDims = new int[3];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		aDims[2] = m_cStates;
		// 实例化转移函数，三维
		m_fTransition = new SparseTabularFunction(aDims);

		aDims = new int[2];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		// 实例化值函数，二维
		m_fReward = new SparseTabularFunction(aDims);
	}
}
