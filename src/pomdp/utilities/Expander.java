package pomdp.utilities;

import java.util.LinkedList;
import java.util.Queue;

import pomdp.environments.POMDP;
import pomdp.utilities.distance.L1Distance;

/*
 * 2018-4-21  在PBVI基础上新增
 */
public class Expander {

	private POMDP pomdp = null;

	public Expander(POMDP pomdp) {
		this.pomdp = pomdp;
	}

	public BeliefStateVector<BeliefState> completeLinkCluster(BeliefStateVector<BeliefState> vAllEpsilonBeliefPoints) {

		return null;
	}

	// R-BFS
	public BeliefStateVector<BeliefState> expandRBFS(double epsilon, int max_size, int max_level) {
		// RBFS得到的点集，点集中任意两个信念点距离大于EPSILON
		BeliefStateVector<BeliefState> vAllEpsilonBeliefPoints = new BeliefStateVector<BeliefState>();

		Queue<BeliefState> queue = new LinkedList<BeliefState>();

		int cnt = 0;
		BeliefState initial = pomdp.getBeliefStateFactory().getInitialBeliefState();
		vAllEpsilonBeliefPoints.add(initial);
		queue.offer(initial);
		cnt++;

		int actionCount = pomdp.getActionCount();
		int observationCount = pomdp.getObservationCount();

		// 遍历队列
		while (!queue.isEmpty()) {
			BeliefState bs = queue.poll();
			for (int iAction = 0; iAction < actionCount; ++iAction) {
				for (int iObservation = 0; iObservation < observationCount; ++iObservation) {
					BeliefState next = bs.nextBeliefState(iAction, iObservation);

					if (next != null) {
						cnt++;
						next.setLevel(bs.getLevel() + 1);

						boolean inRange = false;
						L1Distance distancer = new L1Distance();
						for (BeliefState beliefState : vAllEpsilonBeliefPoints) {
							double closureDistance = distancer.distance(beliefState, next);
							if (closureDistance < epsilon) {
								inRange = true;
								break;
							}
						}
						if (!inRange) {
							if (next.getLevel() <= max_level) {
								queue.offer(next);
								vAllEpsilonBeliefPoints.add(next);
								if (vAllEpsilonBeliefPoints.size() >= max_size) {
									// 第一种情况
									System.out.println("达到了最大闭包数，当前next所在层数为：" + next.getLevel());
									System.out.println("总数 = " + cnt);
									return vAllEpsilonBeliefPoints;
								}
							} else {
								System.out.println("达到了" + max_level + "层，闭包的数目：" + vAllEpsilonBeliefPoints.size());
								System.out.println("总数 = " + cnt);
								return vAllEpsilonBeliefPoints;
							}
						}
					}
				}
			}
		}
		// 第三种情况
		System.out.println("点集探索完成, 总闭包的数目为：" + vAllEpsilonBeliefPoints.size());
		System.out.println("总数 = " + cnt);
		return vAllEpsilonBeliefPoints;
	}

	public BeliefStateVector<BeliefState> expand(double epsilon, int max_size, int max_level) {
		BeliefStateVector<BeliefState> vBeliefPoints = new BeliefStateVector<BeliefState>();

		// 队列用于层次遍历信念树的时候，不遗漏每个点的后继结点
		Queue<BeliefState> queue = new LinkedList<BeliefState>();
		int ANum = 0; // 在闭包内数量
		int BNum = 0; // 在一倍到两倍之间数量
		int CNum = 0; // 在闭包外数量
		int cnt = 0; // 探索到的点总数

		// 初始结点先加入
		BeliefState initial = pomdp.getBeliefStateFactory().getInitialBeliefState();
		vBeliefPoints.add(initial);
		queue.offer(initial);
		CNum++;
		cnt++;

		int actionCount = pomdp.getActionCount();
		int observationCount = pomdp.getObservationCount();

		// 遍历队列
		while (!queue.isEmpty()) {
			BeliefState bs = queue.poll();
			for (int iAction = 0; iAction < actionCount; ++iAction) {
				for (int iObservation = 0; iObservation < observationCount; ++iObservation) {
					BeliefState next = bs.nextBeliefState(iAction, iObservation);

					// 后继结点要存在且不是已知闭包中心点
					if (next != null && !vBeliefPoints.contains(next)) {
						cnt++;
						next.setLevel(bs.getLevel() + 1); // 设置层数

						// 判断next点和任一点的距离，不过要注意避免闭包重叠
						boolean inRange = false;
						L1Distance distancer = new L1Distance();
						for (BeliefState beliefState : vBeliefPoints) {
							double closureDistance = distancer.distance(beliefState, next);
							if (closureDistance < epsilon) {
								// 距离小于epsilon，则在闭包内
								inRange = true;
								ANum++;
								break;
							} else if (closureDistance >= epsilon && closureDistance < 2 * epsilon) {
								// 距离在epsilon和两倍epsilon之间，暂时先无视
								inRange = true;
								BNum++;
								break;
							} else if (closureDistance >= 2 * epsilon) {
								// 距离大于所有结点两倍epsilon，则生成新的闭包
							}
						}
						if (!inRange) {
							CNum++;
							if (next.getLevel() <= max_level) {
								queue.offer(next); // 在闭包内不考虑它的后继
								vBeliefPoints.add(next);
								// 调试用的输出语句
								// System.out.println(vBeliefPoints.size() + " " + next.getLevel());
								if (vBeliefPoints.size() >= max_size) {
									// 第一种情况
									System.out.println("达到了最大闭包数，当前next所在层数为：" + next.getLevel());
									System.out.println("总数 = " + cnt + ", 在闭包内数量 = " + ANum + ", 在一倍到两倍之间数量 = " + BNum
											+ ", 在闭包外数量 = " + CNum);
									return vBeliefPoints;
								}
							} else {
								// 第二种情况
								System.out.println("达到了" + max_level + "层，闭包的数目：" + vBeliefPoints.size());
								System.out.println("总数 = " + cnt + ", 在闭包内数量 = " + ANum + ", 在一倍到两倍之间数量 = " + BNum
										+ ", 在闭包外数量 = " + CNum);
								return vBeliefPoints;
							}
						}
					}
				}
			}
		}
		// 第三种情况
		System.out.println("点集探索完成, 总闭包的数目为：" + vBeliefPoints.size());
		System.out.println("总数 = " + cnt + ", 在闭包内数量 = " + ANum + ", 在一倍到两倍之间数量 = " + BNum + ", 在闭包外数量 = " + CNum);
		return vBeliefPoints;
	}

}