package pomdp.environments;

import java.io.IOException;
import java.io.Serializable;
import java.util.AbstractCollection;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.pointbased.NewMDPIteration;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.Logger;
import pomdp.utilities.POMDPLoader;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.SparseTabularFunction;
import pomdp.utilities.TabularAlphaVector;
import pomdp.utilities.datastructures.Function;
import pomdp.utilities.datastructures.TabularFunction;
import pomdp.valuefunction.MDPValueFunction;

public class POMDP implements Serializable {

	protected final static double MAX_INF = Double.POSITIVE_INFINITY;
	protected final static double MIN_INF = Double.NEGATIVE_INFINITY;

	private static final long serialVersionUID = -231630700034970161L;
	protected Function m_fTransition;
	protected Function m_fReward;
	protected Function m_fObservation;
	protected Function m_fStartState;
	// S
	protected Vector<String> m_vStateNames;
	protected Map<String, Integer> m_mStates;
	// A
	protected Map<String, Integer> m_mActionIndexes;
	protected Vector<String> m_vActionNames;
	// O
	protected Map<String, Integer> m_mObservations;
	protected int m_cStates;
	protected int m_cActions;
	protected int m_cObservations;
	protected double m_dGamma;

	protected static int g_sMaxTabularSize = 3000;
	protected Vector<Integer> m_vTerminalStates;
	protected Vector<Integer> m_vObservationStates;
	protected double[][] m_adStoredRewards;
	protected double[] m_adMinActionRewards;

	protected RandomGenerator m_rndGenerator;
	protected long m_iRandomSeed;
	protected String m_sName;
	protected RewardType m_rtReward;
	protected BeliefStateFactory m_bsFactory;
	protected MDPValueFunction m_vfMDP;
	protected double m_dMinReward;

	public enum RewardType {
		StateActionState, ActionEndState, StateAction, State;
	}

	public POMDP() {
		m_fTransition = null;
		m_fReward = null;
		m_fObservation = null;
		m_mActionIndexes = null;
		m_vActionNames = null;
		m_mStates = null;
		m_mObservations = null;
		m_fStartState = null;
		m_cStates = 0;
		m_cActions = 0;
		m_cObservations = 0;
		m_dGamma = 0.95;
		m_vTerminalStates = null;
		m_vObservationStates = new Vector<Integer>();
		m_adMinActionRewards = null;
		m_iRandomSeed = 0;
		m_rndGenerator = new RandomGenerator("POMDP");
		m_sName = "";
		m_rtReward = RewardType.StateAction;

		m_bsFactory = null;
		m_vfMDP = null;
		m_dMinReward = 0.0; // Double.POSITIVE_INFINITY;
	}

	public Function getM_FReward() {
		return m_fReward;
	}

	public void load(String sFileName) throws IOException,
			InvalidModelFileFormatException {
		m_sName = sFileName.substring(sFileName.lastIndexOf("/") + 1,
				sFileName.lastIndexOf("."));
		POMDPLoader p = new POMDPLoader(this);
		p.load(sFileName);
		if (m_rtReward == RewardType.StateActionState)
			initStoredRewards();

		initBeliefStateFactory();
		m_vfMDP = new MDPValueFunction(this, 0.0);

		Logger.getInstance().logln();
	}

	public BeliefStateFactory getBeliefStateFactory() {
		return m_bsFactory;
	}

	public MDPValueFunction getMDPValueFunction() {
		if (m_vfMDP == null) {
			m_vfMDP = new MDPValueFunction(this, 0.0);
		}
		return m_vfMDP;
	}

	public void resetMDPValueFunction() {
		m_vfMDP = new MDPValueFunction(this, 0.0);
	}

	protected void initStoredRewards() {
		m_adStoredRewards = new double[m_cStates][m_cActions];
		int iState = 0, iAction = 0;
		for (iState = 0; iState < m_cStates; iState++) {
			for (iAction = 0; iAction < m_cActions; iAction++) {
				m_adStoredRewards[iState][iAction] = MIN_INF;
			}
		}
	}

	/**
	 * T Function
	 * 
	 * @param iState1
	 * @param iAction
	 * @param iState2
	 * @return
	 */
	public double tr(int iState1, int iAction, int iState2) {
		return m_fTransition.valueAt(iState1, iAction, iState2);
	}

	/**
	 * Immediate reward function of the POMDP. Checks reward type
	 * (R(s,a,s'),R(s,a) or R(s)) before accessing the reward function.
	 * 
	 * @param iStartState
	 * @param iAction
	 * @param iEndState
	 * @return immediate reward
	 */
	public double R(int iStartState, int iAction, int iEndState) {
		double dReward = 0.0;
		if (m_rtReward == RewardType.StateActionState)
			dReward = m_fReward.valueAt(iStartState, iAction, iEndState);
		else if (m_rtReward == RewardType.StateAction)
			dReward = m_fReward.valueAt(iStartState, iAction);
		else if (m_rtReward == RewardType.ActionEndState)
			dReward = m_fReward.valueAt(iAction, iEndState);
		else if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		return dReward;
	}

	/**
	 * Immediate reward function of the POMDP.
	 * 
	 * @param iStartState
	 * @return immediate reward
	 */
	public double R(int iStartState) {
		double dReward = 0.0;
		if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		return dReward;
	}

	/**
	 * Immediate reward function of the POMDP of the form R(s,a). If reward
	 * structure is of the form R(s,a,s') it sums over all possible s'.
	 * 
	 * @param iStartState
	 * @param iAction
	 * @return immediate reward
	 */
	public double R(int iStartState, int iAction) {
		int iEndState = 0;
		double dReward = 0.0, dSumReward = 0.0;
		double dTr = 0.0;
		Iterator<Entry<Integer, Double>> itNonZeroEntries = null;
		Entry<Integer, Double> e = null;

		if (m_rtReward == RewardType.StateAction)
			dReward = m_fReward.valueAt(iStartState, iAction);
		else if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		else if (m_rtReward == RewardType.StateActionState) {

			dReward = m_adStoredRewards[iStartState][iAction];
			if (dReward == MIN_INF) {

				dSumReward = m_fReward.valueAt(iStartState, iAction);
				if (dSumReward == 0) {
					itNonZeroEntries = m_fReward.getNonZeroEntries(iStartState,
							iAction);
					if (itNonZeroEntries != null) {
						while (itNonZeroEntries.hasNext()) {
							e = itNonZeroEntries.next();
							iEndState = ((Number) e.getKey()).intValue();
							dReward = ((Number) e.getValue()).doubleValue();
							dTr = tr(iStartState, iAction, iEndState);
							if (dTr > 0)
								dSumReward += dReward * dTr;
						}
					}
				}

				m_adStoredRewards[iStartState][iAction] = dSumReward;

				dReward = dSumReward;
			}

		}

		return dReward;
	}

	public double O(int iAction, int iEndState, int iObservation) {
		return m_fObservation.valueAt(iAction, iEndState, iObservation);
	}

	public String getActionName(int iAction) {
		if (m_vActionNames != null) {
			return (m_vActionNames.get(new Integer(iAction))).toString();
		}
		return "" + iAction;
	}

	public int getActionIndex(String sAction) {
		if (m_mActionIndexes != null) {
			Object oIdx = m_mActionIndexes.get(sAction);
			if (oIdx != null) {
				return ((Integer) oIdx).intValue();
			}
		}
		try {
			return Integer.parseInt(sAction);
		} catch (NumberFormatException e) {
			return -1;
		}
	}

	public String getObservationName(int iObservation) {
		return iObservation + "";
	}

	public String getStateName(int iState) {
		if (m_vStateNames != null)
			return (String) m_vStateNames.elementAt(iState);
		return iState + "";
	}

	public int getStateIndex(String sState) {
		Object oIdx = null;
		if (m_mStates != null)
			oIdx = m_mStates.get(sState);
		if (oIdx != null) {
			return ((Integer) oIdx).intValue();
		} else {
			try {
				return Integer.parseInt(sState);
			} catch (NumberFormatException e) {
				return -1;
			}
		}
	}

	public int getObservationIndex(String sObservation) {
		Object oIdx = null;
		if (m_mObservations != null)
			oIdx = m_mObservations.get(sObservation);
		if (oIdx != null) {
			return ((Integer) oIdx).intValue();
		} else {
			try {
				return Integer.parseInt(sObservation);
			} catch (NumberFormatException e) {
				return -1;
			}
		}
	}

	public void setTransition(int iStartState, int iAction, int iEndState,
			double dTr) {
		m_fTransition.setValue(iStartState, iAction, iEndState, dTr);
	}

	public void setObservation(int iAction, int iEndState, int iObservation,
			double dValue) {
		m_fObservation.setAllValues(iAction, iEndState, iObservation, dValue);
	}

	public void setDiscountFactor(double dGamma) {
		m_dGamma = dGamma;
	}

	public void setStateCount(int cStates) {
		m_cStates = cStates;
	}

	public void addState(String sState) {
		if (m_mStates == null) {
			m_mStates = new TreeMap<String, Integer>();
			m_vStateNames = new Vector<String>();
		}

		m_mStates.put(sState, m_cStates);
		m_vStateNames.add(sState);
		m_cStates++;
	}

	public void setActionCount(int cActions) {
		m_cActions = cActions;
	}

	public void addAction(String sAction) {
		if (m_mActionIndexes == null) {
			m_mActionIndexes = new TreeMap<String, Integer>();
			m_vActionNames = new Vector<String>();
		}

		m_mActionIndexes.put(sAction, m_cActions);
		m_vActionNames.add(sAction);
		m_cActions++;
	}

	public void setObservationCount(int cObservations) {
		m_cObservations = cObservations;
	}

	public void addObservation(String sObservation) {
		if (m_mObservations == null) {
			m_mObservations = new TreeMap<String, Integer>();
		}

		m_mObservations.put(sObservation, m_cObservations);
		m_cObservations++;
	}

	/**
	 * 
	 * @param iAction
	 * @param iState
	 * @return
	 */
	public int execute(int iAction, int iState) {
		int iNextState = -1;
		double dProb = m_rndGenerator.nextDouble();
		double dTr = 0.0;
		Iterator<Entry<Integer, Double>> itNonZero = getNonZeroTransitions(
				iState, iAction);
		Entry<Integer, Double> e = null;
		while (dProb > 0) {
			e = itNonZero.next();
			iNextState = e.getKey();
			dTr = e.getValue();
			dProb -= dTr;
		}
		return iNextState;
	}

	/**
	 * 
	 * @param iAction
	 * @param iState
	 * @return
	 */
	public int observe(int iAction, int iState) {
		int iObservation = -1;
		double dProb = m_rndGenerator.nextDouble(), dO = 0.0;
		Iterator<Entry<Integer, Double>> itNonZeroObservations = m_fObservation
				.getNonZeroEntries(iAction, iState);
		Entry<Integer, Double> e = null;
		while (dProb > 0) {
			e = itNonZeroObservations.next();
			iObservation = e.getKey();
			dO = e.getValue();
			dProb -= dO;
		}
		if (iObservation == m_cObservations)
			throw new Error("Corrupted observation function - O( "
					+ getActionName(iAction) + ", " + getStateName(iState)
					+ ", * ) = 0");
		return iObservation;
	}

	protected double round(double d, int cDigits) {
		double dPower = Math.pow(10, cDigits);
		double d1 = Math.round(d1 = d * dPower);
		return d1 / dPower;
	}

	public double computeAverageDiscountedReward(int cTests,
			int cMaxStepsToGoal, boolean bOutputMessages, NewMDPIteration newMDP) {
		double dSumDiscountedRewards = 0.0, dDiscountedReward = 0.0, dSumSquares = 0.0;
		int iTest = 0; 
		RandomGenerator rndGenerator = m_rndGenerator;
		int[] aiActionCount = new int[m_cActions];
		double dStdev = 10000.0, dStandardError = 10.0, dADR = 0.0;

		boolean bCacheBeliefStates = getBeliefStateFactory().cacheBeliefStates(
				false);

		//m_cSteps = 0;
		for (iTest = 0; (iTest < cTests) && (dStandardError > 0.01 * dADR); iTest++) {
			dDiscountedReward = computeDiscountedRewardII(cMaxStepsToGoal, aiActionCount, newMDP);
			dSumSquares += (dDiscountedReward * dDiscountedReward);
			dSumDiscountedRewards += dDiscountedReward;

			if (iTest >= 50 && iTest % 10 == 0) {
				dADR = dSumDiscountedRewards / iTest;
				dStdev = Math.sqrt((dSumSquares - (iTest + 1) * dADR * dADR)
						/ iTest);
				if (!Double.isNaN(dStdev))
					dStandardError = 2.0 * dStdev / Math.sqrt(iTest + 1);
			}
		}

		if (bOutputMessages) {
			dADR = dSumDiscountedRewards / iTest;
			dStdev = Math.sqrt(Math.abs((dSumSquares - (iTest + 1) * dADR
					* dADR)
					/ iTest));
			dStandardError = 2.0 * dStdev / Math.sqrt(iTest);
			Logger.getInstance().log(
					"POMDP",
					0,
					"computeAverageDiscountedReward",
					"After " + iTest + " tests. ADR " + round(dADR, 3)
							+ ", stdev " + round(dStdev, 5));
			Logger.getInstance().logln();
		}

		m_rndGenerator = rndGenerator;

		getBeliefStateFactory().cacheBeliefStates(bCacheBeliefStates);

		return dSumDiscountedRewards / iTest;
	}

	/**
	 * 2018-5-13 更改
	 * 不用policy.getAcion获取动作
	 * 
	 * @param cMaxStepsToGoal
	 * @param policy
	 * @param vObservedBeliefPoints
	 * @param bExplore
	 * @param aiActionCount
	 * @return
	 */
	public double computeDiscountedRewardII(int cMaxStepsToGoal, int[] aiActionCount, NewMDPIteration newMDP) {
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0;

		int iStep = 0, iAction = 0, iObservation = 0;

		int iState = chooseStartState(), iNextState = 0;
		BeliefState bsCurrentBelief = getBeliefStateFactory()
				.getInitialBeliefState(), bsNext = null;

		boolean bDone = false;
		int cSameStates = 0;

		for (iStep = 0; (iStep < cMaxStepsToGoal) && !bDone; iStep++) {

			// 2018-5-13
			iAction = newMDP.getBestAction(bsCurrentBelief);

			// iAction = m_rndGenerator.nextInt(m_cActions);

			if (iAction == -1)
				return Double.NEGATIVE_INFINITY;

			if (aiActionCount != null)
				aiActionCount[iAction]++;

			iNextState = execute(iAction, iState);
			iObservation = observe(iAction, iNextState);

			if (m_rtReward == RewardType.StateAction)
				dCurrentReward = R(iState, iAction); // R(s,a)
			else if (m_rtReward == RewardType.StateActionState)
				dCurrentReward = R(iState, iAction, iNextState); // R(s,a,s')
			else if (m_rtReward == RewardType.State)
				dCurrentReward = R(iState);
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;

			bDone = endADR(iNextState);
			if (bDone)
				dDiscountFactor = 0.0;

			bsNext = bsCurrentBelief.nextBeliefState(iAction, iObservation);

			if (iState != iNextState)
				cSameStates = 0;
			else
				cSameStates++;
			if (bsNext == null || (bsNext.valueAt(iNextState) == 0)
					|| (cSameStates > 10)) {
				bDone = true;
			}

			iState = iNextState;
			bsCurrentBelief.release();
			bsCurrentBelief = bsNext;
		}
		return dDiscountedReward;// + m_dMinReward * ( 1 / ( 1 - dDiscountFactor
									// ) );
	}

	/**
	 * ADR: average discount reward
	 * 
	 * @param iState
	 * @param dReward
	 * @return
	 */
	public boolean endADR(int iState) {
		return (terminalStatesDefined() && isTerminalState(iState));
	}

	BeliefStateFactory bsf = null;

	/**
	 * 
	 * @return
	 */
	public int chooseStartState() {
		int iStartState = -1;
		double dInitialProb = m_rndGenerator.nextDouble();
		double dProb = dInitialProb;
		while (dProb > 0) {
			iStartState++;
			dProb -= probStartState(iStartState);
		}
		return iStartState;
	}

	public boolean terminalStatesDefined() {
		return (m_vTerminalStates != null);
	}

	public boolean isTerminalState(int iState) {
		if (terminalStatesDefined()) {
			return m_vTerminalStates.contains(new Integer(iState));
		}
		return false;
	}

	public int getStateCount() {// ����״̬����
		return m_cStates;
	}

	public int getActionCount() {
		return m_cActions;
	}

	public int getObservationCount() {
		return m_cObservations;
	}

	public double getDiscountFactor() {
		return m_dGamma;
	}

	/* 
	 * 获得立即回报R(b,a)
	 */
	public double immediateReward(BeliefState bs, int iAction) {

		if (bs == null)
			return 0.0;

		double dReward = bs.getActionImmediateReward(iAction);
		if (dReward != MIN_INF)
			return dReward;

		dReward = computeImmediateReward(bs, iAction);
		bs.setActionImmediateReward(iAction, dReward);

		return dReward;
	}

	protected double computeImmediateReward(BeliefState bs, int iAction) {
		int iState = 0;
		double dReward = 0.0, dPr = 0.0, dValue = 0.0;

		for (Entry<Integer, Double> e : bs.getNonZeroEntries()) {
			iState = e.getKey();
			dPr = e.getValue();
			dValue = R(iState, iAction);
			dReward += dPr * dValue;
		}
		return dReward;
	}

	/**
	 * @param iStartState
	 * @param iAction
	 * @return
	 */
	public Iterator<Entry<Integer, Double>> getNonZeroTransitions(
			int iStartState, int iAction) {
		return m_fTransition.getNonZeroEntries(iStartState, iAction);
	}

	public Iterator<Entry<Integer, Double>> getNonZeroObservations(int iAction,
			int iEndState) {
		return m_fObservation.getNonZeroEntries(iAction, iEndState);
	}

	public double probStartState(int iState) {
		return m_fStartState.valueAt(iState);
	}

	public double getMinR() {
		return m_fReward.getMinValue();
	}

	public double getMaxR() {
		return m_fReward.getMaxValue();
	}

	public double getMaxMinR() {
		int iAction = 0;
		double dMaxR = MIN_INF;
		for (iAction = 0; iAction < m_cActions; iAction++) {
			if (m_adMinActionRewards[iAction] > dMaxR)
				dMaxR = m_adMinActionRewards[iAction];
		}
		return dMaxR;
	}

	public int getStartStateCount() {
		return m_fStartState.countNonZeroEntries();
	}

	@SuppressWarnings("unchecked")
	public Iterator<Entry<Integer, Double>> getStartStates() {
		return m_fStartState.getNonZeroEntries();
	}

	public void initRandomGenerator(long iSeed) {
		m_rndGenerator.init(iSeed);
	}

	public void setRandomSeed(long iSeed) {
		m_iRandomSeed = iSeed;
	}

	public boolean isValid(int iState) {
		return true;
	}

	public Collection<Integer> getValidStates() {
		return new IntegerCollection(0, m_cStates);
	}

	public static class IntegerCollection extends AbstractCollection<Integer>
			implements Serializable {
		private static final long serialVersionUID = 1L;
		
		private int m_iFirst, m_iLast;

		public IntegerCollection(int iFirst, int iLast) {
			m_iFirst = iFirst;
			m_iLast = iLast;
		}

		public Iterator<Integer> iterator() {
			return new IntegerIterator(m_iFirst, m_iLast);
		}

		public int size() {
			return m_iLast - m_iFirst;
		}
	}

	private static class IntegerIterator implements Iterator<Integer> {

		private int m_iNext, m_iLast;

		public IntegerIterator(int iFirst, int iLast) {
			m_iNext = iFirst;
			m_iLast = iLast;
		}

		public boolean hasNext() {
			return m_iNext < m_iLast;
		}

		public Integer next() {
			m_iNext++;
			return m_iNext - 1;
		}

		public void remove() {
		}

	}

	public boolean isFactored() {
		return false;
	}

	public String getName() {
		return m_sName;
	}

	public Vector<Integer> getObservationRelevantStates() {
		return m_vObservationStates;
	}

	public RewardType getRewardType() {
		return m_rtReward;
	}

	public void initBeliefStateFactory() {
		m_bsFactory = new BeliefStateFactory(this, 20);
	}

	public Collection<Integer> getRelevantActions(BeliefState bs) {
		return new IntegerCollection(0, getActionCount());
	}

	public void addTerminalState(int iTerminalState) {
		if (m_vTerminalStates == null)
			m_vTerminalStates = new Vector<Integer>();
		m_vTerminalStates.add(iTerminalState);
	}

	public void addObservationSensitiveState(int iObservationState) {
		m_vObservationStates.add(iObservationState); // m_vObservationStates
	}

	public void setStartStateProb(int iStartState, double dValue) {
		m_fStartState.setValue(iStartState, dValue); // m_fStartState
	}

	public void setRewardType(RewardType type) {
		m_rtReward = type;

	}

	public void setReward(int iStartState, double dValue) {/* R(s) */
		m_fReward.setValue(iStartState, dValue);

	}

	public AlphaVector newAlphaVector() {
		return new TabularAlphaVector(null, 0, this);
	}

	public void setReward(int iStartState, int iAction, double dValue) {/*
																		 * R(s,a)
																		 */
		m_fReward.setValue(iStartState, iAction, dValue);

	}

	public void setReward(int iStartState, int iAction, int iEndState,
			double dValue) {/* R(s,a,s') */
		m_fReward.setValue(iStartState, iAction, iEndState, dValue);

	}

	public void setMinimalReward(int iAction, double dValue) {
		if (iAction != -1) {
			if (dValue < m_adMinActionRewards[iAction]) {
				m_adMinActionRewards[iAction] = dValue;
			}
		}
		if (dValue < m_dMinReward)
			m_dMinReward = dValue;
	}

	public void initDynamicsFunctions() {
		int[] aDims = new int[3];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		aDims[2] = m_cStates;
		m_fTransition = new SparseTabularFunction(aDims);
		aDims[0] = m_cActions;
		aDims[1] = m_cStates;
		aDims[2] = m_cObservations;
		m_fObservation = new SparseTabularFunction(aDims);
		aDims = new int[1];
		aDims[0] = m_cStates;
		if (m_cStates > g_sMaxTabularSize)
			m_fStartState = new SparseTabularFunction(aDims);
		else
			m_fStartState = new TabularFunction(aDims);
		aDims = new int[3];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		aDims[2] = m_cStates;
		if (m_cStates > g_sMaxTabularSize)
			m_fReward = new SparseTabularFunction(aDims);
		else
			m_fReward = new TabularFunction(aDims);

		m_adMinActionRewards = new double[m_cActions];
		for (int idx = 0; idx < m_cActions; idx++) {
			m_adMinActionRewards[idx] = 0;
		}

	}

	public RandomGenerator getRandomGenerator() {
		return m_rndGenerator;
	}

}
