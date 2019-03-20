package pomdp.utilities;

import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.environments.POMDP;
import pomdp.utilities.distance.DistanceMetric;
import pomdp.utilities.distance.L1Distance;

public class BeliefStateFactory{

	protected POMDP m_pPOMDP;
	/*
	 * m_bCacheBeliefStates为true时，
	 * 各种计算出来的b，都会在这里缓存着。
	 */
	protected TreeMap<BeliefState,BeliefState> m_hmCachedBeliefStates;
	protected int m_cDiscretizationLevels;
	protected int m_cBeliefPoints;//记录信念点的数量
	protected boolean m_bCacheBeliefStates;
	protected boolean m_bSparseBeliefStates = true;
	protected BeliefState m_bsInitialState;
	public static double m_dEpsilon = 0.000000001;
	protected BeliefState m_bsUniformState;
	protected RandomGenerator m_rndGenerator = new RandomGenerator( "BeliefStateFactory" );

	public long m_cBeliefStateSize;
	public BeliefStateFactory( POMDP pomdp, int cDiscretizationLevels ){
		m_pPOMDP = pomdp;
		m_cDiscretizationLevels = cDiscretizationLevels;
		init();
	}

	private void init(){
		m_hmCachedBeliefStates = new TreeMap<BeliefState,BeliefState>(getBeliefStateComparator(m_dEpsilon));
		m_cBeliefPoints = 0;
		m_bsInitialState = null;
		m_bsUniformState = null;
		m_cBeliefStateSize = 0;
		m_bCacheBeliefStates = true;
	}
	
	protected Comparator<BeliefState> getBeliefStateComparator( double dEpsilon ) {
		return BeliefStateComparator.getInstance( dEpsilon );
	}
	
	//b_a,o(s') = O(a,s',o)\sum_s tr(s,a,s')b(s)
		protected double nextBeliefValue( BeliefState bs, int iAction, int iEndState, int iObservation ){
			double dProb = 0.0, dO = 0.0, dTr = 0.0, dBelief = 0.0;
			int iStartState = 0;

			Logger.getInstance().log( "BeliefStateFactory", 11, "nextBeliefValue", " s' = " + iEndState );

			dO = m_pPOMDP.O( iAction, iEndState, iObservation );
			if( dO == 0.0 )
				return 0.0;

			Collection<Entry<Integer,Double>> colBSNonZero = bs.getNonZeroEntries();
			
			for( Entry<Integer, Double> e : colBSNonZero ){
				iStartState = e.getKey();
				dBelief = e.getValue();
				dTr = m_pPOMDP.tr( iStartState, iAction, iEndState );//执行动作iAction，iStartState转移到iEndState的概率
				dProb += dTr * dBelief;
			}

			dProb *= dO;
			return dProb;
		}

	public BeliefStateFactory( POMDP pomdp ){
		this( pomdp, -1 );
	}
	
	public boolean isCachingBeliefStates(){
		return m_bCacheBeliefStates;
	}
	
	public boolean cacheBeliefStates( boolean bCache ){
		boolean bFormerValue = m_bCacheBeliefStates;
		m_bCacheBeliefStates = bCache;
		return bFormerValue;
	}
	
	protected BeliefState newBeliefState(){
		return newBeliefState( m_cBeliefPoints );
	}
	
	public BeliefState getUniformBeliefState(){
		if( m_bsUniformState == null ){
			int iState = 0, cStates = m_pPOMDP.getStateCount();
			double dUnifomValue = 1.0 / cStates;
			m_bsUniformState = newBeliefState();
			for( iState = 0 ; iState < cStates ; iState++ )
				m_bsUniformState.setValueAt( iState, dUnifomValue );

			BeliefState bsExisting = m_hmCachedBeliefStates.get( m_bsUniformState );
			if( bsExisting == null ){
				//cacheBeliefState( m_bsUniformState );
				m_cBeliefPoints++;
			}
			else{
				m_bsUniformState = bsExisting;
			}

		}
		return m_bsUniformState;
	}

	
	protected BeliefState newBeliefState( int id ){
		if( !m_bCacheBeliefStates )
			id = -1;

		BeliefState bs = new TabularBeliefState( m_pPOMDP.getStateCount(), m_pPOMDP.getActionCount(),
				m_pPOMDP.getObservationCount(), id,
				m_bSparseBeliefStates, m_bCacheBeliefStates, this );

		return bs;
	}
	
	public BeliefState getInitialBeliefState(){
		if( m_bsInitialState == null ){
			BeliefState bsInitial = newBeliefState();
			m_cBeliefPoints++;
			int iState = 0, cStates = m_pPOMDP.getStateCount();
			double dSum = 0.0, dValue = 0.0;
			Logger.getInstance().logln(m_pPOMDP.probStartState(0));
			for(iState = 0 ;iState < cStates; iState++){
				dValue = m_pPOMDP.probStartState( iState );
				bsInitial.setValueAt( iState, dValue );//将起始概率赋给初信念点
				dSum += dValue;
			}
			if( dSum < 0.99999 || dSum > 1.000001 )
				Logger.getInstance().log( "BeliefStateFactory", 0, "getInitialBeliefState", "Corrupted initial belief state " + m_bsInitialState.toString() );
			cacheBeliefState( bsInitial );
			m_bsInitialState = bsInitial;
			Logger.getInstance().log( "BeliefStateFactory", 11, "getInitialBeliefState", m_bsInitialState.toString() );
		}
		return m_bsInitialState;
	}
	
	public double calcNormalizingFactor( BeliefState bs, int iAction, int iObservation ){//计算bs执行动作action，得到观察observation的概率

		double dProb = 0.0, dO = 0.0, dBelief = 0.0, dTr = 0.0, dSum = 0.0;
		int iStartState = 0, iEndState = 0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = null;
		Iterator<Entry<Integer,Double>> itNonZeroBeliefs = bs.getNonZeroEntries().iterator();
		Map.Entry<Integer,Double> eTransition = null, eBelief = null;

		while( itNonZeroBeliefs.hasNext() ){
			eBelief = itNonZeroBeliefs.next();
			iStartState = (eBelief.getKey()).intValue();//获得状态
			dBelief = (eBelief.getValue()).doubleValue();//得到每个状态的概率
			dSum = 0.0;
			itNonZeroTransitions = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );//在状态s下，执行动作a所能够转移到的状态
			while( itNonZeroTransitions.hasNext() ){
				eTransition = itNonZeroTransitions.next();
				iEndState = (eTransition.getKey()).intValue();
				dTr = (eTransition.getValue()).doubleValue();//转移到iEndState的概率
				dO = m_pPOMDP.O( iAction, iEndState, iObservation );//已知执行动作a并转移到状态s，得到观察o的概率
				dSum += dO * dTr;
			}
			dProb += dSum * dBelief;
		}
          return dProb;
	}
	/**
	 * Computes the next belief state given the current belief state, and action and an observation
	 * @param bs - current belief state
	 * @param iAction - action
	 * @param iObservation - observation
	 * @return next belief state
	 */
	@SuppressWarnings("rawtypes")
	public BeliefState nextBeliefState( BeliefState bs, int iAction, int iObservation ){//计算下一个信念点
		try{
			BeliefState bsNext = newBeliefState();//new一个信念点

			double dNormalizingFactor = 0.0, dNextValue = 0.0;
			double dBelief = 0.0, dTr = 0.0, dOb = 0.0;
			int iEndState = 0, iStartState = 0;
			int cStates = m_pPOMDP.getStateCount();
			Collection<Entry<Integer, Double>> cNonZeroBeliefs = bs.getNonZeroEntries();
			Iterator<Entry<Integer, Double>> itNonZeroTransitions = null;
			Entry<Integer, Double> eTr = null;

			dNormalizingFactor = 0.0;

			if( cNonZeroBeliefs.size() > m_pPOMDP.getStateCount() / 2.0 ){	//sparse beliefs
				for( iEndState = 0 ; iEndState < cStates ; iEndState++ ){
					dNextValue = nextBeliefValue( bs, iAction, iEndState, iObservation );
					bsNext.setValueAt( iEndState, dNextValue );//设置信念点在每一个状态的概率
					dNormalizingFactor += dNextValue;
				}
			}
			else
			{
				for( Entry<Integer, Double> eBelief : cNonZeroBeliefs ){ //dense beliefs
					iStartState = eBelief.getKey();
					dBelief = eBelief.getValue();
					itNonZeroTransitions = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
					while( itNonZeroTransitions.hasNext() ){
						eTr = itNonZeroTransitions.next();
						iEndState = eTr.getKey();
						dTr = eTr.getValue();
						dOb = m_pPOMDP.O( iAction, iEndState, iObservation );
						if( dOb > 0.0 ){
							dNextValue = bsNext.valueAt( iEndState );
							bsNext.setValueAt( iEndState, dNextValue + dBelief * dTr * dOb );
							dNormalizingFactor += dBelief * dTr * dOb;
						}
					}
				}
			}

			bs.setProbabilityOGivenA( iAction, iObservation, dNormalizingFactor );

			if( dNormalizingFactor == 0.0 ){
				return null;
			}



			Iterator itNonZeroEntries = bsNext.getNonZeroEntries().iterator();
			Map.Entry e = null;
			while( itNonZeroEntries.hasNext() ){
				e = (Entry) itNonZeroEntries.next();
				iEndState = ((Number) e.getKey()).intValue();
				dNextValue = ((Number) e.getValue()).doubleValue();
				bsNext.setValueAt( iEndState, dNextValue / dNormalizingFactor );//同样设置状态的概率值
			}

			

			if( m_bCacheBeliefStates ){
				BeliefState bsExisting = m_hmCachedBeliefStates.get( bsNext );
				if( bsExisting == null ){
					cacheBeliefState( bsNext );
					m_cBeliefPoints++;
				}
				else{
					bsNext = bsExisting;
				}

				if( bsNext != bs )
					bsNext.addPredecessor( bs, dNormalizingFactor, iAction );
			}

			return bsNext;
		}
		catch( Error err ){
			Runtime rtRuntime = Runtime.getRuntime();
			Logger.getInstance().logln( "|BeliefSpace| " + m_cBeliefPoints + ", " + err +
					" allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
					" free " + rtRuntime.freeMemory() / 1000000 +
					" max " + rtRuntime.maxMemory() / 1000000 );

			err.printStackTrace();
			System.exit( 0 );
		}
		return null;
	}
	
	protected synchronized void cacheBeliefState( BeliefState bs ){
		m_hmCachedBeliefStates.put( bs, bs );
	}
	
	public void clear() {
		init();
	}

	public POMDP getPOMDP() {
		return m_pPOMDP;
	}
	
	/**
	 * 计算最远的后继
	 * 
	 * action是随机取一个，o是全遍历的
	 * @param vBeliefPoints 用来计算距离的
	 * @param bs 当前b
	 * @return
	 */
	public BeliefState computeRandomFarthestSuccessor( Vector<BeliefState> vBeliefPoints, BeliefState bs )
	{
		int cObservations = m_pPOMDP.getObservationCount();
		int iAction = 0, iObservation = 0;
		
		BeliefState bsMaxDist = null;
		BeliefState bsNext = null;
		double dMaxDist = 0.0, dDist = 0.0;
		double dOb = 0.0;
		
		//随机取一个action
		iAction = m_rndGenerator.nextInt( m_pPOMDP.getActionCount() );
		//遍历observation
		for(iObservation = 0; iObservation<cObservations; iObservation++)
		{
			//bs下执行iAction观察到iObservation的概率
			dOb = bs.probabilityOGivenA( iAction, iObservation );
			if(dOb>0.0)
			{
				//计算b、a、o时，后继b
				bsNext = bs.nextBeliefState(iAction, iObservation);
				if(bsNext!=null)
				{
					dDist = distance(vBeliefPoints, bsNext);
					if( dDist > dMaxDist )//哪个后继距离信念点集合vBeliefPoints最远就取哪个
					{
						dMaxDist = dDist;
						bsMaxDist = bsNext;
					}
				}
			}
		}
		if( dMaxDist == 0.0 )
		{
			return null;
		}
		return bsMaxDist;
	}
	
	/**
	 * 求vBeliefStates这个集合与bs的最近距离————即求集合中的信念状态与bs的最短距离
	 * 
	 * @param vBeliefStates
	 * @param bs
	 * @return
	 */
	public double distance(Collection<BeliefState> vBeliefStates, BeliefState bs)
	{
		Iterator<BeliefState> it = vBeliefStates.iterator();
		double dDist = 0.0;
		double dMinDist = 10000.0;
		BeliefState bsCurrent = null;
		
		DistanceMetric dmDistance = L1Distance.getInstance();
		
		while(it.hasNext())
		{
			bsCurrent = (BeliefState)it.next();
			dDist = dmDistance.distance(bs, bsCurrent);
			if(dDist<dMinDist)
			{
				dMinDist = dDist;
			}
		}
		return dMinDist;
	}
}
