package pomdp.valuefunction;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.RandomGenerator;


public class MDPValueFunction extends PolicyStrategy {
	protected LinearValueFunctionApproximation m_vValueFunction;
	protected POMDP m_pPOMDP;
	protected int m_cObservations;
	protected int m_cStates;
	protected int m_cActions;
	protected double m_dGamma;
	protected AlphaVector m_avBestActions;
	protected boolean m_bConverged;
	protected boolean m_bLoaded;
	protected RandomGenerator m_rndGenerator;
	
	public MDPValueFunction( POMDP pomdp, double dExplorationRate ){
		m_pPOMDP = pomdp;
		m_vValueFunction = new LinearValueFunctionApproximation( 0.0001, false );
		m_cStates = m_pPOMDP.getStateCount();
		m_cActions = m_pPOMDP.getActionCount();
		m_cObservations = m_pPOMDP.getObservationCount();
		m_dGamma = m_pPOMDP.getDiscountFactor();
		m_avBestActions = null;
		m_bConverged = false;
		m_bLoaded = false;
		m_rndGenerator = new RandomGenerator( "MDPVI", 0 );
	}

	@Override
	public int getAction(BeliefState bsCurrent) {
		return 0;
	}
}
