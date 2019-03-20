package pomdp.utilities.concurrent;

import java.util.Vector;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.environments.POMDP.RewardType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.Logger;
import pomdp.utilities.RandomGenerator;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class ComputeDiscountedReward extends Task {

	private int m_cMaxStepsToGoal;
	private double m_dSumDiscountedReward;
	private int m_cTests;
	private LinearValueFunctionApproximation m_vValueFunction;
	@Override
	public void execute() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void copyResults(Task tProcessed) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public Element getDOM(Document doc) throws Exception {
		// TODO Auto-generated method stub
		return null;
	}
	
	
}
