/*
 * Created on May 5, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package pomdp.algorithms;

import pomdp.utilities.BeliefState;


/**
 * @author shanigu
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public abstract class PolicyStrategy/*  implements Serializable*/{
	protected boolean m_bExploring;
	protected boolean m_bStationary;
	
	public abstract int getAction( BeliefState bsCurrent );
	
	public PolicyStrategy(){
		m_bExploring = true;
	}
	/**
	 * Returns an action for this belief state following the required policy
	 */
}
