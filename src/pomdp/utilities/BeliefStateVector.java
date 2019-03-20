package pomdp.utilities;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Vector;

public class BeliefStateVector<E> extends Vector<E> 
{
	private static final long serialVersionUID = 1L;
    
	/**
	 * 0����    1����һ��    2���ڶ���
	 */
	private ArrayList<ArrayList<E>> treeLevelInfo = null;
	
	/**
	 * ���ֻ����µ�b����ô������tree��Ϣ���ͷ�����
	 */
	private ArrayList<ArrayList<E>> treeLevelInfoComplete = null;
	
	private boolean notComplete = false;
	
	public BeliefStateVector()
	{
		super();
		treeLevelInfo = new ArrayList<ArrayList<E>>();
	}
	
	public BeliefStateVector(Vector<E> vector)
	{
		super(vector);
	}
	
	public BeliefStateVector(BeliefStateVector<E> beliefStateVector) 
	{
		super(beliefStateVector);
		ArrayList<ArrayList<E>> treeLevelInfoIn = beliefStateVector.getTreeLevelInfo();
		treeLevelInfo = new ArrayList<ArrayList<E>>();
		for(int i = 0; i < treeLevelInfoIn.size(); i++)
		{
			treeLevelInfo.add(new ArrayList<E>());
			treeLevelInfo.get(i).addAll(treeLevelInfoIn.get(i));
		}
	}
	
	public ArrayList<ArrayList<E>> getTreeLevelInfo()
	{
		return treeLevelInfo;
	}
	
	public synchronized void add(E parent, E current)
	{
		super.add(current);
		//Ϊ��
		if(parent==null)
		{
			treeLevelInfo.clear();
			ArrayList<E> levelOne = new ArrayList<E>();
			treeLevelInfo.add(levelOne);
			levelOne.add(current);
		}
		else//�Ǹ�
		{
			int pLevelNum = getLevelNum(parent);
			if(pLevelNum<0)//parent������,�����޷����
			{
				Logger.getInstance().logln("Error: BeliefStateVector.add(E parent, E current): parent not exist!");
		        return;
			}
			else// parent����
			{
				pLevelNum += 1;
				if(treeLevelInfo.size()<=pLevelNum)//��Ҫ����һ��
				{
					ArrayList<E> newLevel = new ArrayList<E>();
					treeLevelInfo.add(newLevel);
					if(notComplete)
					{
						ArrayList<E> newLevel2 = new ArrayList<E>();
						treeLevelInfoComplete.add(newLevel2);
					}
				}
				treeLevelInfo.get(pLevelNum).add(current);//�ҵ��������
				if(notComplete)
				{
					treeLevelInfoComplete.get(pLevelNum).add(current);
				}
			}
		}
	}
	
	/**
	 * ��ѯһ��b�Ĳ���
	 * 
	 * @param e
	 * @return -1: not found
	 */
	public synchronized int getLevelNum(E e)
	{
		if(notComplete)
		{
			for(int i=0;i<treeLevelInfoComplete.size();i++)
			{
				if(treeLevelInfoComplete.get(i).contains(e))
				{
					return i;
				}
			}
		}
		else
		{
			for(int i=0;i<treeLevelInfo.size();i++)
			{
				if(treeLevelInfo.get(i).contains(e))
				{
					return i;
				}
			}
		}
		return -1;
	}
	
	/**
	 * ���һ����tree�ϣ���������ɨ���iterator
	 * 
	 * @return
	 */
	public Iterator<E> getTreeDownUpIterator() 
	{
		ArrayList<E> resultList = new ArrayList<E>();
		for (int i = treeLevelInfo.size() - 1; i >= 0; i--) 
		{
			resultList.addAll(treeLevelInfo.get(i));//addAll(E),����E��һ������,��ô�ὫE�����е�Ԫ�ض�����list��
		}
		return resultList.iterator();
	}
}
