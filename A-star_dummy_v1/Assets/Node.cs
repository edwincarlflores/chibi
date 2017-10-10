﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : IHeapItem<Node> {

	public bool walkable;
	public Vector3 worldPosition;

	//Variables for tracking each node's position
	public int gridX;
	public int gridY;

	public int movementPenalty;

	//Variable modifiers for the cost to be assigned for each grid node
	public int gCost;
	public int hCost;

	public Node parent;

	int heapIndex;

	public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY, int _penalty){
		walkable = _walkable;
		worldPosition = _worldPos;
		gridX = _gridX;
		gridY = _gridY;
		movementPenalty = _penalty;
	}

	//Method to automatically assign F cost to grid nodes
	public int fCost{
		get{
			return gCost + hCost;
		}
	}

	public int HeapIndex{
		get{
			return heapIndex;
		}
		set{
			heapIndex = value;
		}
	}

	public int CompareTo(Node nodeToCompare){
		int compare = fCost.CompareTo(nodeToCompare.fCost);

		if(compare == 0){
			compare = hCost.CompareTo(nodeToCompare.hCost);
		}

		return -compare;
	}

}
