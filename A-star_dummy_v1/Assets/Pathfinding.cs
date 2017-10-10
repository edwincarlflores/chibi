using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;

public class Pathfinding : MonoBehaviour {

	// public Transform seeker;
	// public Transform target;

	PathRequestManager requestManager;

	Grid grid;

	void Awake(){
		requestManager = GetComponent<PathRequestManager>();
		grid = GetComponent<Grid>();
	}

	// void Update(){
	// 	//if(Input.GetButtonDown("Jump")){ //to incorporate onclick to start playing scene
	// 		FindPath(seeker.position, target.position);
	// 	//}
	// }

	public void StartFindPath(Vector3 startPos, Vector3 targetPos){
		StartCoroutine(FindPath(startPos, targetPos));
	}

	IEnumerator FindPath(Vector3 startPos, Vector3 targetPos){
		Stopwatch sw = new Stopwatch();
		sw.Start();

		Vector3[] waypoints = new Vector3[0];
		bool pathSuccess = false;

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		if(startNode.walkable && targetNode.walkable){

			//A* implementation using heap structure
			Heap<Node> openSet = new Heap<Node>(grid.MaxSize); //for open set of nodes
			HashSet<Node> closedSet = new HashSet<Node>(); //for closed set of node
			openSet.Add(startNode);

			while(openSet.Count > 0){
				Node currentNode = openSet.RemoveFirst();
				closedSet.Add(currentNode);

				if (currentNode == targetNode){
					sw.Stop();
					print("Path found in " + sw.ElapsedMilliseconds + " ms");
					pathSuccess = true;

					break;
				}

				foreach(Node neighbour in grid.GetNeighbours(currentNode)){
					if(!neighbour.walkable || closedSet.Contains(neighbour)){
						continue;
					}

					int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour) + neighbour.movementPenalty;
					if(newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)){
						neighbour.gCost = newMovementCostToNeighbour;
						neighbour.hCost = GetDistance(neighbour, targetNode);
						neighbour.parent = currentNode;

						if(!openSet.Contains(neighbour)){
							openSet.Add(neighbour);
						}
						else{
							openSet.UpdateItem(neighbour);
						}
					}
				}
			}
		}
		yield return null;
		if(pathSuccess){
			waypoints = RetracePath(startNode, targetNode);
		}
		requestManager.FinishedProcessingPath(waypoints, pathSuccess);
	}

	Vector3[] RetracePath(Node startNode, Node endNode){
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while(currentNode != startNode){
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		Vector3[] waypoints = SimplifyPath(path);
		Array.Reverse(waypoints);
		//waypoints.Reverse();
		//grid.path = path;
		return waypoints;

	}

	Vector3[] SimplifyPath(List<Node> path){
		List<Vector3> waypoints = new List<Vector3>();
		Vector2 directionOld = Vector2.zero;

		for(int i = 1; i < path.Count; i++){
			Vector2 directionNew = new Vector2(path[i-1].gridX - path[i].gridX, path[i-1].gridY - path[i].gridY);
			if(directionNew != directionOld){
				waypoints.Add(path[i].worldPosition);
			}
			directionOld = directionNew;
		}
		return waypoints.ToArray();
	}

	//function to compute H cost == Heuristic function for getting the distance from the currentNode to the targetNode
	int GetDistance(Node nodeA, Node nodeB){
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX); //distance on the x-axis
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY); //distance on the y-axis

		if(dstX > dstY){
			return 14*dstY + 10*(dstX - dstY);
		}

		return 14*dstX + 10*(dstY - dstX);
	}

} //end class
