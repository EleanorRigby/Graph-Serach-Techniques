#include<iostream>
#include<cstdio>
#include<fstream>
#include<string>
#include<stack>
#include<vector>
#include<queue>
#include<algorithm>
#include<map>

using namespace std;


/*
@class-info	Structure node which stores the state and other information needed like level of tree and parent node
for backtracking the solution path.
@Note:			For BFS and DFS we will use the cum_cost as the depth of tree we went to find the solution.
@Note:			For UniCost BFS we will use cum_cost to track actual cost of path.
*/
class node {

public:

	string		name;		///< Name of the Person
	long long	index;		///< Index of the person
	long long	cum_cost;	///< Cumulative cost for the person from intial state
	long long	depth;		///< Depth of the node in the tree.
	node	*	parent;		///< Parent of the node to backtrack mutual friend network.
	
	//Copy constructor
	node(const node & that) {
		name		= that.name;
		index		= that.index;
		cum_cost	= that.cum_cost;
		depth		= that.depth;
		parent		= that.parent;

	}


	//Default Contructor
	node() {
		name		= "";
		index		= 0;
		cum_cost	= 0;
		depth		= 0;
		parent		= NULL;
	}

	//Parameterized Constructor
	node(string pname, long long pindex, long long pcumcost, long long pdepth, node * pparent) {
		name		= pname;
		index		= pindex;
		cum_cost	= pcumcost;
		depth		= pdepth;
		parent		= pparent;
	}

	//Overloading < operator to be used for maps.
	//@remove Not used till now
	bool operator < (const node & p_node)
	{
		return cum_cost < p_node.cum_cost ? true : false;
	}

};





/*
@brief Custom Comparator : For the purpose of custom comparator in a std::map
*/
class EquateStrings {

public:

	/*
	Returns true if strings are equal
	param[in]	string	str1	first string
	param[in]	string	str2	second string

	@return		bool			whether the two strings are equal or not
	*/
	bool operator() (const string & str1, const string & str2)
	{
		return str1.compare(str2) < 0;
	}


};


/*
@brief Custom Comparator : For the purpose of custom comparator in a priority queue for Uni Cost Search.
*/
class NodeCostComparatorUniCost {

public:

	/*
	Returns true if strings are equal
	param[in]	string	str1	first string
	param[in]	string	str2	second string

	@return		bool			whether the two strings are equal or not
	*/
	bool operator() (const node & node1, const node & node2)
	{
		if (node1.cum_cost != node2.cum_cost)
			return node1.cum_cost > node2.cum_cost;
		else
			return node1.name.compare(node2.name) > 0;
	}


};

/*
@brief Custom Comparator : For the purpose of custom comparator in a priority queue for BFS
*/
class NodeCostComparatorBFS {

public:

	/*
	Returns true if strings are equal
	param[in]	string	str1	first string
	param[in]	string	str2	second string

	@return		bool			whether the two strings are equal or not
	*/
	bool operator() (const node & node1, const node & node2)
	{
		if (node1.depth != node2.depth)
			return node1.depth > node2.depth;
		else
			return node1.name.compare(node2.name) > 0;
	}


};


/*
@brief Custom Comparator : For the purpose of custom comparator in a priority queue for BFS
*/
class NodeCostComparatorDFS {

public:

	/*
	Returns true if strings are equal
	param[in]	string	str1	first string
	param[in]	string	str2	second string

	@return		bool			whether the two strings are equal or not
	*/
	bool operator() (const node & node1, const node & node2)
	{
		if (node1.depth != node2.depth)
			return node1.depth < node2.depth;
		else
			return node1.name.compare(node2.name) > 0;
	}


};





//The goal state
node goal_state;

//stack of solution path
stack<node> soluton_path;

//queue for expansion path.
queue<node> expansion_path;

//Global map to hold names and corresponding integer values
map<long long, string>		name_map;

//Globar 2-D vector to hold social graph
vector< vector<long long> >	social_graph;


/*
@brief Checks the goal

param[in]	node	p_state		state to be compared with goal

@return		bool				whether goal has been reached or not.
*/
bool GoalCheck(node & p_state)
{
	return p_state.name.compare(goal_state.name) == 0 ? true : false;

}

/*********************************************DFS Start***********************************************************/

/*
@brief : Functions travel through the queue and finds if node exists. Returns the node if found

param

*/
bool findinDFSq(priority_queue<node, deque<node>, NodeCostComparatorDFS> & p_myQ, node & src, bool pFindOnly = true) {

	queue<node>			 tempstoreQ;
	node				 tempstore;
	bool				 flag = false;

	
	//case: while original queue is not empty
	while (!p_myQ.empty()) {

		tempstore = p_myQ.top();

		//case: if node got from priority queue is same as one we are trying to insert(src)
		if (tempstore.name.compare(src.name) == 0) {

			flag = true;

			//case: if node in open queue has path cost greater than child to be inserted
			if (!pFindOnly && tempstore.depth > src.cum_cost) {
				//Remove it from queue
				p_myQ.pop();
				//push the child instead
				p_myQ.push(src);

			}



			break;

		}

		//case: node on top is not equal to child we are looking for (otherwise code would break out of loop)
		//pop the queue
		p_myQ.pop();
		//push into temporary store
		tempstoreQ.push(tempstore);

	}

	//now irrespective of whether we found node or not we will restore original node

	while (!tempstoreQ.empty()) {
		p_myQ.push(tempstoreQ.front());
		tempstoreQ.pop();
	}

	return flag;

}

/*
@brief Generates children states and inserts in stack if not already explored.

param [in]		node	p_currstate		current state
param [in,out]	queue	p_myS			stack in which insertion is to be done
param [in,out]	map		p_map			Map from which verification of explored state has to be done.

This function will generate children from the current state passed.
If the states are not explored they are inserted into the stack.
It uses the global array of graph for lookup of possible states or nodes in the graph and their edges.

@return	void
*/
void DFS_Gen_Child_Insert_after_Verify_from_Explored_States(node & p_currstate, priority_queue<node, deque<node>, NodeCostComparatorDFS> & p_myS,
	map<string, node, EquateStrings> & p_map)
{
	long long iterator;

	//From the current state extract the name of the person
	string		name = p_currstate.name;

	//extract the index of the person for social graph lookup
	long long	index = p_currstate.index;

	//For this particular index traverse the social graph to generate children
	for (iterator = 0; iterator < social_graph[index].size(); ++iterator) {

		//case:there exists a link, otherwise value will be zero
		if (social_graph[index][iterator]) {

			//generate child 
			node child( name_map.find(iterator)->second,
				iterator,
				p_currstate.cum_cost + social_graph[index][iterator],
				p_currstate.depth + 1,
				&p_currstate
			);

			//check if child is already in explored state
			//if not added to explored.
			//the map returns the map.end if not in the map
			if (p_map.find(child.name) == p_map.end() && findinDFSq(p_myS, child, true) == false) {
				//push the child state in queue
				p_myS.push(child);


			//case: node is present in the queue
			}
			else if (findinDFSq(p_myS, child, false)) {
				continue;

			//case: there exists a node in map of explored state same as child and it's path cost is more than child
			}
			else if (p_map.find(child.name) != p_map.end() && p_map.find(child.name)->second.depth > child.depth) {
				//delete the original entry in explored map.
				p_map.erase(p_map.find(child.name));
				//add child to unexplored queue
				p_myS.push(child);

			}

			////check if child is already in explored state
			////if not added to explored.
			////the map returns the map.end if not in the map
			//if (p_map.find(child.name) == p_map.end() && findinDFSq(p_myS, child) == false) {
			//	//push the child state in queue
			//	p_myS.push(child);
			//	
			//}//end if exploration 

		}//end if edge existed


	}//end for all child explored

}


/*
@brief BFS algorithm that takes in the start state and outputs a success or failure

param[in]	node	start	starting state

@return		bool			indicates success of failure of whether goal is found or not
*/
bool DFS(node & start)
{
	node	goal;				///< Variable to hold top entry of queue.
	bool	flag = false;		///< Indicates whether goal found or not.
	node *  first;

	//Stack for the BFS.
	//Although implemented as queue but will behave as stack as we will follow LIFO
	//*******Pay Attention Later***********
	//@note : this can become a priority queue based on custom comparison of depth of node and name for
	//alphabetical popping of states.
	//**************************************
	priority_queue<node, deque<node>, NodeCostComparatorDFS> myQ_unexplored_states;
		
	//Map of explored states
	//Takes the string as a key i.e. name of the person and the entire node as a value
	//EquateString is the custom comparator for the map.
	map<string, node, EquateStrings> map_explored_states;

	//push the start state into Queue
	myQ_unexplored_states.push(start);

	//add this to the map of explored state as well
	//The map[index]. This index is the key and the data is on RHS.
	//map_explored_states[start.name] = start;



	//loop till the queue is not empty to explore all the states.
	while (!myQ_unexplored_states.empty()) {

		//extract the front element. FIFO
		first = new node(myQ_unexplored_states.top());

		//add to expansion path
		expansion_path.push(myQ_unexplored_states.top());

		//push the node in explored map
		map_explored_states[first->name] = (*first);

		//pop it to delete from the queue
		myQ_unexplored_states.pop();

		//Check if this the goal
		if (GoalCheck(*first)) {
			flag = true;
			goal = *first;
			break;
		}

		//Generate children states and insert on verification from explored states
		DFS_Gen_Child_Insert_after_Verify_from_Explored_States(*first, myQ_unexplored_states, map_explored_states);

	}//end while



	//case: goal not found
	if (!flag) {

		return false;

		//case: goal found
	}
	else {

		//push the goal into solution path
		soluton_path.push(goal);

		//intitalize parent to track back solution
		node  * parent = goal.parent;

		//keep pushing onto stack until we reach root
		while (parent) {
			//push the subsequent parents to stack
			soluton_path.push(*(parent));
			//move parent upwards towards start state.
			parent = parent->parent;

		}

		//Populate solution path stack from goal state.
		return true;

	}

}

/*********************************************DFS END***********************************************************/

/*********************************************Uni Cost Start***********************************************************/

/*
@brief : Functions travel through the queue and finds if node exists. Returns the node if found

param

*/
bool findandswapstate(priority_queue<node, deque<node>, NodeCostComparatorUniCost> & p_myQ, node & src, bool pFindOnly = true) {

	queue<node>			 tempstoreQ;
	node				 tempstore;
	bool				 flag = false;

	//case: while original queue is not empty
	while (!p_myQ.empty()) {

		tempstore = p_myQ.top();

		//case: if node got from priority queue is same as one we are trying to insert(src)
		if (tempstore.name.compare(src.name) == 0) {

			flag = true;

			//case: if node in open queue has path cost greater than child to be inserted
			if (!pFindOnly && tempstore.cum_cost > src.cum_cost) {
				//Remove it from queue
				p_myQ.pop();
				//push the child instead
				p_myQ.push(src);

			}

			break;

		}

		//case: node on top is not equal to child we are looking for (otherwise code would break out of loop)
		//pop the queue
		p_myQ.pop();
		//push into temporary store
		tempstoreQ.push(tempstore);

	}

	//now irrespective of whether we found node or not we will restore original node

	while (!tempstoreQ.empty()) {
		p_myQ.push(tempstoreQ.front());
		tempstoreQ.pop();
	}

	return flag;

}



/*
@brief Generates children states and inserts in stack if not already explored.

param [in]		node	p_currstate		current state
param [in,out]	queue	p_myS			stack in which insertion is to be done
param [in,out]	map		p_map			Map from which verification of explored state has to be done.

This function will generate children from the current state passed.
If the states are not explored they are inserted into the stack.
It uses the global array of graph for lookup of possible states or nodes in the graph and their edges.

@return	void
*/


void UniCost_Gen_Child_Insert_after_Verify_from_Explored_States(node & p_currstate, priority_queue<node, deque<node>, NodeCostComparatorUniCost> & p_myQ,
	map<string, node, EquateStrings> & p_map)
{
	long long	iterator;
	node	*	find = NULL;

	//From the current state extract the name of the person
	string		name = p_currstate.name;

	//extract the index of the person for social graph lookup
	long long	index = p_currstate.index;

	//For this particular index traverse the social graph to generate children
	for (iterator = 0; iterator < social_graph[index].size(); ++iterator) {

		//case:there exists a link, otherwise value will be zero
		if (social_graph[index][iterator]) {

			//generate child 
			node child ( name_map.find(iterator)->second,
				iterator,
				p_currstate.cum_cost + social_graph[index][iterator],
				p_currstate.depth + 1,
				&p_currstate
			);

			//check if child is already in explored state
			//if not added to explored.
			//the map returns the map.end if not in the map
			if (p_map.find(child.name) == p_map.end() && findandswapstate(p_myQ, child, true) == false) {
				//push the child state in queue
				p_myQ.push(child);
				
			
			//case: node is present in the queue
			} else if (findandswapstate(p_myQ, child, false)) {
				continue;

			//case: there exists a node in map of explored state same as child and it's path cost is more than child
			} else if ( p_map.find(child.name) != p_map.end() && p_map.find(child.name)->second.cum_cost > child.cum_cost) {
				//delete the original entry in explored map.
				p_map.erase(p_map.find(child.name));
				//add child to unexplored queue
				p_myQ.push(child);
			}

		}//end if edge existed


	}//end for all child explored

}


/*
@brief BFS algorithm that takes in the start state and outputs a success or failure

param[in]	node	start	starting state

@return		bool			indicates success of failure of whether goal is found or not
*/


bool UniCost(node & start)
{
	node	goal;				///< Variable to hold top entry of queue.
	bool	flag = false;		///< Indicates whether goal found or not.
	node *  first;

	//Stack for the BFS.
	//Although implemented as queue but will behave as stack as we will follow LIFO
	//*******Pay Attention Later***********
	//@note : this can become a priority queue based on custom comparison of depth of node and name for
	//alphabetical popping of states.
	//**************************************
	priority_queue<node, deque<node>, NodeCostComparatorUniCost> myQ_unexplored_states;

	//Map of explored states
	//Takes the string as a key i.e. name of the person and the entire node as a value
	//EquateString is the custom comparator for the map.
	map<string, node, EquateStrings> map_explored_states;


	//push the start state into Queue
	myQ_unexplored_states.push(start);

	//add this to the map of explored state as well
	//The map[index]. This index is the key and the data is on RHS.
	//map_explored_states[start.name] = start;

	//loop till the queue is not empty to explore all the states.
	while (!myQ_unexplored_states.empty()) {

		//extract the front element. FIFO
		first = new node(myQ_unexplored_states.top());

		//add to expansion path
		expansion_path.push(myQ_unexplored_states.top());

		//push the node in explored map
		map_explored_states[first->name] = (*first);

		//pop it to delete from the queue
		myQ_unexplored_states.pop();

		//Check if this the goal
		if (GoalCheck(*first)) {
			flag = true;
			goal = *first;
			break;
		}

		//Generate children states and insert on verification from explored states
		UniCost_Gen_Child_Insert_after_Verify_from_Explored_States(*first, myQ_unexplored_states, map_explored_states);

	}//end while



	//case: goal not found
	if (!flag) {

		return false;

		//case: goal found
	}
	else {

		//push the goal into solution path
		soluton_path.push(goal);

		//intitalize parent to track back solution
		node  * parent = goal.parent;

		//keep pushing onto stack until we reach root
		while (parent) {
			//push the subsequent parents to stack
			soluton_path.push(*(parent));
			//move parent upwards towards start state.
			parent = parent->parent;

		}

		//Populate solution path stack from goal state.
		return true;

	}

}
/*********************************************Uni Cost End***********************************************************/



/***********************************************BFS Start ******************************************************/

/*
@brief : Functions travel through the queue and finds if node exists. Returns the node if found

param

*/
bool findinBFSq(priority_queue<node, deque<node>, NodeCostComparatorBFS> & p_myQ, node & src, bool pFindOnly = true) {

	queue<node>			 tempstoreQ;
	node				 tempstore;
	bool				 flag = false;

	//case: while original queue is not empty
	while (!p_myQ.empty()) {

		tempstore = p_myQ.top();

		//case: if node got from priority queue is same as one we are trying to insert(src)
		if (tempstore.name.compare(src.name) == 0) {

			flag = true;

			//case: if node in open queue has path cost greater than child to be inserted
			if (!pFindOnly && tempstore.depth > src.depth) {
				//Remove it from queue
				p_myQ.pop();
				//push the child instead
				p_myQ.push(src);

			}

			break;

		}

		//case: node on top is not equal to child we are looking for (otherwise code would break out of loop)
		//pop the queue
		p_myQ.pop();
		//push into temporary store
		tempstoreQ.push(tempstore);

	}

	//now irrespective of whether we found node or not we will restore original node

	while (!tempstoreQ.empty()) {
		p_myQ.push(tempstoreQ.front());
		tempstoreQ.pop();
	}

	return flag;

}



/*
@brief Generates children states and inserts in Queue if not already explored.

param [in]		node	p_currstate		current state
param [in,out]	queue	p_myQ			Queue in which insertion is to be done
param [in,out]	map		p_map			Map from which verification of explored state has to be done.

This function will generate children from the current state passed.
If the states are not explored they are inserted into the queue.
It uses the global array of graph for lookup of possible states or nodes inthe graph and their edges.

@return	void
*/
void BFS_Gen_Child_Insert_after_Verify_from_Explored_States(node & p_currstate, priority_queue<node, deque<node>, NodeCostComparatorBFS> & p_myQ,
															map<string, node, EquateStrings> & p_map)
{
	long long iterator;

	//From the current state extract the name of the person
	string		name		= p_currstate.name;

	//extract the index of the person for social graph lookup
	long long	index		= p_currstate.index;

	//For this particular index traverse the social graph to generate children
	for (iterator = 0; iterator < social_graph[index].size(); ++iterator) {

		//case:there exists a link, otherwise value will be zero
		if (social_graph[index][iterator]) {
			
			//generate child 
			node child ( name_map.find(iterator)->second, 
						 iterator, 
						 p_currstate.cum_cost + social_graph[index][iterator],
						 p_currstate.depth + 1, 
						 &p_currstate 
					   );

			//check if child is already in explored state
			//if not added to explored.
			//the map returns the map.end if not in the map
			if (p_map.find(child.name) == p_map.end() && findinBFSq(p_myQ, child, true) == false) {
				//push the child state in queue
				p_myQ.push(child);


				//case: node is present in the queue
			}
			else if (findinBFSq(p_myQ, child, false)) {
				continue;

				//case: there exists a node in map of explored state same as child and it's path cost is more than child
			}
			else if (p_map.find(child.name) != p_map.end() && p_map.find(child.name)->second.depth > child.depth) {
				//delete the original entry in explored map.
				p_map.erase(p_map.find(child.name));
				//add child to unexplored queue
				p_myQ.push(child);
			}
		
			////check if child is already in explored state
			////if not added to explored.
			////the map returns the map.end if not in the map
			//if (p_map.find(child.name) == p_map.end() && findinBFSq(p_myQ, child) == false) {
			//	//push the child state in queue
			//	p_myQ.push(child);
			//	
			//}//end if exploration 
	
		}//end if edge existed


	}//end for all child explored

}


/*
@brief BFS algorithm that takes in the start state and outputs a success or failure 

param[in]	node	start	starting state

@return		bool			indicates success of failure of whether goal is found or not
*/
bool BFS(node & start)
{
	node	goal;				///< Variable to hold top entry of queue.
	bool	flag = false;		///< Indicates whether goal found or not.
	node *  first;

	//Queue for the BFS.
	priority_queue<node, deque<node>, NodeCostComparatorBFS> myQ_unexplored_states;

	//Map of explored states
	//Takes the string as a key i.e. name of the person and the entire node as a value
	//EquateString is the custom comparator for the map.
	map<string, node, EquateStrings> map_explored_states;
	
	//push the start state into Queue
	myQ_unexplored_states.push(start);

	//add this to the map of explored state as well
	//The map[index]. This index is the key and the data is on RHS.
	//map_explored_states[start.name] = start;

	//loop till the queue is not empty to explore all the states.
	while (!myQ_unexplored_states.empty()) {

		//extract the front element. FIFO
		first = new node(myQ_unexplored_states.top());
		
		//add to expansion path
		expansion_path.push(myQ_unexplored_states.top());

		//pop it to delete from the queue
		myQ_unexplored_states.pop();

		//push the node in explored map
		map_explored_states[first->name] = (*first);

		//Check if this the goal
		if (GoalCheck(*first)) {
			flag = true;
			goal = *first;
			break;
		}

		//Generate children states and insert on verification from explored states
		BFS_Gen_Child_Insert_after_Verify_from_Explored_States(*first, myQ_unexplored_states, map_explored_states);

		

	}//end while

	
	
	//case: goal not found
	if (!flag) {
		
		return false;
	
	//case: goal found
	} else {

		//push the goal into solution path
		soluton_path.push(goal);

		//intitalize parent to track back solution
		node  * parent = goal.parent;

		//keep pushing onto stack until we reach root
		while (parent) {
			//push the subsequent parents to stack
			soluton_path.push(*(parent));
			//move parent upwards towards start state.
			parent = parent->parent;

		}

		//Populate solution path stack from goal state.
		return true;

	}

}

/*********************************************BFS End***********************************************************/


//Main Entry Point.
int main() {


	fstream		myfile, ofile;								///< File pointer
	int			choice;										///< Gets the type of algorithm. 1 = BFS 2 = DFS 3 = UniCost.
	long long	number;										///< to read number of nodes too.	
	long long	edge;										///< Reads edge weight
	string		start;										///< Start state name
	string		goaln;										///< goal state name
	string		name;										///< name of edges from input
	bool		result = false;										///< Whether algorithm succeeded or not

	//intialize start state
	node startstate( "", 0, 0, 0, NULL );

	//open the file in read mode
	myfile.open("input.txt", fstream::in | fstream::out);

	

		//Reads type of algorithm.
		myfile >> choice;
		//cout   << endl << "type of algo " << number << endl;

		//Read the start state name
		myfile >> start;

		//fill the name to start state
		startstate.name = start;

		//read the goal state name
		myfile >> goaln;

		//set the name in the goal state
		goal_state.name = goaln;

		//read the number of nodes.
		myfile >> number;

		//Populate the name map which stores name of person along with index of social graph.
		//It will help us to match names with their indexes and thus to generate children nodes quickly
		for (int i = 0; i < number; ++i) {
			//read the name
			myfile >> name;
			//push into map
			name_map[i] = name;
			//set index for start state
			if (startstate.name.compare(name) == 0)
				startstate.index = i;

		}

		//Next start reading into social graph.
		//Fill the 2-d matrix or 2-d vector 
		for (int i = 0; i < number; ++i) {
			//Initialize one row
			social_graph.push_back(vector<long long>());
			//loop till all nodes are covered
			for (int j = 0; j < number; ++j) {
				//read the edge
				myfile >> edge;
				//push the edge to the vector
				social_graph[i].push_back(edge);
			}

		}


	

	myfile.close();

	//case : Which algorithm to run
	switch (choice)
	{
		case 1:  {
			result = BFS(startstate); 
			break;
		}
		case 2: {
			result = DFS(startstate);
			break;
		}
		case 3: {
			result = UniCost(startstate);
			break;
		}
		default:
			break;
	}

	ofile.open("output.txt", fstream::out);

	//@to-do : Depending on choice it will call bfs, dfs or unicost
	//case : Call BFS , if success
	if (result) {

		//cout << endl << "Success" << endl;

		//intitialize temporary node to traverse solution path stack
		node top;

		//print expansion path.
		while (!expansion_path.empty()) {
			top = expansion_path.front();
			ofile << top.name;
			if (expansion_path.size() != 1)
				ofile << "-"; 
			expansion_path.pop();
		}

		ofile << endl;

		//print solution path.
		while (!soluton_path.empty()) {
			top = soluton_path.top();
			ofile << top.name;
			if (soluton_path.size() == 1)
				ofile << endl << top.cum_cost;
			else
				ofile << "-";
			soluton_path.pop();
		}

	}
	else {
		//node top;

		////print expansion path.
		//while (!expansion_path.empty()) {
		//	top = expansion_path.front();
		//	ofile << top.name;
		//	if (expansion_path.size() != 1)
		//		ofile << "-";
		//	expansion_path.pop();
		//}
		//
		//ofile << endl;

		ofile << "NoPathAvailable";// << endl;
	}


	ofile.close();

	

}
