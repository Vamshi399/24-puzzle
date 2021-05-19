////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Use A* algorithm to solve 24 digital problems
//
// Created by Vamshi @2021-01-12
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <new>
#include <iomanip>
#include <utility>
#include <vector>
#include <fstream>

#include <ctype.h>

using namespace std;

// Configuration
#define NUM_TIMES_TO_RUN_SEARCH 1
#define DISPLAY_SOLUTION_FORWARDS 1		// Show forward steps
#define DISPLAY_SOLUTION_BACKWARDS 0	// Show backward steps
#define DISPLAY_SOLUTION_INFO 1
#define DEBUG_LISTS 0					// display DEBUG List
#define SPIRAL 0	// Target status: spiral
#define NORMAL 1 	// Target status: Normal

// Astar search class
#include "stlastar.h" // Check the header file to get copyright information and usage information

// Global variable
#define BOARD_WIDTH   (5)
#define BOARD_HEIGHT  (5)
#define GM_TILE       (-1)
#define GM_SPACE	  (0)
#define GM_OFF_BOARD  (1)

vector<pair<int, int> > Move;

// definition

// Define the following function to use the Astar search class

// data
//		Your own state space information
// function
//		(Optional) Constructor.
//		Nodes are created by the user, so whether you use a
//      constructor with parameters as below, or just set the object up after the 
//      constructor, is up to you.
//
//		(Optional) Destructor. 
//		The destructor will be called if you create one. You 
//		can rely on the default constructor unless you dynamically allocate something in
//		your data
//
//		float GoalDistanceEstimate( PuzzleState &nodeGoal );
//		Return the estimated cost to goal from this node (pass reference to goal node)
//
//		bool IsGoal( PuzzleState &nodeGoal );
//		Return true if this node is the goal.
//
//		bool GetSuccessors( AStarSearch<PuzzleState> *astarsearch );
//		For each successor to this state call the AStarSearch's AddSuccessor call to 
//		add each one to the current search - return false if you are out of memory and the search
//		will fail
//
//		float GetCost( PuzzleState *successor );
//		Return the cost moving from this state to the state of successor
//
//		bool IsSameState( PuzzleState &rhs );
//		Return true if the provided state is the same as this state

// Here the example is the 24-puzzle state ...
// 24-Digital state
class PuzzleState
{

public:

	// definition
	typedef enum
	{
		TL_SPACE,   // 0
		TL_1, 
		TL_2,  
		TL_3, 
		TL_4, 
		TL_5, 
		TL_6, 
		TL_7, 
		TL_8,
        TL_9,
        TL_10,
        TL_11,
        TL_12,
        TL_13,
        TL_14,
        TL_15,
        TL_16,
        TL_17,
        TL_18,
        TL_19,
        TL_20,
        TL_21,
        TL_22,
        TL_23,
        TL_24,
	} TILE;

	// data
	static TILE g_goal[ BOARD_WIDTH*BOARD_HEIGHT];      // Goal state
	static TILE g_start[ BOARD_WIDTH*BOARD_HEIGHT];     // Initial state

	// the tile data for the 24-puzzle
    // 24-Digital block data
	TILE tiles[ BOARD_WIDTH*BOARD_HEIGHT ];

	// Member function

	PuzzleState() {  
						memcpy( tiles, g_goal, sizeof( TILE ) * BOARD_WIDTH * BOARD_HEIGHT );			
					}

	PuzzleState( TILE *param_tiles ) 
					{
						memcpy( tiles, param_tiles, sizeof( TILE ) * BOARD_WIDTH * BOARD_HEIGHT );			
					}

	float GoalDistanceEstimate( PuzzleState &nodeGoal );
	bool IsGoal( PuzzleState &nodeGoal );
	bool GetSuccessors( AStarSearch<PuzzleState> *astarsearch, PuzzleState *parent_node );
	float GetCost( PuzzleState &successor );
	bool IsSameState( PuzzleState &rhs );
	
	void PrintNodeInfo();
	void WriteNodeInfo();
	void WriteDream();

private:
	// User stuff - Just add what you need to help you write the above functions...
    // User stuff-feel free to add the functions you need to achieve your functions

	void GetSpacePosition( PuzzleState *pn, int *rx, int *ry );
	bool LegalMove( TILE *StartTiles, TILE *TargetTiles, int spx, int spy, int tx, int ty );
	int GetMap( int x, int y, TILE *tiles );

};


// Goal state
PuzzleState::TILE PuzzleState::g_goal[] = 
{
	
#if SPIRAL	// spiral
	TL_1,  TL_2,  TL_3,  TL_4,  TL_5,
    TL_16, TL_17, TL_18, TL_19, TL_6, 
    TL_15, TL_24, TL_SPACE, TL_20, TL_7, 
    TL_14, TL_23, TL_22, TL_21, TL_8, 
    TL_13, TL_12, TL_11, TL_10, TL_9
	
#elif  NORMAL	// ordinary
	TL_1, TL_2, TL_3, TL_4, TL_5,
	TL_6, TL_7, TL_8, TL_9, TL_10,
	TL_11,TL_12, TL_13,TL_14, TL_15,
	TL_16,TL_17, TL_18, TL_19, TL_20,
	TL_21, TL_22, TL_23, TL_24, TL_SPACE
	
#endif

};

// 一Some good initial state
PuzzleState::TILE PuzzleState::g_start[] = 
{

#if 0
	// spiral a - 2 steps 
	TL_1,  TL_2,  TL_3,  TL_4,  TL_5,
    TL_16, TL_17, TL_18, TL_19, TL_6, 
    TL_15, TL_24, TL_20, TL_7, TL_SPACE,
    TL_14, TL_23, TL_22, TL_21, TL_8, 
    TL_13, TL_12, TL_11, TL_10, TL_9

#elif 0
	// spiral b -  4 steps
	TL_1,  TL_2,  TL_3,  TL_4,  TL_5,
    TL_16, TL_17, TL_18, TL_19, TL_6, 
    TL_15, TL_24, TL_20, TL_7, TL_8,
    TL_14, TL_23, TL_22, TL_21, TL_9, 
    TL_13, TL_12, TL_11, TL_10, TL_SPACE


#elif 0
	// spiral c -  52 steps
	TL_1,  TL_3,  TL_4,  TL_19, TL_5,
	TL_16, TL_2,  TL_18, TL_6,  TL_7,
	TL_24, TL_17, TL_20, TL_21, TL_8,
	TL_15, TL_14, TL_22, TL_11, TL_10,
	TL_13, TL_23, TL_12, TL_9,  TL_SPACE


#elif 0
	// normal a - 2 steps
	TL_1, TL_2, TL_3, TL_4, TL_5,
	TL_6, TL_7, TL_8, TL_9, TL_10,
	TL_11,TL_12, TL_13,TL_14, TL_15,
	TL_16,TL_17, TL_18, TL_19, TL_20,
	TL_21, TL_22, TL_SPACE, TL_23, TL_24

#elif 0
	// normal b - 8 steps
	TL_1, TL_3, TL_SPACE, TL_4, TL_5,
	TL_6, TL_2, TL_8, TL_9, TL_10,
	TL_11,TL_7, TL_12,TL_14, TL_15,
	TL_16,TL_17, TL_13, TL_19, TL_20,
	TL_21, TL_22, TL_18, TL_23, TL_24

#elif 0
	// normal c - 30 steps
	TL_SPACE, TL_2, TL_8, TL_4, TL_5,
	TL_3, TL_1, TL_9, TL_10, TL_15,
	TL_6,TL_7, TL_13,TL_14, TL_20,
	TL_12,TL_22, TL_16, TL_17, TL_24,
	TL_11, TL_21, TL_18, TL_19, TL_23
	
#elif 1
	// normal d - 38 steps 
	TL_2, TL_8, TL_4, TL_5, TL_15,
	TL_3, TL_1, TL_9, TL_10, TL_20,
	TL_6,TL_7, TL_SPACE,TL_13, TL_14,
	TL_12,TL_22, TL_16, TL_17, TL_24,
	TL_11, TL_21, TL_18, TL_19, TL_23

#endif  

};

// Determine whether the board status is the same
bool PuzzleState::IsSameState( PuzzleState &rhs )
{

	for( int i=0; i<(BOARD_HEIGHT*BOARD_WIDTH); i++ )
	{
		if( tiles[i] != rhs.tiles[i] )
		{
			return false;
		}
	}

	return true;

}


// Print node information The entire chessboard is a "node"
void PuzzleState::PrintNodeInfo()
{
	cout<<setiosflags(ios::left);
	cout<<setw(3)<<tiles[0]<<setw(3)<<tiles[1]<<setw(3)<<tiles[2]<<setw(3)<<tiles[3]<<setw(3)<<tiles[4]<<endl;
	cout<<setw(3)<<tiles[5]<<setw(3)<<tiles[6]<<setw(3)<<tiles[7]<<setw(3)<<tiles[8]<<setw(3)<<tiles[9]<<endl;
	cout<<setw(3)<<tiles[10]<<setw(3)<<tiles[11]<<setw(3)<<tiles[12]<<setw(3)<<tiles[13]<<setw(3)<<tiles[14]<<endl;
	cout<<setw(3)<<tiles[15]<<setw(3)<<tiles[16]<<setw(3)<<tiles[17]<<setw(3)<<tiles[18]<<setw(3)<<tiles[19]<<endl;
	cout<<setw(3)<<tiles[20]<<setw(3)<<tiles[21]<<setw(3)<<tiles[22]<<setw(3)<<tiles[23]<<setw(3)<<tiles[24]<<endl;
	
	int x,y;

	for( y=0; y<BOARD_HEIGHT; y++ )
	{
		for( x=0; x<BOARD_WIDTH; x++ )
		{
			if( tiles[(y*BOARD_WIDTH)+x] == TL_SPACE )
			{
				// Record the movement path of the blank grid
				Move.push_back(make_pair(y, x));
			}
		}
	}
	
}


ofstream out("data.txt");

// Output node information to file
void PuzzleState::WriteNodeInfo()
{
	out<<tiles[0]<<" "<<tiles[1]<<" "<<tiles[2]<<" "<<tiles[3]<<" "<<tiles[4]<<" ";
	out<<tiles[5]<<" "<<tiles[6]<<" "<<tiles[7]<<" "<<tiles[8]<<" "<<tiles[9]<<" ";
	out<<tiles[10]<<" "<<tiles[11]<<" "<<tiles[12]<<" "<<tiles[13]<<" "<<tiles[14]<<" ";
	out<<tiles[15]<<" "<<tiles[16]<<" "<<tiles[17]<<" "<<tiles[18]<<" "<<tiles[19]<<" ";
	out<<tiles[20]<<" "<<tiles[21]<<" "<<tiles[22]<<" "<<tiles[23]<<" "<<tiles[24]<<endl;
}

// Output normal target status
void PuzzleState::WriteDream()
{
	for(int i=0; i<BOARD_WIDTH * BOARD_HEIGHT; i++)
	{
		out<<this->g_goal[i]<<" ";
	}
	out<<endl;
}


// Here's the heuristic function that estimates the distance from a PuzzleState
// to the Goal. 
// This is a heuristic function used to estimate the distance between the current board state and the target state, and returns the heuristic score h of the current node

float PuzzleState::GoalDistanceEstimate( PuzzleState &nodeGoal )
{

#if SPIRAL // Spiral heuristic score

	// Nilsson's sequence score Nielsen serial score

	int i, cx, cy, ax, ay;	// i: iterator cx,cy: target position of the block ax,ay: current position of the block
	int h = 0;	// Heuristic score
	int s, t;	// s score t score

	// given a tile this returns the tile that should be clockwise
	// Given a square, return the first square in the clockwise direction
	TILE correct_follower_to[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/*
			1  2  3  4  5
			16 17 18 19 6
			15 24    20 7
			14 23 22 21 8
			13 12 11 10 9
		*/
		/* TL_SPACE */ TL_SPACE, // Spaces will make mistakes, spaces are not considered	
		/* TL_1 */ TL_2,
		/* TL_2 */ TL_3,
		/* TL_3 */ TL_4,
		/* TL_4 */ TL_5,
		/* TL_5 */ TL_6,
		/* TL_6 */ TL_7,
		/* TL_7 */ TL_8,
		/* TL_8 */ TL_9,
		/* TL_9 */ TL_10,
		/* TL_10 */ TL_11,
		/* TL_11 */ TL_12,
		/* TL_12 */ TL_13,
		/* TL_13 */ TL_14,
		/* TL_14 */ TL_15,
		/* TL_15 */ TL_16,
		/* TL_16 */ TL_1,
		/* TL_17 */ TL_18,
		/* TL_18 */ TL_19,
		/* TL_19 */ TL_20,
		/* TL_20 */ TL_21,
		/* TL_21 */ TL_22,
		/* TL_22 */ TL_23,
		/* TL_23 */ TL_24,
		/* TL_24 */ TL_17
	};

	// given a table index returns the index of the tile that is clockwise to it 3*3 only
	// Given the index of a table, return the index of the first square in the clockwise direction
	int clockwise_tile_of[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/*
			0  1  2  3  4
			5  6  7  8  9
			10 11 12 13 14
			15 16 17 18 19
			20 21 22 23 24
		*/
		1,	// 0
		2,  // 1	  
		3,	// 2   
		4,  // 3
		9,	// 4
		0,	// 5
		7,	// 6
		8,	// 7
		13,	// 8
		14,	// 9
		5,	// 10
		6,	// 11
		-1,	// 12 The center square will not be called
		18,	// 13
		19,	// 14
		10,	// 15
		11,	// 16
		16,	// 17
		17,	// 18
		24,	// 19
		15,	// 20
		20,	// 21
		21,	// 22
		22,	// 23
		23	// 24
	};

	// The x coordinate of the target position of the block
	int tile_x[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/* TL_SPACE */ 2,
		/* TL_1 */ 0,    
		/* TL_2 */ 1,    
		/* TL_3 */ 2,    
		/* TL_4 */ 4,    
		/* TL_5 */ 2,    
		/* TL_6 */ 4,    
		/* TL_7 */ 4,    
		/* TL_8 */ 4,    
		/* TL_9 */ 4,
		/* TL_10 */ 3,
		/* TL_11 */ 2,
		/* TL_12 */ 1,
		/* TL_13 */ 0,
		/* TL_14 */ 0,
		/* TL_15 */ 0,
		/* TL_16 */ 0,
		/* TL_17 */ 1,
		/* TL_18 */ 2,
		/* TL_19 */ 3,
		/* TL_20 */ 3,
		/* TL_21 */ 3,
		/* TL_22 */ 2,
		/* TL_23 */ 1,
		/* TL_24 */ 1
	};

	// The y coordinate of the target position of the block
	int tile_y[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/* TL_SPACE */ 2,	
		/* TL_1 */ 0,
		/* TL_2 */ 0,
		/* TL_3 */ 0,
		/* TL_4 */ 0,
		/* TL_5 */ 0,
		/* TL_6 */ 1,
		/* TL_7 */ 2,
		/* TL_8 */ 3,
		/* TL_9 */ 4,
		/* TL_10 */ 4,
		/* TL_11 */ 4,
		/* TL_12 */ 4,
		/* TL_13 */ 4,
		/* TL_14 */ 3,
		/* TL_15 */ 2,
		/* TL_16 */ 1,
		/* TL_17 */ 1,
		/* TL_18 */ 1,
		/* TL_19 */ 1,
		/* TL_20 */ 2,
		/* TL_21 */ 3,
		/* TL_22 */ 3,
		/* TL_23 */ 3,
		/* TL_24 */ 2
	};

	s=0;
	
	// score 1 point if centre is not correct 
	// If the center of the current board and the center of the target state do not match, 1 point
	if( tiles[(BOARD_HEIGHT*BOARD_WIDTH)/2] != nodeGoal.tiles[(BOARD_HEIGHT*BOARD_WIDTH)/2] )
	{
 		s = 1;
	}

	for( i=0; i<(BOARD_HEIGHT*BOARD_WIDTH); i++ )
	{
		// this loop adds up the totaldist element in h and
		// the sequence score in s

		// the space does not count Spaces don't count
		if( tiles[i] == TL_SPACE )
		{
			continue;
		}

		// get correct x and y of this tile	
		// Get the target position of the block
		cx = tile_x[tiles[i]];
		cy = tile_y[tiles[i]];

		// get actual	
		ax = i % BOARD_WIDTH;
		ay = i / BOARD_WIDTH;

		// add manhatten distance to h
		// Calculate the manhatten distance and add it to h
		h += abs( cx-ax );
		h += abs( cy-ay );

		// no s score for center tile
		// The center square does not need to calculate the s score
		if( (ax == (BOARD_WIDTH/2)) && (ay == (BOARD_HEIGHT/2)) )
		{
			continue;
		}

		// score 2 points if not followed by successor
		// If the first block in the clockwise direction of the current block is not the first block in the clockwise direction in the target state, then s gets 2 points
		if( correct_follower_to[ tiles[i] ] != tiles[ clockwise_tile_of[ i ] ] )
		{
			// clockwise_tile_of[i]Is the first block in the clockwise direction of the block with index i
			s += 2;
		}
	}

	// mult by 3 and add to h
	// multiply s by 3 and add h
	t = h + (3*s);
	
	return (float) t;	// Returns the heuristic score of the current node h


	
#elif NORMAL	// Ordinary heuristic function

	int i, cx, cy, ax, ay;	// i: iterator cx,cy: target position of the block ax,ay: current position of the block
	int h = 0;	// Heuristic score
	
	// The x coordinate of the target position of the block
	int tile_x[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/* TL_SPACE */ 4,
		/* TL_1 */ 0,    
		/* TL_2 */ 1,    
		/* TL_3 */ 2,    
		/* TL_4 */ 3,    
		/* TL_5 */ 4,    
		/* TL_6 */ 0,    
		/* TL_7 */ 1,    
		/* TL_8 */ 2,    
		/* TL_9 */ 3,
		/* TL_10 */ 4,
		/* TL_11 */ 0,
		/* TL_12 */ 1,
		/* TL_13 */ 2,
		/* TL_14 */ 3,
		/* TL_15 */ 4,
		/* TL_16 */ 0,
		/* TL_17 */ 1,
		/* TL_18 */ 2,
		/* TL_19 */ 3,
		/* TL_20 */ 4,
		/* TL_21 */ 0,
		/* TL_22 */ 1,
		/* TL_23 */ 2,
		/* TL_24 */ 3
	};

	// The y coordinate of the target position of the block
	int tile_y[ BOARD_WIDTH * BOARD_HEIGHT ] =
	{
		/* TL_SPACE */ 4,
		/* TL_1 */ 0,    
		/* TL_2 */ 0,    
		/* TL_3 */ 0,    
		/* TL_4 */ 0,    
		/* TL_5 */ 0,    
		/* TL_6 */ 1,    
		/* TL_7 */ 1,    
		/* TL_8 */ 1,    
		/* TL_9 */ 1,
		/* TL_10 */ 1,
		/* TL_11 */ 2,
		/* TL_12 */ 2,
		/* TL_13 */ 2,
		/* TL_14 */ 2,
		/* TL_15 */ 2,
		/* TL_16 */ 3,
		/* TL_17 */ 3,
		/* TL_18 */ 3,
		/* TL_19 */ 3,
		/* TL_20 */ 3,
		/* TL_21 */ 4,
		/* TL_22 */ 4,
		/* TL_23 */ 4,
		/* TL_24 */ 4
	};
	
	for( i=0; i<(BOARD_HEIGHT*BOARD_WIDTH); i++ )
	{
		// Cumulative heuristic score

		// get correct x and y of this tile	
		// Get the target position of the block
		cx = tile_x[tiles[i]];
		cy = tile_y[tiles[i]];

		// get actual	
		ax = i % BOARD_WIDTH;
		ay = i / BOARD_WIDTH;

		// add manhatten distance to h
		// Calculate the manhatten distance and add it to h
		h += abs( cx-ax );
		h += abs( cy-ay );
	}
	
	return (float)h;
	
#endif
}


// Determine whether the current board state has reached the target state
bool PuzzleState::IsGoal( PuzzleState &nodeGoal )
{
	return IsSameState( nodeGoal );
}

// Helper
// Return the x and y position of the space tile
// Return the position of the blank square
void PuzzleState::GetSpacePosition( PuzzleState *pn, int *rx, int *ry )
{
	int x,y;

	for( y=0; y<BOARD_HEIGHT; y++ )
	{
		for( x=0; x<BOARD_WIDTH; x++ )
		{
			if( pn->tiles[(y*BOARD_WIDTH)+x] == TL_SPACE )
			{
				*rx = x;
				*ry = y;

				return;
			}
		}
	}


	assert( false && "Something went wrong. There's no space on the board" );

}

int PuzzleState::GetMap( int x, int y, TILE *tiles )
{

	if( x < 0 ||
	    x >= BOARD_WIDTH ||
		 y < 0 ||
		 y >= BOARD_HEIGHT
	  )
		return GM_OFF_BOARD;	 

	if( tiles[(y*BOARD_WIDTH)+x] == TL_SPACE )
	{
		return GM_SPACE;
	}

	return GM_TILE;
}

// Given a node set of tiles and a set of tiles to move them into, do the move as if it was on a tile board
// note : returns false if the board wasn't changed, and simply returns the tiles as they were in the target
// spx and spy is the space position while tx and ty is the target move from position

bool PuzzleState::LegalMove( TILE *StartTiles, TILE *TargetTiles, int spx, int spy, int tx, int ty )
{

	int t;
	
	if( GetMap( spx, spy, StartTiles ) == GM_SPACE )
	{
		if( GetMap( tx, ty, StartTiles ) == GM_TILE )
		{

			// Copy tiles
			for( t=0; t<(BOARD_HEIGHT*BOARD_WIDTH); t++ )
			{
				TargetTiles[t] = StartTiles[t];
			}

			TargetTiles[ (ty*BOARD_WIDTH)+tx ] = StartTiles[ (spy*BOARD_WIDTH)+spx ];
			TargetTiles[ (spy*BOARD_WIDTH)+spx ] = StartTiles[ (ty*BOARD_WIDTH)+tx ];
			
			return true;
		}
	}
	return false;

}

// This generates the successors to the given PuzzleState. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool PuzzleState::GetSuccessors( AStarSearch<PuzzleState> *astarsearch, PuzzleState *parent_node )
{
	PuzzleState NewNode;

	int sp_x,sp_y;

	GetSpacePosition( this, &sp_x, &sp_y );

	bool ret;

	if( LegalMove( tiles, NewNode.tiles, sp_x, sp_y, sp_x, sp_y-1 ) == true )
	{
		ret = astarsearch->AddSuccessor( NewNode );
		
		if( !ret ) return false;
	}

	if( LegalMove( tiles, NewNode.tiles, sp_x, sp_y, sp_x, sp_y+1 ) == true )
	{
		ret = astarsearch->AddSuccessor( NewNode );
		
		if( !ret ) return false;
	}

	if( LegalMove( tiles, NewNode.tiles, sp_x, sp_y, sp_x-1, sp_y ) == true )
	{
		ret = astarsearch->AddSuccessor( NewNode );

		if( !ret ) return false;
	}

	if( LegalMove( tiles, NewNode.tiles, sp_x, sp_y, sp_x+1, sp_y ) == true )
	{
		ret = astarsearch->AddSuccessor( NewNode );

		if( !ret ) return false;
	}

	return true; 
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving
float PuzzleState::GetCost( PuzzleState &successor )
{
	return 1.0f; // I love it when life is simple

}


// Main

int main( int argc, char *argv[] )
{	
	cout << "A* 24-puzzle solver implementation\n@2016";

	if( argc > 1 )
	{
		int i = 0;
		int c;

		while( (c = argv[1][i]) )
		{
			if( isdigit( c ) )
			{
				int num = (c - '0');

				PuzzleState::g_start[i] = static_cast<PuzzleState::TILE>(num);
				
			}
		
			i++;
		}


	}

	// Create an instance of the search class...
	// Create an instance of the search class

	AStarSearch<PuzzleState> astarsearch;

	int NumTimesToSearch = NUM_TIMES_TO_RUN_SEARCH;	// Number of searches run: 1 time

	while( NumTimesToSearch-- )
	{

		// Create initial state
		PuzzleState nodeStart( PuzzleState::g_start );

		// Define target state
		PuzzleState nodeEnd( PuzzleState::g_goal );

		// Set initial state and target state
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;

		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

#if DEBUG_LISTS

			float f,g,h;
			
			cout << "Search steps: " << SearchSteps << endl;

			cout << "Open:\n";
			PuzzleState *p = astarsearch.GetOpenListStart( f,g,h );
			while( p )
			{
				((PuzzleState *)p)->PrintNodeInfo();
				cout << "f: " << f << " g: " << g << " h: " << h << "\n\n";
				
				p = astarsearch.GetOpenListNext( f,g,h );
				
			}

			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart( f,g,h );
			while( p )
			{
				p->PrintNodeInfo();
				cout << "f: " << f << " g: " << g << " h: " << h << "\n\n";
				
				p = astarsearch.GetClosedListNext( f,g,h );
			}

#endif

// Test cancel search
#if 0
			int StepCount = astarsearch.GetStepCount();
			if( StepCount == 10 )
			{
				astarsearch.CancelSearch();
			}
#endif
			SearchSteps++;
		}
		while( SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_SUCCEEDED )
		{
#if DISPLAY_SOLUTION_FORWARDS	// Show forward steps
			cout << "The search begins!\n\n";
#endif
			PuzzleState *node = astarsearch.GetSolutionStart();

			int steps = 0;

#if DISPLAY_SOLUTION_FORWARDS	// Show forward steps
			cout<<"Initial state:"<<endl;
			node->PrintNodeInfo();	// Output to terminal
			cout<<endl;
			
			node->WriteDream();		// Output target state
			node->WriteNodeInfo();	// Output initial state
			
#endif
			cout << "Forward steps:\n";
			// Search for the next node
			for( ;; )
			{
				
				node = astarsearch.GetSolutionNext();

				if( !node )
				{
					break;
				}

#if DISPLAY_SOLUTION_FORWARDS	// Show forward steps
				node->PrintNodeInfo();
				cout << endl;
#endif
				steps ++;
			
			};

#if DISPLAY_SOLUTION_FORWARDS	// Show forward steps
			cout << "Step count: " << steps << endl<<endl;
#endif

////////////
			cout<<"Final state:"<<endl;
			node = astarsearch.GetSolutionEnd();
			
			
			
			
#if DISPLAY_SOLUTION_BACKWARDS	// Show forward steps
			cout << "Show forward steps：\n";
#endif
			steps = 0;

			node->PrintNodeInfo();
			cout << endl;
			for( ;; )
			{
				node = astarsearch.GetSolutionPrev();

				if( !node )
				{
					break;
				}
#if DISPLAY_SOLUTION_BACKWARDS	// Show forward steps
				node->PrintNodeInfo();
                cout << endl;
#endif
				steps ++;
			
			};

#if DISPLAY_SOLUTION_BACKWARDS	// Show forward steps
			cout << "Step count: " << steps << endl;
#endif

//////////////

			// Once you're done with the solution you can free the nodes up
			astarsearch.FreeSolutionNodes();
		
		}
		else if( SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_FAILED ) 
		{
#if DISPLAY_SOLUTION_INFO
			cout << "Search terminated! No goal status found!\n";
#endif		
		}
		else if( SearchState == AStarSearch<PuzzleState>::SEARCH_STATE_OUT_OF_MEMORY ) 
		{
#if DISPLAY_SOLUTION_INFO
			cout << "Search terminated! Out of memory!\n";
#endif		
		}

// Display the number of loops the search went through
// #if DISPLAY_SOLUTION_INFO
// 		cout << "Search times: " << astarsearch.GetStepCount() << endl;
// #endif

			// Join the end
			Move.push_back(make_pair(2,2));
			
			int len_Move = Move.size();
			
			
			// Output blank grid motion path
			for(int i=0; i<len_Move-3; i++)
			{
				int r1 = Move[i].first;
				int c1 = Move[i].second;
				int r2 = Move[i+1].first;
				int c2 = Move[i+1].second;
				
				// cout<<"Th"<<i<<" move:("<<r1<<","<<c1<<") -> ("<<r2<<","<<c2<<")"<<endl;
				out<<r2<<" "<<c2<<" "<<r1<<" "<<c1<<endl;
			}
	}

	return 0;
}


