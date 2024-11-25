#pragma once
#include <Model.h>
#include <queue>
#include <MathUtils.h>
#include <algorithm>
#include <cmath>
#include <tuple>

bool validCell( Map & map,  std::vector<int> & cell)
{
    return (cell[0] >= 0 && cell[0] < map.GetHeight() && cell[1] >= 0 && cell[1] < map.GetWidth());
}

using cell = std::vector<int>;
void searchBFS( Map & map,  Planner & planner, RobotData & data)
{
    
    int count {0};
    std::queue<cell> q;
    auto start = planner.GetStart();
    // TODO: validate if cell is valid later
    q.push(start);
    data.explored[start[0]][start[1]] = true;
    data.distance[start[0]][start[1]] = 0;
    data.iterations[start[0]][start[1]] = 0;
    data.parents[start[0]][start[1]] = {0,0};
   
    while (!q.empty())
    {
        
        auto current = q.front();
        q.pop();
        data.iterations[current[0]][current[1]] = count++;                
        
        if ( current == planner.GetGoal())
        {
            data.policy[current[0]][current[1]] = "*";
            return;
        }

        auto movements = planner.GetMovements();
        
        // Switching to a indexed for loop to iterate over the movements
        for ( int i = 0; i < movements.size(); ++i) 
        {
            auto next = current;
            auto movement = movements[i];
            next[0] += movement[0];
            next[1] += movement[1];
            if ( validCell(map,next) && !data.explored[next[0]][next[1]] && map[next[0]][next[1]] == 0) 
            {
                
                data.parents[next[0]][next[1]] = current;
                q.push(next);
                data.explored[next[0]][next[1]] = true;
                data.distance[next[0]][next[1]] = data.distance[current[0]][current[1]] + 1;
                

            }
        }
    
    }
}

void searchAStar( Map & map,  Planner & planner, RobotData & data)
{
    using cell = std::vector<int>;
    using priority_element = std::tuple<int,cell>;
    int count {0};
    std::priority_queue<priority_element, std::vector<priority_element>, std::greater<> > q;
    auto start = planner.GetStart();
    auto goal = planner.GetGoal();
    int heuristic = 0;

    
    q.push({0,start});
    data.explored[start[0]][start[1]] = true;
    data.distance[start[0]][start[1]] = 0;
    data.iterations[start[0]][start[1]] = 0;
    data.parents[start[0]][start[1]] = {0,0};
   
    while (!q.empty())
    {
        
        auto [current_cost, current] = q.top();
        q.pop();
        data.iterations[current[0]][current[1]] = count++;
                
        
        if ( current == goal)
        {   
            return;
        }

        auto movements = planner.GetMovements();
        // Switching to a indexed for loop to iterate over the movements
        for ( int i = 0; i < movements.size(); ++i) 
        {
            auto next = current;
            auto movement = movements[i];
            next[0] += movement[0];
            next[1] += movement[1];

            if ( validCell(map,next) && !data.explored[next[0]][next[1]] && map[next[0]][next[1]] == 0) 
            {
                
                data.parents[next[0]][next[1]] = current;
                
                data.explored[next[0]][next[1]] = true;
                heuristic =  planner.GetGoal()[0] - next[0] + planner.GetGoal()[1] - next[1]; 
                data.distance[next[0]][next[1]] = data.distance[current[0]][current[1]] + planner.GetCost();
                data.heuristicDistance[next[0]][next[1]] = heuristic + data.distance[next[0]][next[1]];
                q.push({data.heuristicDistance[next[0]][next[1]],next});   

            }
        }
    
    }
}

std::vector<std::vector<int>>  getPath(  const std::vector<std::vector<std::vector<int>>> &parents,  const std::vector<int> & source, const std::vector<int>   & goal)
{
    
    std::vector<std::vector<int>> path;
    std::vector<int> current = goal;
    std::vector<int> start = source;
    path.push_back(goal);
    while (current != start)
    {
        current = parents[current[0]][current[1]];
        path.push_back(current);
    }
    std::reverse(path.begin(),path.end());

    return path;
}


/** Compute the forward difference of the path and stores in the i-1 element */
std::vector<std::vector<int>> pathDifferential(const std::vector<std::vector<int>> & path)
{
    
    auto path_in_eigen = vector_to_matrix(path);
    auto path_foward_difference_eigen = forward_difference_matrix(path.size());
    auto forward_difference = matrix_to_vector(path_foward_difference_eigen * path_in_eigen);
    return forward_difference;
}


/* setPolicy */
void setPolicy (std::vector<std::vector<int> > & path,  Planner &plan, RobotData &rdata)
{
    auto forward_difference = pathDifferential(path);
    for (auto i = 0; i < path.size(); i++)
    {
        for (auto j = 0; j < path[i].size()-1; j++)
        {
          rdata.policy[path[i][0]][path[i][1]] = plan.GetMap()[forward_difference[i]][0] ;    
        }   
    }
    rdata.policy[plan.GetGoal()[0]][plan.GetGoal()[1]] = "*";

}
// JUDE