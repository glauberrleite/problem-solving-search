#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>

using namespace std;

template <typename S, typename A>
class Problem {
protected:
    S initial;
    set<S> goals;

public:
    Problem(S initial, set<S> goals) {
        this->initial = initial;
        this->goals = goals;
    }

    virtual bool isGoal(S state) = 0;

    virtual set<A> actions(S state = S()) = 0;

    virtual S result(S state, A action) = 0;

    virtual float actionCost(S state, A action, S reachead_state) = 0;

    virtual S getInitial() = 0;
};

template <typename S, typename A>
class Node {
protected:
    S state;
    A action;
    Node<S,A> * parent;
    float path_cost;
    float depth;
public:
    Node(S state, Node<S,A> * parent = nullptr, A action = A(), float path_cost = 0) {
        this->state = state;
        this->parent = parent;
        this->action = action;
        this->path_cost = path_cost;
        if (parent == nullptr) {
            this->depth = 0;
        } else {
            this->depth = parent->getDepth() + 1;
        }
    }

    vector<Node<S,A> *> expand(Problem<S,A> * problem) {
        vector<Node<S,A> *> children;
        for (A action : problem->actions(state)) {
            S next_state = problem->result(state, action);
            float cost = path_cost + problem->actionCost(state, action, next_state);
            
            children.push_back(new Node<S,A>(next_state, this, action, cost));
        }

        return children;
    }

    vector<A> solution_vec() {
        vector<A> solution;
        solution.push_back(this->action);
        
        Node<S,A> * node;
        if (this->parent != nullptr) {
            node = this->parent;
        }
        
        while (node->parent != nullptr) {
            solution.push_back(node->getAction());

            node = node->parent;
        }

        return vector<A>(solution.rbegin(), solution.rend()); // reverse the solution vector
    }

    Node<S,A> * getParent() {
        return parent;
    }

    S getState() {
        return state;
    }

    A getAction() {
        return action;
    }

    float getPathCost() {
        return path_cost;
    }

    float getDepth() {
        return depth;
    }
};

template <typename S, typename A>
Node<S,A> * bestFirstSearch(Problem<S,A> * problem, float (*f)(Node<S,A> *)) {
    Node<S,A> * root_node = new Node<S,A>(problem->getInitial());

    auto cmp = [f](Node<S, A>* a, Node<S, A>* b) {
        return f(a) > f(b);  // Min-heap based on function f
    };

    priority_queue<Node<S,A> *, vector<Node<S,A> *>, decltype(cmp)> frontier(cmp);
    frontier.push(root_node);

    map<S, Node<S,A> *> reached;
    reached[root_node->getState()] = root_node;

    while (frontier.empty() == false) {
        Node<S,A> * node = frontier.top();
        frontier.pop();

        if (problem->isGoal(node->getState())) {
            return node;
        }

        for (Node<S,A> * child : node->expand(problem)) {
            S child_state = child->getState();
            
            if ((reached.count(child_state) == 0) || (child->getPathCost() < reached[child_state]->getPathCost())) {
                reached[child_state] = child;
                frontier.push(child);
            }
        }
    }

    return nullptr;
}

enum Direction {
    NONE = -1,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

class GridProblem : public Problem<vector<int>,Direction> {
private:
    
    vector<vector<int>> world;
    vector<int> current_state;

public:
    
    GridProblem(vector<int> initial, set<vector<int>> goals) : Problem(initial, goals) {
       // World configuration
       world = {
           {0,1,0,0,0,0,0,0,0,0},
           {0,1,0,0,0,0,0,0,0,0},
           {0,0,0,0,0,0,0,0,0,0}
       };

       current_state = initial;
    }

    bool isGoal(vector<int> state) override {
        if (goals.count(state)) {
            return true;
        } else {
            return false;
        }
    }

    set<Direction> actions(vector<int> state = {}) override {
        if (state.empty()) {
            state = current_state;
        }

        set<Direction> possible_actions;
        
        // Lets start with all possibilities
        possible_actions.insert(UP);
        possible_actions.insert(DOWN);
        possible_actions.insert(LEFT);
        possible_actions.insert(RIGHT);

        // Check borders
        if ((state[0] <= 0) || (world[state[0] - 1][state[1]] == 1)) {
            possible_actions.erase(UP);
        }
        if ((state[0] >= world.size() - 1) || (world[state[0] + 1][state[1]] == 1)) {
            possible_actions.erase(DOWN);
        }
        if ((state[1] <= 0) || (world[state[0]][state[1] - 1] == 1)) {
            possible_actions.erase(LEFT);
        }
        if ((state[1] >= world[0].size() - 1) || (world[state[0]][state[1] + 1] == 1)) {
            possible_actions.erase(RIGHT);
        }

        return possible_actions;
    }

    vector<int> result(vector<int> state, Direction action) override {
        vector<int> new_state = state;
        switch (action) {
            case UP:
                new_state[0] -= 1;
                break;
            case DOWN:
                new_state[0] += 1;
                break;
            case LEFT:
                new_state[1] -= 1;
                break;
            case RIGHT:
                new_state[1] += 1;
                break;
            default:
                break;
        }
        
        return new_state;
    }

    float actionCost(vector<int> state, Direction action, vector<int> reached_state) override {
        return 1;
    }

    vector<int> getInitial() override {
        return initial;
    }

    void printWorld() {
        for (int i = 0; i < world.size(); i++) {

            cout << "[";

            for (int j = 0; j < world[i].size(); j++) {

                if ((i == current_state[0]) && (j == current_state[1])) {
                    cout << "\033[32mX\033[0m";
                } else {
                    cout << world[i][j];
                }

                if (j + 1 < world[i].size()) {
                    cout << ",";
                }

            }
            cout << "]" << endl;

        };
    }

    void actionInCurrent(Direction action) {
        switch (action) {
            case UP:
                current_state[0] -= 1;
                break;
            case DOWN:
                current_state[0] += 1;
                break;
            case LEFT:
                current_state[1] -= 1;
                break;
            case RIGHT:
                current_state[1] += 1;
                break;
            default:
                break;
        }

    }
};

float f(Node<vector<int>,Direction> * node) {
    return node->getPathCost();
}

int main() {
    vector<int> initial = {1,0};
    
    set<vector<int>> goals;
    vector<int> goal = {1,8};
    goals.insert(goal);

    GridProblem problem(initial, goals);

    problem.printWorld();

    Node<vector<int>,Direction> * solution = bestFirstSearch<vector<int>,Direction>(&problem, f);
    
    if (solution != nullptr) {
        cout << "---------------------" << endl;
        for (Direction action : solution->solution_vec()) {
            cout << action << endl;
            problem.actionInCurrent(action);
        
            problem.printWorld();

            cout << "---------------------" << endl;
        }
    }
    
    return 0;
}
