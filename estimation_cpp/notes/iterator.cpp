#include <vector>
#include <iostream>

using namespace std;

// :::: Variable definition
float dt;

int main ()
{
    vector<int> predictedState;
    vector<int> curState;
    
    for (int i =1; i <=10; i++)
    {
        predictedState.push_back(i);
        curState.push_back(i+10);
        dt = i;
        cout << i << endl;
    }

    for (int j = 0; j<10; j++)
    {
        cout << predictedState[j] << endl;
        predictedState[j] = curState[j] + dt * curState[j+3];
        cout << predictedState[j] << endl;
    }

}   