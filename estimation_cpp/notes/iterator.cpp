#include <vector>
#include <iostream>

using namespace std;

int main ()
{
    vector<int> estimator;

    for (int i =1; i <=10; i++)
    {
        estimator.push_back(i);
    }

    for (int j = 0; j<10; j++)
    {
        cout << estimator[j] << endl;
    }
}   

int main ()
{
    vector<int> predictedState;
    vector<int> curState;
    for (int i =1; i <=10; i++)
    {
        predictedState.push_back(i);
        curState.push_back(i+10)
        
    }

    for (int j = 0; j<10; j++)
    {
        cout << predictedState[j] << endl;


    }

}   