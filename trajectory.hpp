#include<bits/stdc++.h>
#include<constants.hpp>
using namespace std;

state motion(state temp, float velocity, float omega);
vector< state > calc_trajectory (state init, float velocity, float omega);