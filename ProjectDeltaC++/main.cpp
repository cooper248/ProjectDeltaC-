//
//  main.cpp
//  ProjectDeltaC++
//
//  Created by Cooper Richardson on 4/25/17.
//  Copyright © 2017 Cooper Richardson. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <assert.h>
#include <random>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <time.h>
#include "LY_NN.h"

#define crRand (double)rand()/RAND_MAX
using namespace std;

// Create global constants
const int popSize = 100;
const int nnHidden = 5;
const int nnOut = 1;
const int nnIn = 1;
const double pi = 3.14159265359;
const double dT = .2;
const int velocity = 3;
const int T = 5;

// Create class - policy
class policy{
public:
    vector<double> weights;
    int fitness;
    vector<double> mutatePolicy(vector<double>);
    void initPolicy(vector<double>);
};

class simulate{
public:
    double boatStartX;
    double boatStartY;
    double boatXLoc;
    double boatYLoc;
    double boatXLocmin1;
    double boatYLocmin1;
    double goalTopYLoc;
    double goalBottomYLoc;
    double goalTopXLoc;
    double goalBottomXLoc;
    
    double thetaStart;
    double theta;
    double omega;
    double omegaStart;
    
    double u;
    double timeInSim;
    int xResult;
    int yResult;
    int timeResult;
    
    //double evalBeta(); //angle from boat to goal (center to center)
    //double evalProgress(); //magnitude of velocity component towards goal center
    void evalXLoc(simulate*);
    void evalYLoc(simulate*);
    void evalNewTheta(simulate*);
    void evalNewOmega(simulate*);
    int evalFitness(simulate*);  //evaluates and assigns fitness based off of full simulation - eventually will add use of time spent facing goal
    void initSim();
    void reset(simulate*,int);
    void simData(simulate*boat);
};

// Initialize population
vector<policy> initPop(neural_network* NN){
    
    vector<policy> population;
    for(int i = 0; i<popSize ; i++)
    {
        policy A;
        for(int j =0; j< NN->get_number_of_weights(); j++ )
        {
            double weight = crRand - crRand;
            A.weights.push_back(weight);
        }
        assert( A.weights.size() == NN->get_number_of_weights() );
        A.fitness = -100000;
        population.push_back(A);
    }
    assert( population.size() == popSize );
    return population;

}

// Initialize program
void simulate::initSim(){
    srand(time(NULL));

    //boatStartX = 600 + rand()% 200; //place boat anywhere from 250 to 750
    boatStartX = 550;
    boatXLoc= boatStartX;
    boatXLocmin1 = boatXLoc;
    //boatStartY = 600 + rand()% 200; //place boat anywhere from 250 to 750
    boatStartY = 750;
    boatYLoc= boatStartY;
    boatYLocmin1 = boatYLoc;
    goalTopXLoc = 950;
    goalTopYLoc = 1000;
    goalBottomXLoc = 950;
    goalBottomYLoc = 0;
    
    if(rand()% 2 == 0)
    {
        thetaStart = (-2*pi)*crRand;
    }
    else
    {
        thetaStart = (2*pi)*crRand;
    }
    
    theta = thetaStart;
    timeInSim = 0;
    
    if(rand()% 2 == 0)
    {
        omegaStart = (rand()% 15)*(-pi/180);
    }
    else
    {
        omegaStart = (rand()% 15)*(pi/180);
    }
    
    omega = omegaStart;
    cout<<"Boat x: "<< boatXLoc<<" Boat Y: "<<boatYLoc<<" Theta: "<<theta<<" Omega: "<<omega<<endl;
};

void simulate::evalXLoc(simulate *boat){
    boat->boatXLoc = boat->boatXLocmin1 + (cos(boat->theta) * velocity * dT);
    boat->xResult = 5;
    
    // out of bounds Check =3
    if(boat->boatXLoc >=1000 || boat->boatXLoc<= 0){
        boat->xResult = 1;
        //cout<<" * * * * * * * * * * * * * * * * * * OUT OF BOUNDS  * * * * * * * * * * * * * * * * * * * * * * *"<<endl;
    }
    
    // FINDS GOAL CHECK = 2
    if(boat->boatXLoc > 950 && boat->boatXLocmin1 < 950){
        boat->xResult = 2;
        //cout<<"GOAL WAS FOUND!!! -----------------------------------------------------------------------------------"<<endl;
    }
    
    if(boat->timeInSim > 10000){
        boat->xResult = 4;
        //cout<<"TOO MUCH TIME!!! *************************************************************************************"<<endl;
    }
    if(boat->xResult ==5){
        boat->timeInSim = boat->timeInSim + 1;
        boat->xResult = 3;
    }
    boat->boatXLocmin1 = boat->boatXLoc;
    assert(boat->xResult != 5);
}

void simulate::evalYLoc(simulate *boat){
    boat->boatYLoc = boat->boatYLocmin1 + (sin(boat->theta) * velocity * dT);
    boat->boatYLocmin1 = boat->boatYLoc;
    
    if(boat->boatYLoc >=1000 || boat->boatYLoc<= 0){
        boat->yResult = 1;
    }
    else{
        boat->yResult = 2;
    }
    assert(boat->yResult != 5);
}

void simulate::evalNewOmega(simulate* boat){
    
    boat->omega = boat->omega + (boat->u - boat->omega)*(dT/T);
}

void simulate::evalNewTheta(simulate* boat)
{
    boat->theta = boat->theta + (boat->omega * dT);
    if(boat->theta > 2*pi)
    {
        boat->theta = boat->theta - (2*pi);
    }
    else if(boat->theta<= -2*pi)
    {
        boat->theta = boat->theta + (2*pi);
    }
}

int simulate::evalFitness(simulate* boat){
    int fitness = -1;
    if(boat->xResult == 1 || boat->yResult == 1){
        fitness = fitness - boat->timeInSim - (1000-boat->boatXLocmin1)-(1000-boat->boatYLocmin1);
    }
    else if(boat->xResult == 2){
        fitness =  100000 - boat->timeInSim;
    }
    else if(boat->xResult == 4){
        fitness = fitness - 100000;
    }
    return fitness;
}

int promptUserStart(){
    int numGen;
    cout<<"Are you ready for this...?"<<endl<<"How many generations would you like to have run?: ";
    cin>>numGen;
    cout<<endl;
    return numGen;
}

void simulate::reset(simulate* boat,int check){
    
    if(check == 1)
    {
        boat->boatStartX = 20;
        boat->boatStartY = 950;
    }
    
    boat->boatXLoc = boat->boatStartX;
    boat->boatYLoc = boat->boatStartY;
    boat->timeInSim = 0;
    boat->omega = boat->omegaStart;
    boat->theta = boat->thetaStart;
    boat->xResult = 5;
    boat->yResult = 5;
    boat->boatYLocmin1 = boat->boatStartY +crRand;
    boat->boatXLocmin1 = boat->boatStartX +crRand;
    //cout<<"Boat x: "<< boat->boatXLoc<<"Boat Y: "<<boat->boatYLoc<<"Theta: "<<boat->theta<<"Omega: "<<boat->omega<<endl;
    //cout<<"------------------------------------- BOAT RESET -------------------------------------"<<endl;
}

void simulate::simData(simulate* boat){
    evalXLoc(boat);
    evalYLoc(boat);
    evalNewOmega(boat);
    evalNewTheta(boat);
}

vector<double> policy::mutatePolicy(vector<double> weights){
    vector<double> mutPol;
    mutPol = weights;
    int numWeights = weights.size();
    int numChanges = numWeights *.5;
    int mutateLoc;
    
    for(int i =0; i < numChanges; i++)
    {
        mutateLoc = rand()% numWeights;
        if(rand()% 2 == 0){
        mutPol.at(mutateLoc) = mutPol.at(mutateLoc) + (crRand*.2);
        }
        else
        {
        mutPol.at(mutateLoc) = mutPol.at(mutateLoc) - (crRand*.2);
        }
    }
    return mutPol;
}

vector<policy> downSelect(vector<policy>* mutatedPopulation){
    vector<policy> population;
    population.clear();
    assert(population.size() == 0);
    
    int max = -1000000;
    int maxLoc = 0;
    
    for(int i=0; i<mutatedPopulation->size(); i++){
        if(mutatedPopulation->at(i).fitness>max){
            max = mutatedPopulation->at(i).fitness;
            maxLoc = i;
        }
    }
    population.push_back(mutatedPopulation->at(maxLoc));
    population.push_back(mutatedPopulation->at(maxLoc));
    
    while(population.size() < popSize)
    {
        int pullLocation1 = rand()% popSize*2;
        int pullLocation2 = rand()% popSize*2;
        
        while(pullLocation1 == pullLocation2)
        {
            pullLocation1 = rand()% popSize*2;
        }
        
        if(mutatedPopulation->at(pullLocation1).fitness > mutatedPopulation->at(pullLocation2).fitness){
            population.push_back(mutatedPopulation->at(pullLocation1));
        }
        
        else if (mutatedPopulation->at(pullLocation1).fitness <= mutatedPopulation->at(pullLocation2).fitness)
        {
            population.push_back(mutatedPopulation->at(pullLocation2));
        }
    }
    assert(population.size() == mutatedPopulation->size()/2);
    return population;
};

void policy::initPolicy(vector<double> nnWeights){
    weights= nnWeights;
    fitness = -100000;

}

vector<policy> replicatePop(vector<policy>* population){
    
    // take in current population
    vector<policy> mutatePop = *population;
    assert(mutatePop.size() == popSize);
    int pullLocation;
    
    // replicate and mutate
    for(int i =0; i<popSize; i++)
    {
        policy B;
        pullLocation = rand()% mutatePop.size();
        vector<double> mutate;
        mutate = B.mutatePolicy(mutatePop.at(pullLocation).weights);
        B.initPolicy(mutate);
        mutatePop.push_back(B);
    }
    
    assert(mutatePop.size() == popSize * 2);
    return mutatePop;
};

int main() {
    srand(time_t(NULL));
    vector<policy> population;
    int numGens;
    int numWeights;
    vector<policy> mutatedPopulation;
    vector<double> xValues;
    vector<double> yValues;
    vector<double> thetaValues;
    
    numGens = promptUserStart();
    
    neural_network NN;
    NN.setup(nnIn, nnHidden, nnOut);
    NN.set_in_min_max(-2*pi , pi*2);
    NN.set_out_min_max(-15,15);
    numWeights = NN.get_number_of_weights();

    // initialize population
    population = initPop(&NN);
    
    //initialize simulation
    simulate boat;
    boat.initSim();
    
    //Start first for loop
    for(int i = 0; i< numGens; i++)
    {
        // Replicate -> Mutate
        mutatedPopulation = replicatePop(&population);
        for(int j = 0; j< popSize*2 ; j++)
        {
            NN.set_weights(mutatedPopulation.at(j).weights, true);
            while(boat.xResult !=1 && boat.xResult !=2 && boat.xResult !=4 && boat.yResult != 1)
            {
                /// give NN the state
                vector<double> state;
                state.push_back(boat.theta);
                NN.set_vector_input(state);
                /// let NN compute what thrust should be
                NN.execute();
                double output = NN.get_output(0);
                output = output*(pi/180);
                boat.u = output;
                /// simulate a time step
                boat.simData(&boat);
                if(boat.xResult != 3){
                }
            }
            mutatedPopulation.at(j).fitness =  boat.evalFitness(&boat);
            cout<< "Fitness: " << mutatedPopulation.at(j).fitness <<endl;
            boat.reset(&boat,0);
            
        }
        
        // Downselect -> Binary tournament
        population.clear();
        population = downSelect(&mutatedPopulation);
        if(i == numGens-1){
            int check = -5;
            int bestLoc;
            for(int k = 0; k<popSize; k++){
                if(population.at(k).fitness>check){
                    check = population.at(k).fitness;
                    bestLoc= k;
                }
            }

            boat.reset(&boat,0);
                NN.set_weights(population.at(bestLoc).weights, true);
                while(boat.xResult !=1 && boat.xResult !=2 && boat.xResult !=4 && boat.yResult != 1)
                {
                    /// give NN the state
                    vector<double> state;
                    state.push_back(boat.theta);
                    NN.set_vector_input(state);
                    /// let NN compute what thrust should be
                    NN.execute();
                    double output = NN.get_output(0);
                    output = output*(pi/180);
                    boat.u = output;
                    /// simulate a time step
                    boat.simData(&boat);
                    xValues.push_back(boat.boatXLoc);
                    yValues.push_back(boat.boatYLoc);
                    thetaValues.push_back(boat.theta);
                }
            ofstream boatPath;
            boatPath.open("BoatPath_File");
            for(int c=0;c<xValues.size();c++)
            {
                boatPath << xValues.at(c) << "\t" << yValues.at(c)<< "\t" << thetaValues.at(c)<< "\n";
                
            }
            
            boatPath.close();
            
        }
    }
}















