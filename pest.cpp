//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

#include <iostream>

using namespace std;
//------------------------------------------------------------------------------
//typedef Eigen::Matrix<float, 7, 1> vector7f;
//typedef Eigen::Matrix<float, 8, 1> vector8f;
void nextStimulus(double min, double max);
  double newStimulus[100];

int main(int argc, char* argv[])
{
    nextStimulus(0.0, 4000.0);


}


void nextStimulus(double min, double max)
{
    
    //newStimulus[0]= min;
    //newStimulus[1] =max;

      // double step = newStimulus[1]-newStimulus[0];


    double max_step = max-min;
    double max_stim = max;
    double min_step=1;
    double init_step=max_step/16.0;
    int answer;
    int prev_answer=0; 
    double dir =1.0;
    int doubling_rule=2;
    bool doubled=false;
    int n=0;
    newStimulus[0]=max_stim;
    int i=0;
    while (n<2)
    {
        n++;
        cout<<"Surface zero or one? "<<endl;
        cin>> answer;
        if (prev_answer != answer)
        {
            dir = -1.0 *dir;
        }
        newStimulus[n]=newStimulus[n-1]+dir*init_step;
        prev_answer=answer;
        std::cout<<"stimulusss "<<newStimulus[n]<<endl;


    }
    double step = init_step;
    bool entered=true;
    while(n<=20)
    {

        n++;
        cout<<"Surface zero or one? "<<endl;
        cin>> answer;
        if (prev_answer != answer)
        {
            i=0;
            dir= (-1)*dir;
            step = step / 2.0;
            std::cout<<"here2"<<endl;

            if (step  >max_step)
            {
                step=max_step;
            }
            if (step<min_step)
            {
                break;
            }
        }
        else if (prev_answer == answer)
        {
            i++;
            if (entered)
            {
                dir = dir * 1.0;
                step = 2.0 * step;
                std::cout<<"here3"<<endl;
                //doubled = true;
                i=i+3;
            }
            else{
                if (i<doubling_rule)
                {
                    dir = dir * 1.0;
                    step=step;

                }
                else if(i=doubling_rule)
                {
                    if(doubled==false)
                    {
                        step= 2.0 * step;
                        doubled = true;
                        std::cout<<"here4"<<endl; 

                    }
                    else if(doubled ==true)
                    {
                        step=step;
                        doubled =false;
                        std::cout<<"here5"<<endl; 
                    }

                }


                else if(i>doubling_rule)
                {
                    dir = dir * 1.0;
                    step = 2.0 * step;
                    std::cout<<"here"<<endl;
                }
            }

            if (step>max_step)
            {
                step=max_step;
            }
    
        }
        double temp_stimulus = newStimulus[n-1] + dir*step;

        if (temp_stimulus >=max){
            newStimulus[n]=(newStimulus[n-1]+max)/2.0;
            step = max-newStimulus[n];
           
        }
        else if (temp_stimulus <=min){
            newStimulus[n]=(newStimulus[n-1]+min)/2.0;
            step = newStimulus[n]-min;
        }
        else
        {
            newStimulus[n]=temp_stimulus;
        }
        prev_answer=answer;
        std::cout<<"stimulus "<<newStimulus[n]<<endl;
 
        entered =false;

    }
}


