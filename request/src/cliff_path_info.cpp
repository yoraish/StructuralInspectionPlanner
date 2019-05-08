// a script to read the plan file and print path length, duration for following in inputted velocity

#include <ros/ros.h>
#include "koptplanner/inspection.h"
#include "shape_msgs/SolidPrimitive.h"
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <ros/package.h>
#include "tf/tf.h"


#include <iostream>
#include<string>
#include<cmath>
#include<vector>
#include <sstream>
#include <map>
using namespace std;


/*
TESTS

stats
workflow video
workflow slides

approximation of clippings on map*
DOCUMENTATION for onboarding
*/

void coord_parser(string line, vector<float> * coord){
    int last_comma_ix = -2; // -2 to account for addition to get rid of spaces later

    for (int ix = 0; ix < 3; ix ++){
        int comma_ix = line.find(',', last_comma_ix+2);
        string value = line.substr(last_comma_ix+2, comma_ix - last_comma_ix);
        last_comma_ix = comma_ix;
        (*coord)[ix] = atof(value.c_str());
    }
}


map<string, float> path_info(string fname, float speed = 1){
    cout << "File: " << fname << endl;
    // open the file
    fstream pathPublication;
    string pkgPath = ros::package::getPath("request");
    pathPublication.open((pkgPath+"/visualization/"+fname).c_str());

    if (!pathPublication.is_open()){
        cout<<"waiting for file" << endl;
    }

    // holder for coords
    vector<float> last_coord;
    last_coord.push_back(0.0);
    last_coord.push_back(0.0);
    last_coord.push_back(0.0);
    vector<float> new_coord;
    new_coord.push_back(0.0);
    new_coord.push_back(0.0);
    new_coord.push_back(0.0);

    // gert first coord
    string line;
    getline(pathPublication, line);
    coord_parser(line.substr(line.find('[')+1, line.size() - line.find('[')-3), &new_coord);

    int counter = 0;
    while (line[0]!= ']'){

        // it is now old, get a new one
        getline(pathPublication, line);
        if (line[0] == ']'){
            break;
        }

        last_coord = new_coord;
        //erase things in new coord
        new_coord[0] = 0.0;
        new_coord[1] = 0.0;
        new_coord[2] = 0.0;
        // populate new coord
        coord_parser(line, &new_coord);

        // update counter with the distance bw the new and old coords
        counter += sqrt(
                        pow(new_coord[0] - last_coord[0],2)+
                        pow(new_coord[1] - last_coord[1],2)+
                        pow(new_coord[2] - last_coord[2],2));

        // for (int ix = 0; ix < 3; ix ++){
        //     cout << new_coord[ix] << ", ";
        // }
        // cout <<endl;

    }
    // calculate the values to return (length in m, runtime)
    map<string, float> data;
    // length in meters
    data["length"] = counter/1000.0;
    // runtime in meters per second
    data["runtime"] = data["length"]/speed;
    return  data;


}


void test(){
    // function with two tests to verify the correctness of this script

    cout << "\n--START OF TESTS--" << endl;

    // TEST 1
    vector<float> speed;
    speed.push_back(0.5); speed.push_back(0.25); speed.push_back(4);

    vector<int> sol_length;
    sol_length.push_back(5); sol_length.push_back(10);sol_length.push_back(5);

    vector<float> sol_runtime;
    sol_runtime.push_back(10); sol_runtime.push_back(40); sol_runtime.push_back(1.25);


    for (int j = 1; j < 4; j++){
        ostringstream s;
        s << j;
        string i(s.str());

        // load the file test1.m and compare it to required output = runtime = 10 seconds, length = 5 meters
        map<string,float> res =  path_info("test" +i+".m", speed[j-1]);
        cout << "TEST "<<i<< "---> ";

        if (res["runtime"] == sol_runtime[j-1] && res["length"] == sol_length[j-1]){
            cout << "OK" << endl;
        } else{
            cout << "FAIL\n" << "Test "+i+" Message:" << endl;
            cout << "Expected length " << sol_length[j-1] << " meters";
            cout << "Got length      " << res["length"] << " meters" <<endl;
            cout << "Expected runtime " << sol_runtime[j-1] << " seconds";
            cout << "Got runtime      " << res["runtime"] << " seconds\n\n" <<endl;      
        }


    }
    cout << "--END OF TESTS--\n" << endl;

}

int main(){

    bool testing = true;
    if (testing){
        test();
    }

    cout <<"Information about cliff inspection path" << endl;

    // call the info function
    map<string, float> data = path_info("inspectionScenario_cliff3.m", 0.25);
    cout << "Path length=" << data["length"] << " meters, \nRuntime=" << data["runtime"] << " seconds" << endl;


    return -1;
}
