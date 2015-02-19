//============================================================================
// Name        : ArdroneBot.cpp
// Author      : Nathan Anderson, Stepheny Perez, Amber Wathen, Xiufeng Wang
// Project     : CS-483 Robotics project 1
// Copyright   : 
// Description : Ardrone AI contoller in C++, Ansi-style. 
//============================================================================

#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <streambuf>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <cmath>   
#include <ctime> 
#include <locale> 
#include <stdio.h>

#define fetchIMU "rostopic echo -n 1 /ardrone/imu"
#define takeOff "rostopic pub -1 /ardrone/takeoff std_msgs/Empty"
#define land "rostopic pub -1 /ardrone/land std_msgs/Empty"
#define moveStrStart "rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear:  {x: "

using namespace std;

streambuf* oldCoutStreamBuf = cout.rdbuf();
string moStrArray[] = { moveStrStart, ", y: ", ", z: ", "}, angular: {x: ", ",y: ", ",z: ", "}}'"  };    
int parseLocations[] = { 15, 17, 19, 35, 37, 39, 53, 55, 57};
FILE* moveFP;
FILE* fp;
FILE* takeOffFP;
ofstream logFile;

struct droneData
{
    double data[9];
}nav;

void setMovement(double speedX, double speedY, double speedZ){
    stringstream moveString;
    moveString.precision(4);
    moveString << fixed << moStrArray[0] << speedX << moStrArray[1] << speedY << moStrArray[2] << speedZ 
           << moStrArray[3] << 0.0 << moStrArray[4] << 0.0 << moStrArray[5] << 0.0 << moStrArray[6] << '\0';
    
    moveFP = popen(moveString.str().c_str(), "r");
    usleep(100000);
}

void updateDroneData()
{
    fp = popen(fetchIMU, "r");

    string dataArray[74];
    int idx = 0;

    if (fp != NULL) {
        char lineWidth [200];

        while(fgets(lineWidth,sizeof lineWidth,fp)!= NULL) {
            int n = 0;
            const char* br[20];
            br[0] = strtok(lineWidth, " ");

            if (br[0])
            {
                for (n = 1; n < 20; n++)
                {
                br[n] = strtok(0, " ");
                if(!br[n])
                    break;
                }
            }
            for (int i = 0; i < n; i++)
            {
                //cout<<"Line "<<i<<": "<<br[i]<<endl;
                dataArray[idx] = br[i];
                idx++;
            }

        }
    }

    stringstream temp;
    double d;

    for (int i = 0; i < 9; ++i)
    {
        temp.clear();
        d = 0.0;
        temp<<dataArray[parseLocations[i]];
        temp>>d;
        nav.data[i] = d;
    }
}

bool onTarget(double xDif, double yDif, double zDif)
{
    if ( xDif < 0.5 && xDif > -0.5 && 
         yDif < 0.5 && yDif > -0.5 &&
         zDif < 0.5 && zDif > -0.5 ){
        return true;
    }
        
    return false;
}

void moveToXYZ(double x, double y, double z, double close)
{
    double xDif;
    double yDif;
    double zDif;
    double distance;
    double locationCount = 0;

    do
    {
        updateDroneData();

        xDif = x - nav.data[6];
        yDif = y - nav.data[7];
        zDif = z - nav.data[8];

        distance = sqrt(pow(xDif, 2.0) + pow(yDif, 2.0) + pow(zDif, 2.0));

        if (distance < close){
            setMovement(0.0, 0.0, 0.0);

            if (++locationCount > 2)
                break;
        }

        if (distance < 7)
	{
		if (xDif < 7.0 && xDif > -7.0)
		    xDif /= (7.0 - distance);
		
		if (yDif < 7.0 && yDif > -7.0)
		    yDif /= (7.0 - distance);
		
		if (zDif < 7.0 && zDif > -7.0)
		    zDif /= (7.0 - distance);
	}

        cout << "Distance remainting: " << distance << endl;
        cout << "location: " << nav.data[6] << " " << nav.data[7] << " " << nav.data[8] << endl;

        setMovement(xDif, yDif, zDif);

        double tempDistance = distance / 1.15;

        while (tempDistance < distance && (tempDistance * 1.15) >= distance)
        {
            updateDroneData();
        
            xDif = x - nav.data[6];
            yDif = y - nav.data[7];
            zDif = z - nav.data[8];

            distance = sqrt(pow(xDif, 2.0) + pow(yDif, 2.0) + pow(zDif, 2.0));
        }

    } while (1);
}

void printLocation()
{
    locale loc;
    const time_put<char>& tmput = use_facet <time_put<char> > (loc);

    time_t timestamp;
    time ( &timestamp );
    tm * now = localtime ( &timestamp );
 
    
    updateDroneData();

    logFile<<"("<<nav.data[6]<<","<<nav.data[7]<<","<<nav.data[8]<<") (";
    tmput.put (logFile, logFile, ' ', now, 'X');
    logFile <<")\n";
}

int main()
{
    logFile.open ("output.txt", std::fstream::out);
    logFile.precision(2);
    logFile<<fixed;

    double ftX = -9.0;
    double ftY = 4.0;
    double ftZ = 0.5;
    double stX = 8.5;
    double stY = -5.7;
    double stZ = 1.0;

    double maxHeight = 6.0;

    printLocation();
    
    takeOffFP = popen(takeOff, "r");
    
    moveToXYZ(0.0, 0.0, maxHeight, 1.5);

    moveToXYZ(ftX, ftY, maxHeight, 0.9);

    
    moveToXYZ(ftX, ftY, maxHeight - 1, 0.9);
    moveToXYZ(ftX, ftY, maxHeight - 2, 0.9);
    moveToXYZ(ftX, ftY, maxHeight - 2, 0.9);

    moveToXYZ(ftX, ftY, ftZ, 0.8);

    printLocation();

    takeOffFP = popen(takeOff, "r");
    
    moveToXYZ(ftX, ftY, maxHeight, 1.5);

    moveToXYZ(stX, stY, maxHeight, 0.9);

    moveToXYZ(stX, stY, maxHeight - 1, 0.9);
    moveToXYZ(stX, stY, maxHeight - 2, 0.9);

    moveToXYZ(stX, stY, stZ, 0.8);

    printLocation();

    takeOffFP = popen(land, "r");

    logFile.close();
    return 0;
}
