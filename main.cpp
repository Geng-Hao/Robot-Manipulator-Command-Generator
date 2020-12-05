
#include<iostream>
#include"stdlib.h"
#include<fstream>
#include<vector>
#include<list>
#include<sstream>
#include<cmath>
#include"Vec.h"
#include"Robot_IK_Solver.h"

using namespace std;


typedef enum modes {
	Rapid_Positionning = 0,
	Linear_interpolation = 1,
	CW_interpolation = 2,
	CCW_interpolation = 3,
	unit_mm = 4
}modes;

/*function prototype*/

list<Vec> gcodeInterpreter(const string&, double, double);


vector<string> stringSplit(string);

double string2double (string);
string double2string (double);



/*main function*/

int main() {

	/*Set Up*/
	
	string path;
	cout<<"Please enter the g-code file path:";
	cin>>path;
	
	

	double line_resolution;
	double angle_resolution;

	cout<<"Please enter the line resolution:";
	cin>>line_resolution;
	cout<<"Please enter the angle resolution:";
	cin>>angle_resolution;


	/*G Code Interpreter*/
	list<Vec> toolPath = gcodeInterpreter(path, line_resolution, angle_resolution); //the tool path we need
	
	
	/* Robot Manipulator Meca500 Info */
	
	// DH Parameters
 	double alpha[6] = {0,-PI/2,0,-PI/2,PI/2,-PI/2};
 	double a[6] = {0,0,431.8,19,0,0};
 	double d[6] = {0,0,125,432,0,0};
 	double theta[6] = {0,1,1,1,0,0};
 	// joint limits
 	double l[6] = {-10,-10,-10,-10,-10,-10};
 	double u[6] = {10,10,10,10,10,10};
 	

 	IK_Solver MECA500(alpha,a,d,theta,l,u);
	
	
	/*Solve the Inverse Kinematics & Write the 6-axis Joint Command into a txt file*/

	fstream file;
	file.open("Command.txt",ios::out);
        
        file<<"ActivateRobot"<<endl<<"Home"<<endl;
        
	while(!toolPath.empty()) {

		Vec p = toolPath.front();
              	p.setXYZ(p.getX()+200,p.getY()+100,p.getZ()-200); p.print();
		toolPath.pop_front();
		
		MECA500.Inv_Kine(p,-PI,0,(-2/3)*PI,0.0001,1,100); //solve the inverse kinematics

		double* Joints = MECA500.getTheta(); // obtain the optimization result
		
		string theta1 = double2string(Joints[0]*180/PI);
		string theta2 = double2string(Joints[1]*180/PI);
		string theta3 = double2string(Joints[2]*180/PI);
		string theta4 = double2string(Joints[3]*180/PI);
		string theta5 = double2string(Joints[4]*180/PI);
		string theta6 = double2string(Joints[5]*180/PI);
		
		file<<"MoveJoints("<<theta1<<","<<theta2<<","<<theta3<<","<<theta4<<","<<theta5<<","<<theta6<<")"<<endl;

	}

     

}





list<Vec> gcodeInterpreter(const string& path, double line_resolution, double angle_resolution) {


	list<Vec> toolPath;



	ifstream file;
	file.open(path, ios::in);

	if (!file) {
		cout << "File could not be opened" << endl;
	}
	else {


		string gcodeline;
		list<Vec> interpolate_Vec;

		Vec	current_Vec; // the current Vec of the command line
		Vec	arc_offsets;      // the x, y offsets relative to the central point
		modes operation_mode;		  // the operation mode of current command

		while (getline(file, gcodeline)) {  // read lines into strings: gcodeline


			


			if (gcodeline.empty()) continue;

			if (gcodeline.at(0) == 'G') {


				

				vector<string> CodeSection = stringSplit(gcodeline); // split a command line into code sections, then store each code section in a vector

				

				/*This section is to determine the G code operation mode and its start Vec or arc offset relative to the central point for each line of the g-code*/
				for (long unsigned int i = 0; i < CodeSection.size(); i++) {

					

					if (CodeSection.at(i) == "G00")
						operation_mode = Rapid_Positionning;
					else if (CodeSection.at(i) == "G01")
						operation_mode = Linear_interpolation;
					else if (CodeSection.at(i) == "G02")
						operation_mode = CW_interpolation;
					else if (CodeSection.at(i) == "G03")
						operation_mode = CCW_interpolation;
					else if (CodeSection.at(i) == "G21")
						operation_mode = unit_mm;
					else {

						if (CodeSection.at(i).at(0) == 'X') {

							string x = "";

							for (long unsigned int k = 1; k < CodeSection.at(i).size(); k++)
								x += CodeSection.at(i).at(k);

							current_Vec.setX(string2double(x));
						}
						else if (CodeSection.at(i).at(0) == 'Y') {

							string y = "";

							for (long unsigned int k = 1; k < CodeSection.at(i).size(); k++)
								y += CodeSection.at(i).at(k);

							current_Vec.setY(string2double(y));


						}
						else if ((CodeSection.at(i).at(0) == 'Z')) {

							string z = "";

							for (long unsigned int k = 1; k < CodeSection.at(i).size(); k++)
								z += CodeSection.at(i).at(k);

							if (string2double(z) >= 0)
								current_Vec.setZ(0);
							else 
								current_Vec.setZ(1);
						
						}
						else if ((CodeSection.at(i).at(0) == 'I')) {

							string I = "";

							for (long unsigned int k = 1; k < CodeSection.at(i).size(); k++)
								I += CodeSection.at(i).at(k);

							arc_offsets.setX(string2double(I));



						}
						else if ((CodeSection.at(i).at(0) == 'J')) {

							string J = "";

							for (long unsigned int k = 1; k < CodeSection.at(i).size(); k++)
								J += CodeSection.at(i).at(k);

							arc_offsets.setY(string2double(J));

						}



					}

				}


				/*Linear Modes Interpolation*/

				if (operation_mode == Linear_interpolation || operation_mode == Rapid_Positionning) {

					

					if (toolPath.size() > 0) {

						double distance = (current_Vec - toolPath.back()).norm2();

					     /* cout << toolPath.back().getX() << " " << toolPath.back().getY() << " " << toolPath.back().getZ() << endl;
						cout << current_Vec.getX() << " " << current_Vec.getY() << " " << current_Vec.getZ() << endl;
						cout << distance << endl; */
						

						if (distance > 0) {

							

							Vec direction_vector = (current_Vec - toolPath.back()) / distance;

							for (double d = 0; d < distance; d = d + line_resolution) 
								interpolate_Vec.push_back(toolPath.back() + direction_vector * d);
							
	
							interpolate_Vec.push_back(current_Vec);



						}


					}
					else {

						interpolate_Vec.push_back(current_Vec);
					}


				}/*Clockwise Modes Interpolation*/
				else if (operation_mode == CW_interpolation) {

					Vec central_point = toolPath.back() + arc_offsets;
					Vec v1 = toolPath.back() - central_point; v1.setZ(0);
					Vec v2 = current_Vec - central_point; 

					

					double r = v2.norm2();

					

					double theta_0 = atan2(v1.getY(), v1.getX()) * 180 / PI;     // start angle
					double theta_end = atan2(v2.getY(), v2.getX()) * 180 / PI;  // end angle
					
					

					if (theta_end > theta_0) theta_end = theta_end - 360;

					Vec v_tmp((central_point.getX() + cos(theta_0 * PI / 180.0) * r), (central_point.getY() + sin(theta_0 * PI / 180.0) * r), current_Vec.getZ());

					// arc interpolation
					for (double ang = theta_0; ang > theta_end; ang -= angle_resolution) {

						v1.setXYZ((central_point.getX() + cos(ang * PI / 180.0) * r), (central_point.getY() + sin(ang * PI / 180.0) * r), current_Vec.getZ());
                        
						
						double distance = (v1 - v_tmp).norm2();

						if ( distance > line_resolution) {

							for (double d = 0; d < distance; d += 1*line_resolution)
								interpolate_Vec.push_back(toolPath.back() + ((v1 - v_tmp)/distance) * d);


						}

						interpolate_Vec.push_back(v1);

						v_tmp = v1;
					}
					
					interpolate_Vec.push_back(current_Vec);

				}/*Counter Clockwise Modes Interpolation*/
				else if (operation_mode == CCW_interpolation) {

					Vec central_point = toolPath.back() + arc_offsets;
					Vec v1 = toolPath.back() - central_point; v1.setZ(0);
					Vec v2 = current_Vec - central_point; v2.setZ(0);

					double r = v2.norm2();

					double theta_0 = atan2(v1.getY(), v1.getX()) * 180 / PI;     // start angle
					double theta_end = atan2(v2.getY(), v2.getX()) * 180 / PI;  // end angle

					if (v1.norm2() < 0.1) theta_0 = 0;
					if (v2.norm2() < 0.1) theta_end = 0;
					if (theta_end < theta_0) theta_end = theta_end + 360;
					

					Vec v_tmp((central_point.getX() + cos(theta_0 * PI / 180.0) * r), (central_point.getY() + sin(theta_0 * PI / 180.0) * r), current_Vec.getZ());

					// arc interpolation
					for (double ang = theta_0; ang < theta_end; ang += angle_resolution) {

						v1.setXYZ((central_point.getX() + cos(ang * PI / 180.0) * r), (central_point.getY() + sin(ang * PI / 180.0) * r), current_Vec.getZ());


						double distance = (v1 - v_tmp).norm2();

						if (distance > line_resolution) {

							for (double d = 0; d < distance; d += 1*line_resolution)
								interpolate_Vec.push_back(toolPath.back() + ((v1 - v_tmp) / distance) * d);


						}

						interpolate_Vec.push_back(v1);

						v_tmp = v1;
					}
					

					interpolate_Vec.push_back(current_Vec);

				}



			}



			while(!interpolate_Vec.empty()) {

				toolPath.push_back(interpolate_Vec.front());

				interpolate_Vec.pop_front();

				
			}

			

			interpolate_Vec.clear();

		}


		file.close();


	}

	return toolPath;


}









vector<string> stringSplit(string buffer) {

	vector<string> stringSectionList;

	stringstream ss(buffer);
	string section;

	int i = 0;

	while (getline(ss, section, ' ')) {
		stringSectionList.push_back(section);
		i++;
	}

	return stringSectionList;

}


double string2double(string numStr) {

	stringstream ss;

	
	double num;
	ss << numStr;
	ss >> num;

	return num;


}


string double2string(double d) {

	stringstream ss;
	
	ss << d; 
    
	string str;
	
	ss >> str;  

	return str;
}

