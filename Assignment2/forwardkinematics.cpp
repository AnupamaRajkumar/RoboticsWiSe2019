/*

 Please fill in the function stubs provided in this file
 
 For achieving full points on this exercise, explain the implemented equations well using comments. Especially:
   - which parameters were used for computation?
   - were equations simplified? What was the original equation?
   - when introducing additional variables, what do they contain?

 
 * Do not use additional includes
 * Do not include/link additional files
 * Do not change the predefined function signature
 
 Tip: use the main() function to test your implementation.

*/

#include "forwardkinematics.hpp" //Have a look at this header file, it declares the class ForwardKinematicsPuma2D
#include <cmath>                 //use sin, cos from cmath for your computation
#include <iostream>
using namespace std;


#ifdef UNITTEST
    #define main STUDENTS_MAIN
#endif   


/*
Convenience function to print out a homogenous transform
*/
void print_HTransform(HTransform tf)
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\nHTransform\n---------------------------\n";
    
    cout << tf[0][0]<<"  "<<tf[0][1]<<"  "<<tf[0][2]<<"  "<<tf[0][3]<<endl;
    cout << tf[1][0]<<"  "<<tf[1][1]<<"  "<<tf[1][2]<<"  "<<tf[1][3]<<endl;
    cout << tf[2][0]<<"  "<<tf[2][1]<<"  "<<tf[2][2]<<"  "<<tf[2][3]<<endl;
    cout << tf[3][0]<<"  "<<tf[3][1]<<"  "<<tf[3][2]<<"  "<<tf[3][3]<<endl;
    cout <<   "---------------------------\n";
}

/*
Convenience function to print out a 3x3 Jacobian matrix
*/
void print_Jacobian(float Jacobian[3][3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\nJacobian\n+++++++++++++++++++++++++++\n";
    cout << Jacobian[0][0]<<"  "<<Jacobian[0][1]<<"  "<<Jacobian[0][2]<<endl;
    cout << Jacobian[1][0]<<"  "<<Jacobian[1][1]<<"  "<<Jacobian[1][2]<<endl;
    cout << Jacobian[2][0]<<"  "<<Jacobian[2][1]<<"  "<<Jacobian[2][2]<<endl;
    cout <<   "+++++++++++++++++++++++++++\n";
}


/*
Convenience function to print out a position
*/
void print_Position(float F[3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\nPosition\nooooooooooooooooooooooooooo\n";
    cout << F[0]<<"  "<<F[1]<<"  "<< F[2]<<endl;
    cout <<   "ooooooooooooooooooooooooooo\n";
}


/*
already implemented

Set the robot's joint values and recompute the forward kinematics.

a1, a2 and a3 are assumed to be given in radians!
*/
void ForwardKinematicsPuma2D::setJoints(float a1, float a2, float a3) 
{
    //store joint angles
    angles[0] = a1;
    angles[1] = a2;
    angles[2] = a3;
    //recompute the dependent variables
    computeT0_1();
    computeT1_2();
    computeT2_3();
    computeT3_E();
    computeT0_E();
    computeF();
    computeJ();
}




/***********************************************/
/******************EDIT BELOW ******************/
/***********************************************/



/*
updates the variable T0_1

<ADD EXPLANATION OF CODE>
 * Any new 3D position and orientation can be described by a rotation and translation in two directions, ie T=R_x*T_x*R_z*T_z where each is 4x4
 * To do this for T0_1, we look to the DH parameters for i=1.
 * Alpha, d and a are zero in this case so there is no rotation (R_x) or translation (T_x and T_z) from these matrices, ie they are identity matrices
 * Therefore, for T0_1, we only have the rotation around the z-axis according to the position of q1.
 * To convert to the simulator reference frame however, we need to do an additional rotation about the x-axis.
*/
void ForwardKinematicsPuma2D::computeT0_1()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T0_1[0][0] = cos(angles[0]);
   T0_1[0][1] = sin(angles[0]);
   T0_1[0][2] = 0.0;
   T0_1[0][3] = 0.0;

   //row vector
   T0_1[1][0] = sin(angles[0]);
   T0_1[1][1] = -cos(angles[0]);
   T0_1[1][2] = 0.0;
   T0_1[1][3] = 0.0;

   //row vector
   T0_1[2][0] = 0.0;
   T0_1[2][1] = 0.0;
   T0_1[2][2] = -1.0;
   T0_1[2][3] = 0.0;

   //row vector
   T0_1[3][0] = 0.0;
   T0_1[3][1] = 0.0;
   T0_1[3][2] = 0.0;
   T0_1[3][3] = 1.0;
}


/*
updates the variable T1_2

<ADD EXPLANATION OF CODE>
 *  * Any new 3D position and orientation can be described by a rotation and translation in two directions, ie T=R_x*T_x*R_z*T_z where each is 4x4
 * To do this for T1_2, we look to the DH parameters for i=2.
 * Alpha and d are zero in this case so there is no rotation (R_x) or translation (T_z) from these matrices, ie they are identity matrices
 * Therefore, for T1_2, we have the rotation around the z-axis according to the position of q1 and the translation in the x-axis by l1.
 * Note that in order to avoid having (angles[1]-PI/2) in all rotational terms, the sin(x+90)=cos(x) identity was used.
*/

void ForwardKinematicsPuma2D::computeT1_2()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T1_2[0][0] = sin(angles[1]);
   T1_2[0][1] = cos(angles[1]);
   T1_2[0][2] = 0.0;
   T1_2[0][3] = l1;

   //row vector
   T1_2[1][0] = -cos(angles[1]);
   T1_2[1][1] = sin(angles[1]);
   T1_2[1][2] = 0.0;
   T1_2[1][3] = 0.0;

   //row vector
   T1_2[2][0] = 0.0;
   T1_2[2][1] = 0.0;
   T1_2[2][2] = 1.0;
   T1_2[2][3] = 0.0;

   //row vector
   T1_2[3][0] = 0.0;
   T1_2[3][1] = 0.0;
   T1_2[3][2] = 0.0;
   T1_2[3][3] = 1.0;
}

/*
updates the variable T2_3

<ADD EXPLANATION OF CODE>
 *  * Any new 3D position and orientation can be described by a rotation and translation in two directions, ie T=R_x*T_x*R_z*T_z where each is 4x4
 * To do this for T2_3, we look to the DH parameters for i=3.
 * Alpha, d and a are zero in this case so there is no rotation (R_x) or translation (T_z) from these matrices, ie they are identity matrices
 * Therefore, for T2_3, we have the rotation around the z-axis according to q3 and the translation in x-axis by l2
 * Note that in order to avoid having (angles[1]-PI/2) in all rotational terms, the sin(x+90)=cos(x) identity was used.
*/
void ForwardKinematicsPuma2D::computeT2_3()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T2_3[0][0] = cos(angles[2]);
   T2_3[0][1] = -sin(angles[2]);
   T2_3[0][2] = 0.0;
   T2_3[0][3] = l2;

   //row vector
   T2_3[1][0] = sin(angles[2]);
   T2_3[1][1] = -cos(angles[2]);
   T2_3[1][2] = 0.0;
   T2_3[1][3] = 0.0;

   //row vector
   T2_3[2][0] = 0.0;
   T2_3[2][1] = 0.0;
   T2_3[2][2] = 1.0;
   T2_3[2][3] = 0.0;

   //row vector
   T2_3[3][0] = 0.0;
   T2_3[3][1] = 0.0;
   T2_3[3][2] = 0.0;
   T2_3[3][3] = 1.0;
}


/*
updates the variable T3_E

<ADD EXPLANATION OF CODE>
 *  * Any new 3D position and orientation can be described by a rotation and translation in two directions, ie T=R_x*T_x*R_z*T_z where each is 4x4
 * To do this for T3_E, we look to the DH parameters for i=4.
 * Alpha, theta and a are zero in this case so there is no rotation (R_x and R_z) and no z-translation (T_z) from these matrices, ie they are identity matrices
 * Therefore, for T3_E, we only have the translation over the x-axis by l3.
*/
void ForwardKinematicsPuma2D::computeT3_E()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T3_E[0][0] = 1.0;
   T3_E[0][1] = 0.0;
   T3_E[0][2] = 0.0;
   T3_E[0][3] = l3;

   //row vector
   T3_E[1][0] = 0.0;
   T3_E[1][1] = 1.0;
   T3_E[1][2] = 0.0;
   T3_E[1][3] = 0.0;

   //row vector
   T3_E[2][0] = 0.0;
   T3_E[2][1] = 0.0;
   T3_E[2][2] = 1.0;
   T3_E[2][3] = 0.0;

   //row vector
   T3_E[3][0] = 0.0;
   T3_E[3][1] = 0.0;
   T3_E[3][2] = 0.0;
   T3_E[3][3] = 1.0;
}


/*

This function updates the variable T0_E

<ADD EXPLANATION OF CODE>
 *  To find T0_E, we can multiply the previous homogenous transforms T0_1,T1_2,T2_3. This will give the homogenous transform from 0 to E.
 * Note that in order to simplify terms, the sin(x+90)=cos(x), the cos(x+y)=cos(x)cos(y)-sin(y)sin(x)
 * and the sin(x+y)=cos(x)sin(y)+sin(x)cos(y) identities  were used. These were used to simplify the compounding rotational terms that result from multiplying T1_2 & T2_3 for example.
*/
void ForwardKinematicsPuma2D::computeT0_E()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T0_E[0][0] = sin(angles[0]+angles[1]+angles[2]);
   T0_E[0][1] = cos(angles[0]+angles[1]+angles[2]);
   T0_E[0][2] = 0.0;
   T0_E[0][3] = sin(angles[0]+angles[1]+angles[2])*l3+sin(angles[0]+angles[1])*l2+cos(angles[0])*l1;

   //row vector
   T0_E[1][0] = cos(angles[0]+angles[1]+angles[2]);
   T0_E[1][1] = -sin(angles[0]+angles[1]+angles[2]);
   T0_E[1][2] = 0.0;
   T0_E[1][3] = cos(angles[0]+angles[1]+angles[2])*l3+cos(angles[0]+angles[1])*l2-sin(angles[0])*l1;

   //row vector
   T0_E[2][0] = 0.0;
   T0_E[2][1] = 0.0;
   T0_E[2][2] = -1.0;
   T0_E[2][3] = 0.0;

   //row vector
   T0_E[3][0] = 0.0;
   T0_E[3][1] = 0.0;
   T0_E[3][2] = 0.0;
   T0_E[3][3] = 1.0;
}



/*
This function updates the variables ee_x, ee_y, ee_alpha

<ADD EXPLANATION OF CODE>
The T0_E 4x4 homogenous transform can be seen as a 3x3 rotation and a 3x1 translation. 
 *The x translation is given by the T0_E[0][3] entry. 
 *The y translation is given by the T0_E[1][3] entry.
 *The alpha position will always simply be the sum of the angles of the joints, ie sum angles[0],angles[1],angles[2].
*/
void ForwardKinematicsPuma2D::computeF()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   F[0] = sin(angles[0]+angles[1]+angles[2])*l3+sin(angles[0]+angles[1])*l2+cos(angles[0])*l1; //x
   F[1] = cos(angles[0]+angles[1]+angles[2])*l3+cos(angles[0]+angles[1])*l2-sin(angles[0])*l1; //y
   F[2] = angles[0]+angles[1]+angles[2]; //alpha
}


/*
This function updates the variable J

<ADD EXPLANATION OF CODE>
 * The Jacobian can be found by taking the partial derivative of each F(x,y,z) row entry in terms of the three different joint angles: angles[0],angles[1],angles[2].
 * Each column of the Jacobian corresponds to the partial derivatives in terms of a joint angle.
 * It's thus a 3x3 matrix: row 1 showing the changes in x with regard to different joint angles,
 row 2 showing the changes in y with regard to different joint angles,
 row 3 showing the changes in alpha with regard to different joint angles
 * 
*/
void ForwardKinematicsPuma2D::computeJ()
{

   // compute from 
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   J[0][0] = cos(angles[0]+angles[1]+angles[2])*l3+cos(angles[0]+angles[1])*l2-sin(angles[0])*l1;
   J[0][1] = cos(angles[0]+angles[1]+angles[2])*l3+cos(angles[0]+angles[1])*l2;
   J[0][2] = cos(angles[0]+angles[1]+angles[2])*l3;

   //row vector
   J[1][0] = -sin(angles[0]+angles[1]+angles[2])*l3-sin(angles[0]+angles[1])*l2-cos(angles[0])*l1;
   J[1][1] = -sin(angles[0]+angles[1]+angles[2])*l3-sin(angles[0]+angles[1])*l2;
   J[1][2] = -sin(angles[0]+angles[1]+angles[2])*l3;

   //row vector
   J[2][0] = 1.0;
   J[2][1] = 1.0;
   J[2][2] = 1.0;
}



/*
Example code to test your functions:

You are free to change main() as you like
*/
int main()
{
 ForwardKinematicsPuma2D* fk = new ForwardKinematicsPuma2D();
 fk->setJoints(1.0,1.0,1.0);
 print_HTransform(fk->T0_E); //example
 print_Position(fk->F); //example
 print_Jacobian(fk->J); //example
 return 0;
}


