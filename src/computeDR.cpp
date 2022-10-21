#include <iostream>
#include <math.h>

const int SUCCESS_RETURN = 0;
const int ERROR_RETURN = -1;

const double pi = 3.141592653589793239;
const double TWOpi = 6.283185307179586478;
const double HALFpi = 1.570796326794896619;
const double NEGHALFpi = -HALFpi;
const double DtoR = 1.45329251994329577e-02;

using std::cout;
using std::endl;

struct AngularVelocity {
   double roll;
   double pitch;
   double yaw;
};

struct VectorXYZ {
   double x;
   double y;
   double z;
};

struct Orientation {
   double psi;
   double theta;
   double phi;
};

struct Quaternion {
   float qU0;
   float qUx;
   float qUy;
   float qUz;
};

enum class DR_ALGORITHM : unsigned char {
   DRM_STATIC = 1,
   DRM_FPW = 2,
   DRM_RPW = 3,
   DRM_RVW = 4,
   DRM_FVW = 5,
   DRM_FPB = 6,
   DRM_RPB = 7,
   DRM_RVB = 8,
   DRM_FVB = 9
};

enum class DR_PARAMS_TYPE : unsigned char {
   DR_PARAMS_LEGACY = 0,
   DR_PARAMS_LOCAL_EULER = 1,
   DR_PARAMS_QUATERNION = 2
};

struct ComputeDRParams {
   double deltaTime;
   DR_ALGORITHM drAlgorithm;
   DR_PARAMS_TYPE drParamsType;
   Orientation localEulerAngles;
   Quaternion quaternion;
   VectorXYZ acceleration;
   AngularVelocity angularVelocity;
   VectorXYZ location;
   VectorXYZ velocity;
   Orientation orientation;
};

struct TrigValues {
   double sinPsi, sinTheta, sinPhi;
   double cosPsi, cosTheta, cosPhi;
   double sPsisPhi, sPsicPhi, cPsisPhi, cPsicPhi;
};

//faster than fabs:
#define ABS(x) ( (x) >= 0 ? (x) : ( -(x) ) )

//minimum significant rate = 1deg/5sec
const double  MIN_ROTATION_RATE = 0.2 * DtoR;

//minimum significant rate = 1m/10sec^2
const double MIN_ACCELERATION_RATE = 0.1;

void printHeader() {
   cout << endl;
   cout << "===============================================" << endl;
   cout << "Tests of the Dead Reckoning algorithms" << endl;
   cout << "implemented by John Towers & Jack Hines" << endl;
   cout << "===============================================" << endl;
   cout << endl;
}


void printValues( const char * pTitle, const ComputeDRParams & params ) {
   cout << pTitle << " Values:" << endl;
   cout << "-------------------------" << endl;
   cout << "Position [m] :" << endl;
   cout << "x     = " << std::fixed << params.location.x << endl;
   cout << "y     = " << std::fixed << params.location.y << endl;
   cout << "z     = " << std::fixed << params.location.z << endl;
   cout << "Velocity [m/s] :" << endl;
   cout << "Vx    = " << std::fixed << params.velocity.x << endl;
   cout << "Vy    = " << std::fixed << params.velocity.y << endl;
   cout << "Vz    = " << std::fixed << params.velocity.z << endl;
   cout << "Orientation [deg] :" << endl;
   cout << "psi   = " << std::fixed << params.orientation.psi / DtoR << endl;
   cout << "theta = " << std::fixed << params.orientation.theta / DtoR << endl;
   cout << "phi   = " << std::fixed << params.orientation.phi / DtoR << endl;
}


void printDRParams( const ComputeDRParams & params) {

   cout << "Dead Reckoning Parameters:"<< endl;
   cout << "--------------------------" << endl;
   cout << "algorithm : " << static_cast<unsigned int>( params.drAlgorithm ) << endl;
   cout << "Angular Velocity [deg/s]  :" << endl;
   cout << "roll  = " << std::fixed << params.angularVelocity.roll / DtoR << endl;
   cout << "pitch = " << std::fixed << params.angularVelocity.pitch / DtoR << endl;
   cout << "yaw   = " << std::fixed << params.angularVelocity.yaw / DtoR << endl;
   cout << "Linear Acceleration [m/s^2] :" << endl;
   cout << "ax    = " << std::fixed << params.acceleration.x << endl;
   cout << "ay    = " << std::fixed << params.acceleration.y << endl;
   cout << "az    = " << std::fixed << params.acceleration.z << endl;

}


int computeDR( ComputeDRParams & params ) {

   if ( 0.0 == params.deltaTime ) {
      cout << ">>> computeDR: Invalid input parameter: deltaTime." << endl;
      return ERROR_RETURN;
   }

   DR_ALGORITHM DRalg = params.drAlgorithm;
   if ( ( DRalg < DR_ALGORITHM::DRM_STATIC ) || (  DRalg > DR_ALGORITHM::DRM_FVB ) ) {
      cout << ">>> computeDR: Unknown DR algorithm's identifier: " << static_cast<unsigned int>( DRalg ) << endl;
      return ERROR_RETURN;
   }

   if (  DR_ALGORITHM::DRM_STATIC == DRalg ) {
      cout << ">>> computeDR: static object (not moving)." << endl;
      return SUCCESS_RETURN;
   }

   cout << ">>> computeDR: object is moving." << endl;

   //Check for rotation
   bool rotating = false;
   if ( ( DR_ALGORITHM::DRM_RPW == DRalg ) || ( DR_ALGORITHM::DRM_RVW == DRalg ) ||
        ( DR_ALGORITHM::DRM_RPB == DRalg ) || ( DR_ALGORITHM::DRM_RVB == DRalg ) ) {
      if ( ( ABS(params.angularVelocity.roll) >= MIN_ROTATION_RATE ) ||
           ( ABS(params.angularVelocity.pitch) >= MIN_ROTATION_RATE ) ||
           ( ABS(params.angularVelocity.yaw) >= MIN_ROTATION_RATE ) ) {
         cout << ">>> computeDR: object is rotating." << endl;
         rotating = true;
      }
      else {
         cout << ">>> computeDR: object is NOT rotating (angular velocities too small)." << endl;
      }
   }

   //Check for acceleration
   bool accelerating = false;
   if ( ( DR_ALGORITHM::DRM_RVW == DRalg ) || ( DR_ALGORITHM::DRM_FVW == DRalg ) ||
        ( DR_ALGORITHM::DRM_RVB == DRalg ) || ( DR_ALGORITHM::DRM_FVB == DRalg ) ) {
      if ( ( ABS(params.acceleration.x) >= MIN_ACCELERATION_RATE ) ||
           ( ABS(params.acceleration.y) >= MIN_ACCELERATION_RATE ) ||
           ( ABS(params.acceleration.z) >= MIN_ACCELERATION_RATE ) ) {
         cout << ">>> computeDR: object is accelerating." << endl;
         accelerating = true;
      }
      else {
         cout << ">>> computeDR: object is NOT accelerating (linear accelerations too small)." << endl;
      }
   }

   //Check for use of body (entity relative) vel/accel coords
   bool bodyCoords = false;
   if ( DRalg >= DR_ALGORITHM::DRM_FPB ) {
      cout << ">>> computeDR: object's linear velocity and acceleration in body (entity relative) coordinates (ECS)."
      << endl;
      bodyCoords = true;
   }
   else {
      cout << ">>> computeDR: object's linear velocity and acceleration in geocentric coordinates (WCS)." << endl;
   }

   //Set up and compute intermediate variables
   cout << ">>> computeDR: Setting up and computing intermediate variables." << endl;
   double WCStoECS1Mat[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
   if ( rotating || bodyCoords ) {

      //Compute all sine and cosine values for the Euler angles
      cout << ">>> computeDR: Computing all sine and cosine values for the Euler angles." << endl;
      TrigValues trigVals = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0};
      trigVals.sinPsi   = sin( params.orientation.psi   );
      trigVals.sinTheta = sin( params.orientation.theta );
      trigVals.sinPhi   = sin( params.orientation.phi   );
      trigVals.cosPsi   = cos( params.orientation.psi   );
      trigVals.cosTheta = cos( params.orientation.theta );
      trigVals.cosPhi   = cos( params.orientation.phi   );
      trigVals.sPsisPhi = trigVals.sinPsi * trigVals.sinPhi;
      trigVals.sPsicPhi = trigVals.sinPsi * trigVals.cosPhi;
      trigVals.cPsisPhi = trigVals.cosPsi * trigVals.sinPhi;
      trigVals.cPsicPhi = trigVals.cosPsi * trigVals.cosPhi;

      //Build rotation matrix for WCS to ECS
      cout << ">>> computeDR: Building rotation matrix for WCS to ECS." << endl;
      WCStoECS1Mat[ 0 ][ 0 ] = trigVals.cosPsi * trigVals.cosTheta;
      WCStoECS1Mat[ 0 ][ 1 ] = trigVals.sinPsi * trigVals.cosTheta;
      WCStoECS1Mat[ 0 ][ 2 ] = - trigVals.sinTheta;
      WCStoECS1Mat[ 1 ][ 0 ] = ( trigVals.cPsisPhi * trigVals.sinTheta ) - trigVals.sPsicPhi;
      WCStoECS1Mat[ 1 ][ 1 ] = ( trigVals.sPsisPhi * trigVals.sinTheta ) + trigVals.cPsicPhi;
      WCStoECS1Mat[ 1 ][ 2 ] = trigVals.cosTheta * trigVals.sinPhi;
      WCStoECS1Mat[ 2 ][ 0 ] = ( trigVals.cPsicPhi * trigVals.sinTheta ) + trigVals.sPsisPhi;
      WCStoECS1Mat[ 2 ][ 1 ] = ( trigVals.sPsicPhi * trigVals.sinTheta ) - trigVals.cPsisPhi;
      WCStoECS1Mat[ 2 ][ 2 ] = trigVals.cosTheta * trigVals.cosPhi;
   }

   double rollSq = 0.0, rollPitch = 0.0, rollYaw = 0.0;
   double pitchSq = 0.0, pitchYaw = 0.0, yawSq = 0.0;
   double wMag = 0.0, wMagSq = 0.0, wMag3 = 0.0;
   double wMagT = 0.0, cosWMagT = 0.0, sinWMagT = 0.0;
   double term1 = 0.0, vIntTerm1;
   double term3 = 0.0, term3Roll, term3Pitch, term3Yaw;
   double ECS1toECS2Mat[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
   if ( rotating ) {

      //Compute initial ECS to final ECS rotation matrix
      cout << ">>> computeDR: Computing initial ECS to final ECS rotation matrix." << endl;
      double term1RollPitch, term1RollYaw, term1PitchYaw;
      rollSq = params.angularVelocity.roll * params.angularVelocity.roll;
      rollPitch = params.angularVelocity.roll * params.angularVelocity.pitch;
      rollYaw = params.angularVelocity.roll * params.angularVelocity.yaw;
      pitchSq = params.angularVelocity.pitch * params.angularVelocity.pitch;
      pitchYaw = params.angularVelocity.pitch * params.angularVelocity.yaw;
      yawSq = params.angularVelocity.yaw * params.angularVelocity.yaw;
      wMagSq = rollSq + pitchSq + yawSq;
      wMag = sqrt( wMagSq );
      wMagT = wMag * params.deltaTime;
      cosWMagT = cos( wMagT );
      term1 = ( 1.0 - cosWMagT ) / wMagSq;
      term1RollPitch = term1 * rollPitch;
      term1RollYaw = term1 * rollYaw;
      term1PitchYaw = term1 * pitchYaw;
      sinWMagT = sin( wMagT );
      term3 = sinWMagT / wMag;
      term3Roll = term3 * params.angularVelocity.roll;
      term3Pitch = term3 * params.angularVelocity.pitch;
      term3Yaw = term3 * params.angularVelocity.yaw;
      ECS1toECS2Mat[ 0 ][ 0 ] = ( term1 * rollSq ) + cosWMagT;
      ECS1toECS2Mat[ 0 ][ 1 ] = term1RollPitch + term3Yaw;
      ECS1toECS2Mat[ 0 ][ 2 ] = term1RollYaw - term3Pitch;
      ECS1toECS2Mat[ 1 ][ 0 ] = term1RollPitch - term3Yaw;
      ECS1toECS2Mat[ 1 ][ 1 ] = ( term1 * pitchSq ) + cosWMagT;
      ECS1toECS2Mat[ 1 ][ 2 ] = term1PitchYaw + term3Roll;
      ECS1toECS2Mat[ 2 ][ 0 ] = term1RollYaw + term3Pitch;
      ECS1toECS2Mat[ 2 ][ 1 ] = term1PitchYaw - term3Roll;
      ECS1toECS2Mat[ 2 ][ 2 ] = ( term1 * yawSq ) + cosWMagT;
   }

   //Compute final velocity and position
   cout << ">>> computeDR: Computing final velocity and position." << endl;
   if ( ! bodyCoords )  {
      //velocity/acceleration in WCS coords
      cout << ">>> computeDR: velocity/acceleration in WCS coords." << endl;
      if ( accelerating ) {

         //Compute final velocity vector in WCS coords
         cout << ">>> computeDR: Computing final velocity vector in WCS coords." << endl;
         VectorXYZ finalVelWCS = { 0.0, 0.0, 0.0 };
         finalVelWCS.x = params.velocity.x + ( params.acceleration.x * params.deltaTime );
         finalVelWCS.y = params.velocity.y + ( params.acceleration.y * params.deltaTime );
         finalVelWCS.z = params.velocity.z + ( params.acceleration.z * params.deltaTime );

         //Update  location in WCS coords
         cout << ">>> computeDR: Updating  location in WCS coords." << endl;
         params.location.x += ( params.velocity.x + finalVelWCS.x ) * 0.5 * params.deltaTime;
         params.location.y += ( params.velocity.y + finalVelWCS.y ) * 0.5 * params.deltaTime;
         params.location.z += ( params.velocity.z + finalVelWCS.z ) * 0.5 * params.deltaTime;

         //Update  velocity values
         cout << ">>> computeDR: Updating  velocity in WCS coords." << endl;
         params.velocity.x = finalVelWCS.x;
         params.velocity.y = finalVelWCS.y;
         params.velocity.z = finalVelWCS.z;
      }
      else {
         //not accelerating, use first-order linear projection
         cout << ">>> computeDR: not accelerating, using first-order linear projection." << endl;
         params.location.x += params.velocity.x * params.deltaTime;
         params.location.y += params.velocity.y * params.deltaTime;
         params.location.z += params.velocity.z * params.deltaTime;
      }
   }
   else {
      //velocity/acceleration in ECS coords
      cout << ">>> computeDR: velocity/acceleration in ECS coords." << endl;
      VectorXYZ vDistECS1 = { 0.0, 0.0, 0.0 };
      VectorXYZ aDistECS1 = { 0.0, 0.0, 0.0 };
      if ( rotating ) {
         //Compute the first integral of the final ECS to initial ECS rotation matrix for use with velocity
         cout << ">>> computeDR: Computing the first integral of the final ECS to initial ECS rotation matrix for use with velocity."
              << endl;
         wMag3 = wMagSq * wMag;
         vIntTerm1 = ( wMagT - sinWMagT ) / wMag3;
         //Terms 2&3 of this matrix  calc are the same as terms 3&1 of earlier initial to final ECS matrix calcs
         // so we'll reuse
         double vIntTerm1RollPitch = vIntTerm1 * rollPitch;
         double vIntTerm1RollYaw = vIntTerm1 * rollYaw;
         double vIntTerm1PitchYaw = vIntTerm1 * pitchYaw;
         double vIntTerm3Roll = term1 * params.angularVelocity.roll;
         double vIntTerm3Pitch = term1 * params.angularVelocity.pitch;
         double vIntTerm3Yaw = term1 * params.angularVelocity.yaw;
         double vIntECS2toECS1Mat[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
         vIntECS2toECS1Mat[0][0] = ( vIntTerm1 * rollSq ) + term3;
         vIntECS2toECS1Mat[0][1] = vIntTerm1RollPitch - vIntTerm3Yaw;
         vIntECS2toECS1Mat[0][2] = vIntTerm1RollYaw + vIntTerm3Pitch;
         vIntECS2toECS1Mat[1][0] = vIntTerm1RollPitch + vIntTerm3Yaw;
         vIntECS2toECS1Mat[1][1] = ( vIntTerm1 * pitchSq ) + term3;
         vIntECS2toECS1Mat[1][2] = vIntTerm1PitchYaw - vIntTerm3Roll;
         vIntECS2toECS1Mat[2][0] = vIntTerm1RollYaw - vIntTerm3Pitch;
         vIntECS2toECS1Mat[2][1] = vIntTerm1PitchYaw + vIntTerm3Roll;
         vIntECS2toECS1Mat[2][2] =  ( vIntTerm1 * yawSq ) + term3;

         //Compute movement in initial entity coordinates due to initial velocity (ignoring acceleration)
         cout << ">>> computeDR: Computing movement in initial entity coordinates due to initial velocity (ignoring acceleration) - rotating."
              << endl;
         vDistECS1.x = ( params.velocity.x * vIntECS2toECS1Mat[0][0] ) +
                       ( params.velocity.y * vIntECS2toECS1Mat[0][1] ) +
                       ( params.velocity.z * vIntECS2toECS1Mat[0][2]);
         vDistECS1.y = ( params.velocity.x * vIntECS2toECS1Mat[1][0] ) +
                       ( params.velocity.y * vIntECS2toECS1Mat[1][1] ) +
                       ( params.velocity.z * vIntECS2toECS1Mat[1][2]);
         vDistECS1.z = ( params.velocity.x * vIntECS2toECS1Mat[2][0] ) +
                       ( params.velocity.y * vIntECS2toECS1Mat[2][1] ) +
                       ( params.velocity.z * vIntECS2toECS1Mat[2][2]);
      }
      else {
         //not rotating
         //Compute movement in initial entity coordinates due to initial velocity (ignoring acceleration)
         cout << ">>> computeDR: Computing movement in initial entity coordinates due to initial velocity (ignoring acceleration) - not rotating."
              << endl;
         vDistECS1.x = params.velocity.x * params.deltaTime;
         vDistECS1.y = params.velocity.y * params.deltaTime;
         vDistECS1.z = params.velocity.z * params.deltaTime;
      }

      if ( accelerating ) {

         //Update velocity values in final ECS coords
         cout << ">>> computeDR: Updating velocity values in final ECS coords." << endl;
         params.velocity.x += params.acceleration.x * params.deltaTime;
         params.velocity.y += params.acceleration.y * params.deltaTime;
         params.velocity.z += params.acceleration.z * params.deltaTime;

         if ( rotating ) {
            //Compute the first integral of the final ECS to initial ECS rotation matrix times time for use with acceleration
            cout << ">>> computeDR: Computing the first integral of the final ECS to initial ECS rotation matrix times time for use with acceleration."
                 << endl;

            double wMag4 = wMag3 * wMag;
            double aIntTerm2Top = cosWMagT + ( wMagT * sinWMagT ) - 1.0;
            double aIntTerm1 = ( ( 0.5 * wMagT * wMagT ) - aIntTerm2Top ) / wMag4;
            double aIntTerm2 = aIntTerm2Top / wMagSq;
            double aIntTerm3 = ( sinWMagT - ( wMagT * cosWMagT ) ) / wMag3;
            double aIntTerm1RollPitch = aIntTerm1 * rollPitch;
            double aIntTerm1RollYaw = aIntTerm1 * rollYaw;
            double aIntTerm1PitchYaw = aIntTerm1 * pitchYaw;
            double aIntTerm3Roll = aIntTerm3 * params.angularVelocity.roll;
            double aIntTerm3Pitch = aIntTerm3 * params.angularVelocity.pitch;
            double aIntTerm3Yaw = aIntTerm3 * params.angularVelocity.yaw;
            double aIntECS2toECS1Mat[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
            aIntECS2toECS1Mat[0][0] = ( aIntTerm1 * rollSq ) + aIntTerm2;
            aIntECS2toECS1Mat[0][1] = aIntTerm1RollPitch - aIntTerm3Yaw;
            aIntECS2toECS1Mat[0][2] = aIntTerm1RollYaw + aIntTerm3Pitch;
            aIntECS2toECS1Mat[1][0] = aIntTerm1RollPitch + aIntTerm3Yaw;
            aIntECS2toECS1Mat[1][1] = ( aIntTerm1 * pitchSq ) + aIntTerm2;
            aIntECS2toECS1Mat[1][2] = aIntTerm1PitchYaw - aIntTerm3Roll;
            aIntECS2toECS1Mat[2][0] = aIntTerm1RollYaw - aIntTerm3Pitch;
            aIntECS2toECS1Mat[2][1] = aIntTerm1PitchYaw + aIntTerm3Roll;
            aIntECS2toECS1Mat[2][2] = (aIntTerm1 * yawSq) + aIntTerm2;

            //Compute movement in initial entity coordinates due to acceleration (ignoring initial velocity)
            cout << ">>> computeDR: Computing movement in initial entity coordinates due to acceleration (ignoring initial velocity) - rotating."
                 << endl;
            aDistECS1.x = ( params.acceleration.x * aIntECS2toECS1Mat[0][0] ) +
                          ( params.acceleration.y * aIntECS2toECS1Mat[0][1] ) +
                          ( params.acceleration.z * aIntECS2toECS1Mat[0][2] );

            aDistECS1.y = ( params.acceleration.x * aIntECS2toECS1Mat[1][0] ) +
                          ( params.acceleration.y * aIntECS2toECS1Mat[1][1] ) +
                          ( params.acceleration.z * aIntECS2toECS1Mat[1][2] ) ;

            aDistECS1.z = ( params.acceleration.x * aIntECS2toECS1Mat[2][0] ) +
                          ( params.acceleration.y * aIntECS2toECS1Mat[2][1] ) +
                          ( params.acceleration.z * aIntECS2toECS1Mat[2][2] );
         }
         else {
            //not rotating
            //Compute movement in initial entity coordinates due to acceleration (ignoring initial velocity)
            cout << ">>> computeDR: Computing movement in initial entity coordinates due to acceleration (ignoring initial velocity) - not rotating."
                 << endl;
            double halfDeltaTimeSq = 0.5 * params.deltaTime * params.deltaTime;
            aDistECS1.x = params.acceleration.x * halfDeltaTimeSq;
            aDistECS1.y = params.acceleration.y * halfDeltaTimeSq;
            aDistECS1.z = params.acceleration.z * halfDeltaTimeSq;
         }
      }
      else {
         //not accelerating
         aDistECS1.x = 0.0;
         aDistECS1.y = 0.0;
         aDistECS1.z = 0.0;
      }

      //Compute the total movement in initial ECS coords
      cout << ">>> computeDR: Computing the total movement in initial ECS coords." << endl;
      VectorXYZ distECS1 = { 0.0, 0.0, 0.0 };
      distECS1.x = vDistECS1.x + aDistECS1.x;
      distECS1.y = vDistECS1.y + aDistECS1.y;
      distECS1.z = vDistECS1.z + aDistECS1.z;

      //Convert the movement in initial ECS coords to update  location in WCS coords
      //Use transposed WCStoECS1Mat rather building an ECS1toWCSMat
      cout << ">>> computeDR: Converting the movement in initial ECS coords to update  location in WCS coords."
           << endl;
      params.location.x += ( distECS1.x * WCStoECS1Mat[0][0] ) +
                           ( distECS1.y * WCStoECS1Mat[1][0] ) +
                           ( distECS1.z * WCStoECS1Mat[2][0] );
      params.location.y += ( distECS1.x * WCStoECS1Mat[0][1] ) +
                           ( distECS1.y * WCStoECS1Mat[1][1] ) +
                           ( distECS1.z * WCStoECS1Mat[2][1] );
      params.location.z += ( distECS1.x * WCStoECS1Mat[0][2] ) +
                           ( distECS1.y * WCStoECS1Mat[1][2] ) +
                           ( distECS1.z * WCStoECS1Mat[2][2] );

   } //end of velocity/acceleration in ECS coords

   //Compute final Euler angles
   if ( rotating ) {
      cout << ">>> computeDR: Computing final Euler angles." << endl;
      //Compute theta
      cout << ">>> computeDR: Computing theta." << endl;
      Orientation finalOrient = { 0.0, 0.0, 0.0 };
      double WCStoECS2Mat[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
      WCStoECS2Mat[0][2] = ECS1toECS2Mat[0][0] * WCStoECS1Mat[0][2] +
                           ECS1toECS2Mat[0][1] * WCStoECS1Mat[1][2] +
                           ECS1toECS2Mat[0][2] * WCStoECS1Mat[2][2];
      //trap round-off domain errors
      if ( WCStoECS2Mat[0][2] >= 1.0 ) {
         finalOrient.theta = HALFpi;
      }
      else if ( WCStoECS2Mat[0][2] <= -1.0 ) {
         finalOrient.theta = NEGHALFpi;
      }
      else {
         finalOrient.theta = -asin( WCStoECS2Mat[0][2] );
      }

      double cosTheta = cos( finalOrient.theta );
      double acosArg = 0.0;
      if ( cosTheta != 0.0 ) {
         //normal case: theta is not + or - pi/2
         cout << ">>> computeDR: normal case: theta is not + or - pi/2." << endl;
         //Compute Psi
         cout << ">>> computeDR: Computing psi." << endl;
         WCStoECS2Mat[0][0] = ECS1toECS2Mat[0][0] * WCStoECS1Mat[0][0] +
                              ECS1toECS2Mat[0][1] * WCStoECS1Mat[1][0] +
                              ECS1toECS2Mat[0][2] * WCStoECS1Mat[2][0];
         acosArg =  WCStoECS2Mat[0][0] / cosTheta;
         //trap round-off domain errors
         if ( acosArg >= 1.0 ) {
            finalOrient.psi = 0.0;
         }
         else if ( acosArg <= -1.0 ) {
            finalOrient.psi = pi;
         }
         else {
            finalOrient.psi = acos( acosArg );
         }

         //Normalize Psi to range of + or - pi radians
         cout << ">>> computeDR: Normalizing Psi to range of + or - pi radians." << endl;
         WCStoECS2Mat[0][1] = ECS1toECS2Mat[0][0] * WCStoECS1Mat[0][1] +
                              ECS1toECS2Mat[0][1] * WCStoECS1Mat[1][1] +
                              ECS1toECS2Mat[0][2] * WCStoECS1Mat[2][1];
         if ( WCStoECS2Mat[0][1] < 0.0 ) {
            finalOrient.psi = -finalOrient.psi;
         }

         //Compute Phi
         cout << ">>> computeDR: Computing phi." << endl;
         WCStoECS2Mat[2][2] = ECS1toECS2Mat[2][0] * WCStoECS1Mat[0][2] +
                              ECS1toECS2Mat[2][1] * WCStoECS1Mat[1][2] +
                              ECS1toECS2Mat[2][2] * WCStoECS1Mat[2][2];
         acosArg = WCStoECS2Mat[2][2] / cosTheta;
         //trap round-off domain errors
         if ( acosArg >= 1.0 ) {
            finalOrient.phi = 0.0;
         }
         else if ( acosArg <= -1.0 ) {
            finalOrient.phi = pi;
         }
         else {
            finalOrient.phi = acos( acosArg );
         }

         //Normalize Phi to range of + or - pi radians
         cout << ">>> computeDR: Normalizing Phi to range of + or - pi radians." << endl;
         WCStoECS2Mat[1][2] = ECS1toECS2Mat[1][0] * WCStoECS1Mat[0][2] +
                              ECS1toECS2Mat[1][1] * WCStoECS1Mat[1][2] +
                              ECS1toECS2Mat[1][2] * WCStoECS1Mat[2][2];
         if ( WCStoECS2Mat[1][2] < 0.0 ) {
            finalOrient.phi = -finalOrient.phi;
         }
      }
      else {
         //special case: theta is + or - pi/2
         cout << ">>> computeDR: special case: theta is + or - pi/2." << endl;
         // Compute psi
         cout << ">>> computeDR: Computing psi." << endl;
         WCStoECS2Mat[1][1] = ECS1toECS2Mat[1][0] * WCStoECS1Mat[0][1] +
                              ECS1toECS2Mat[1][1] * WCStoECS1Mat[1][1] +
                              ECS1toECS2Mat[1][2] * WCStoECS1Mat[2][1];
         acosArg = WCStoECS2Mat[1][1];
         //trap round-off domain errors
         if ( acosArg >= 1.0 ) {
            finalOrient.psi = 0.0;
         }
         else if ( acosArg <= -1.0 ) {
            finalOrient.psi = pi;
         }
         else {
            finalOrient.psi = acos( acosArg );
         }
         //Normalize Psi to range of + or - pi radians
         cout << ">>> computeDR: Normalizing Psi to range of + or - pi radians." << endl;
         WCStoECS2Mat[1][0] = ECS1toECS2Mat[1][0] * WCStoECS1Mat[0][0] +
                              ECS1toECS2Mat[1][1] * WCStoECS1Mat[1][0] +
                              ECS1toECS2Mat[1][2] * WCStoECS1Mat[2][0];
         if ( WCStoECS2Mat[1][0] > 0.0 ) {
            finalOrient.psi = - finalOrient.psi;
         }

         //Set phi to zero - no further rotations required
         cout << ">>> computeDR: Setting phi to zero - no further rotations required." << endl;
         finalOrient.phi = 0.0;
      }

      //Update  Euler angle values
      params.orientation.psi   = finalOrient.psi;
      params.orientation.theta = finalOrient.theta;
      params.orientation.phi   = finalOrient.phi;
   } // end of compute final Euler angles

   //Updating articulation parameter values not implemented
   cout << ">>> computeDR: Updating articulation parameter values not implemented." << endl;

   return SUCCESS_RETURN;
}

int main( int argc, char *argv[] ) {
   printHeader();

   //EntityStatePDU entityStatePdu;

   int retVal = SUCCESS_RETURN;
   ComputeDRParams params;

   // Testing algorithm DRM_RVW (4)
   params.location.x = -6378137;
   params.location.y = -5;
   params.location.z = -10;

   params.orientation.psi = 15.0 * DtoR;
   params.orientation.theta = 20.0 * DtoR;
   params.orientation.phi = 25.0 * DtoR;

   params.velocity.x = 30;
   params.velocity.y = 35;
   params.velocity.z = 40;

   params.drAlgorithm = DR_ALGORITHM::DRM_RVW;
   //params.drAlgorithm = static_cast<DR_ALGORITHM>( 10 );

   params.angularVelocity.roll = 60.0 * DtoR;
   params.angularVelocity.pitch = 65.0 * DtoR;
   params.angularVelocity.yaw = 70.0 * DtoR;

   params.acceleration.x = -80.0;
   params.acceleration.y = -85.0;
   params.acceleration.z = -90.0;

   params.deltaTime = 0.5;

   cout << endl << "Calling computeDR with parameters: algorithm's ID = "
        << static_cast<unsigned int>( params.drAlgorithm )
        << ", deltaTime = " << params.deltaTime << endl;
   printValues( "Initial", params );
   printDRParams( params );
   retVal = computeDR( params );
   cout << "computeDR called with: parameters: algorithm's ID = "
        << static_cast<unsigned int>( params.drAlgorithm )
        << ", deltaTime = " << params.deltaTime << " - returned: " << retVal << endl;
   printValues( "Final", params );

   // Testing algorithm DRM_RVB (8)
   params.location.x = 6378137;
   params.location.y = 5;
   params.location.z = 10;

   params.orientation.psi = 15.0 * DtoR;
   params.orientation.theta = 20.0 * DtoR;
   params.orientation.phi = 25.0 * DtoR;

   params.velocity.x = 30;
   params.velocity.y = 35;
   params.velocity.z = 40;

   params.drAlgorithm = DR_ALGORITHM::DRM_RVB;
   //params.drAlgorithm = static_cast<DR_ALGORITHM>( 10 );

   params.angularVelocity.roll = -60.0 * DtoR;
   params.angularVelocity.pitch = -65.0 * DtoR;
   params.angularVelocity.yaw = -70.0 * DtoR;

   params.acceleration.x = -80.0;
   params.acceleration.y = -85.0;
   params.acceleration.z = -90.0;

   params.deltaTime = 0.5;

   cout << endl << "Calling computeDR with parameters: algorithm's ID = "
        << static_cast<unsigned int>( params.drAlgorithm )
        << ", deltaTime = " << params.deltaTime << endl;
   printValues( "Initial", params );
   printDRParams( params );
   retVal = computeDR( params );
   cout << "computeDR called with: parameters: algorithm's ID = "
        << static_cast<unsigned int>( params.drAlgorithm )
        << ", deltaTime = " << params.deltaTime << " - returned: " << retVal << endl;
   printValues( "Final", params );

   return 0;
}
