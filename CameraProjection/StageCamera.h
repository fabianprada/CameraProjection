#ifndef STAGE_CAMERA_INCLUDED
#define STAGE_CAMERA_INCLUDED
#include <Misha/Image.h>
#include "SimpleMesh.h"

class StageCamera{
public:
	int camId;
	//Read From Input File
	Point3D<double> cameraToWorldAngleAxisRotation;
	Point3D<double> cameraToWorldTranslation;
	double fxy;

	double fx, fy, cx, cy;
	double distortionParams[5];
	Image<Point3D<float>> image;
	Image<Point3D<float>> silhoette;
	int imageWidth, imageHeight;

	//Extrinsic
	SquareMatrix<double, 3> worldToCameraRotation;
	Point3D<double> worldToCameraTranslation;
	Point3D<double> position;
	Point3D<double> forward;
	Point3D<double> up;
	Point3D<double> right;

	//GL Visualization
	SquareMatrix<double, 4> glWorldToCamera;
	SquareMatrix<double, 4> glProjectionMatrix;

	//Oclusion test
	Image<float> depthBuffer;
	int depthBufferWidth, depthBufferHeight;
	double depthRange;
	double nearPlane;
	double farPlane;
	//Discontinuity test
	Image<float> discontinuityScore;

	void Initialize(const std::vector<Point3D<double>> & meshVertices);
	bool TextureCoordinates(const Point3D<double> & point, Point2D<double> & st, float & discontinuityValue, bool applyCameraDistortion = false) const;
	bool TextureCoordinatesAndDepth(const Point3D<double> & point, Point2D<double> & st, float & depth, bool applyCameraDistortion = false) const;
};

// Takes a point in 3D and returns the texture coordinate of its projection to camera.
// If the projection lies inside the image the texture value is in [0,1]x[0,1]
// Returns true if the texture value is in [0,1]x[0,1] and the point is not occluded. Return false otherwise
bool StageCamera::TextureCoordinates(const Point3D<double> & point, Point2D<double> & st, float & discontinuityValue, bool applyCameraDistortion) const{
	Point3D<double> w = worldToCameraRotation*point + worldToCameraTranslation;
	Point2D<double> nw(w[0] / w[2], w[1] / w[2]);

	if (applyCameraDistortion){//Alreay applied to RGBMasks
		Point2D<double> nw_copy(nw[0], nw[1]);
		double r2 = nw[0] * nw[0] + nw[1] * nw[1];
		double poly_coeff = (1.0 + r2 *(distortionParams[0] + r2*(distortionParams[1] + r2*distortionParams[4])));
		nw[0] = poly_coeff*nw_copy[0] + 2.0*distortionParams[2] * nw_copy[0] * nw_copy[1] + distortionParams[3] * (r2 + 2.0* nw_copy[0] * nw_copy[0]);
		nw[1] = poly_coeff*nw_copy[1] + 2.0*distortionParams[3] * nw_copy[0] * nw_copy[1] + distortionParams[2] * (r2 + 2.0* nw_copy[1] * nw_copy[1]);
	}

	st[0] = (nw[0] * fx + cx) / static_cast<double>(imageWidth);
	st[1] = (nw[1] * fy + cy) / static_cast<double>(imageHeight);
	if (st[0] > 0.0 && st[0]<1.0 && st[1]>0.0 && st[1] < 1.0){
		double ez = depthBuffer.sample(st[0] * static_cast<double>(depthBufferWidth)-0.5, st[1] * static_cast<double>(depthBufferHeight)-0.5);
		if (abs(ez - w[2]) > 0.001f*depthRange)  return false; //Visibility Test
#if 1
		discontinuityValue = discontinuityScore(int(st[0] * static_cast<double>(depthBufferWidth)), int(st[1] * static_cast<double>(depthBufferHeight)));
#else
		discontinuityValue = discontinuityScore.sample(st[0] * static_cast<double>(depthBufferWidth) - 0.5, st[1] * static_cast<double>(depthBufferHeight) - 0.5);
#endif 
		
		//int neighbourRadii = 5;
		//for (int l = -1; l <= 1; l++) for (int k = -1; k <= 1; k++){
		//	double _nz = depthBuffer.sample(st[0] * static_cast<double>(depthBufferWidth)-0.5 + static_cast<double>(neighbourRadii*l), st[1] * static_cast<double>(depthBufferHeight)-0.5 + static_cast<double>(neighbourRadii*k))*2.0 - 1.0;
		//	double _ez = -(2.0*(farPlane*nearPlane) / (farPlane - nearPlane)) / (-(farPlane + nearPlane) / (farPlane - nearPlane) + _nz);
		//	//printf("Visibility Test %g %g \n", abs(ez - w[2]), 0.001f*depthRange);
		//	//printf("Smoothness Test %g %g \n", abs(ez - _ez), 0.001f*depthRange);
		//	if (abs(_ez - w[2]) > 0.05f*depthRange)  return false; //Continuity Test
		//}
		return true;
	}
	else{
		return false;
	}
}


bool StageCamera::TextureCoordinatesAndDepth(const Point3D<double> & point, Point2D<double> & st, float & depth, bool applyCameraDistortion) const {
	Point3D<double> w = worldToCameraRotation*point + worldToCameraTranslation;
	Point2D<double> nw(w[0] / w[2], w[1] / w[2]);

	if (applyCameraDistortion) {//Alreay applied to RGBMasks
		Point2D<double> nw_copy(nw[0], nw[1]);
		double r2 = nw[0] * nw[0] + nw[1] * nw[1];
		double poly_coeff = (1.0 + r2 *(distortionParams[0] + r2*(distortionParams[1] + r2*distortionParams[4])));
		nw[0] = poly_coeff*nw_copy[0] + 2.0*distortionParams[2] * nw_copy[0] * nw_copy[1] + distortionParams[3] * (r2 + 2.0* nw_copy[0] * nw_copy[0]);
		nw[1] = poly_coeff*nw_copy[1] + 2.0*distortionParams[3] * nw_copy[0] * nw_copy[1] + distortionParams[2] * (r2 + 2.0* nw_copy[1] * nw_copy[1]);
	}
	st[0] = (nw[0] * fx + cx) / static_cast<double>(imageWidth);
	st[1] = (nw[1] * fy + cy) / static_cast<double>(imageHeight);
	if (st[0] > 0.0 && st[0]<1.0 && st[1]>0.0 && st[1] < 1.0) {
		double ez = depthBuffer.sample(st[0] * static_cast<double>(depthBufferWidth) - 0.5, st[1] * static_cast<double>(depthBufferHeight) - 0.5);
		if (abs(ez - w[2]) > 0.001f*depthRange) {//Visibility Test
			depth = FLT_MAX;
			return false;
		}
		else {
			depth = ez;
			return true;
		}

	}
	else {
		return false;
	}
}

Point3D<double> AngleAxisRotation(const Point3D<double> point, const Point3D<double> angleAxis){
	double angle = Point3D<double>::Length(angleAxis);
	if (angle){
		Point3D<double> axis = angleAxis / angle;
		double proj = Point3D<double>::Dot(point, axis);
		Point3D<double> orth = point - axis*proj;
		Point3D<double> Jorth = Point3D<double>::CrossProduct(axis, orth);
		return (axis*proj + orth*cos(angle) + Jorth*sin(angle));
	}
	else return point;
}

void StageCamera::Initialize(const std::vector<Point3D<double>> & meshVertices){ //Parse xml

	//Set world to camera transforms
	if (0) printf("World to camera translation: \n");
	if (0) printf(" %f %f %f \n", worldToCameraTranslation[0], worldToCameraTranslation[1], worldToCameraTranslation[2]);
	if (0) printf("World to camera rotation: \n");
	if (0) for (int k = 0; k < 3; k++) printf(" %f %f %f \n", worldToCameraRotation.coords[0][k], worldToCameraRotation.coords[1][k], worldToCameraRotation.coords[2][k]);


	//Set camera GL frame
	SquareMatrix<double, 3> worldToCameraRotationT = worldToCameraRotation.transpose();
	position = worldToCameraRotationT*(-worldToCameraTranslation);
	forward = worldToCameraRotationT*(Point3D<double>(0, 0, 1));
	right = worldToCameraRotationT*(Point3D<double>(1, 0, 0));
	up = worldToCameraRotationT*(Point3D<double>(0, -1, 0));

	//Identify near and far planes
	double closest = DBL_MAX;
	double farthest = 0.0;
	for (int i = 0; i < meshVertices.size(); i++){
		double projection = Point3D<double>::Dot(Point3D<double>(meshVertices[i]) - position, forward);
		if (projection > 0.0){
			closest = std::min<double>(projection, closest);
			farthest = std::max<double>(projection, farthest);
		}
	}

	double maxSeparation = farthest - closest;
	nearPlane = closest - maxSeparation*0.5;
	farPlane = farthest + maxSeparation*0.5;
	depthRange = farPlane - nearPlane;

	if (0) printf("Near Plane %f \n", nearPlane);
	if (0) printf("Far Plane %f \n", farPlane);
	if (0) printf("Eye Depth Range %f \n", depthRange);

	//Set GL Transformations 
	glWorldToCamera.coords[0][0] = worldToCameraRotation.coords[0][0];
	glWorldToCamera.coords[0][1] = -worldToCameraRotation.coords[0][1];
	glWorldToCamera.coords[0][2] = -worldToCameraRotation.coords[0][2];
	glWorldToCamera.coords[0][3] = 0.0;

	glWorldToCamera.coords[1][0] = worldToCameraRotation.coords[1][0];
	glWorldToCamera.coords[1][1] = -worldToCameraRotation.coords[1][1];
	glWorldToCamera.coords[1][2] = -worldToCameraRotation.coords[1][2];
	glWorldToCamera.coords[1][3] = 0.0;

	glWorldToCamera.coords[2][0] = worldToCameraRotation.coords[2][0];
	glWorldToCamera.coords[2][1] = -worldToCameraRotation.coords[2][1];
	glWorldToCamera.coords[2][2] = -worldToCameraRotation.coords[2][2];
	glWorldToCamera.coords[2][3] = 0.0;

	glWorldToCamera.coords[3][0] = worldToCameraTranslation[0];
	glWorldToCamera.coords[3][1] = -worldToCameraTranslation[1];
	glWorldToCamera.coords[3][2] = -worldToCameraTranslation[2];
	glWorldToCamera.coords[3][3] = 1.0;

	glProjectionMatrix.coords[0][0] = 2.0*fx / static_cast<double>(imageWidth);
	glProjectionMatrix.coords[0][1] = 0.0;
	glProjectionMatrix.coords[0][2] = 0.0;
	glProjectionMatrix.coords[0][3] = 0.0;

	glProjectionMatrix.coords[1][0] = 0.0;
	glProjectionMatrix.coords[1][1] = 2.0*fy / static_cast<double>(imageHeight);
	glProjectionMatrix.coords[1][2] = 0.0;
	glProjectionMatrix.coords[1][3] = 0.0;

	glProjectionMatrix.coords[2][0] = -2.0*cx / static_cast<double>(imageWidth)+1.0;
	glProjectionMatrix.coords[2][1] = 2.0*cy / static_cast<double>(imageHeight)-1.0;
	glProjectionMatrix.coords[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
	glProjectionMatrix.coords[2][3] = -1.0; //Holds if the camera is viewing in the z direction

	glProjectionMatrix.coords[3][0] = 0.0;
	glProjectionMatrix.coords[3][1] = 0.0;
	glProjectionMatrix.coords[3][2] = -2.0*(farPlane*nearPlane) / (farPlane - nearPlane);
	glProjectionMatrix.coords[3][3] = 0.0;

}


#endif // !STAGE_CAMERA_INCLUDED



