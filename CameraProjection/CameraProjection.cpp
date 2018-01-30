#include <stdlib.h>
#include "SimpleMesh.h"
#include "StageCamera.h"

void CPURaster(const StageCamera & camera, const std::vector<Point3D<double>> & vertices, const std::vector<TriangleIndex> & triangles, Image<float> & ZBuffer, int width, int height){

	SquareMatrix<double, 3> projectionMatrix;
	projectionMatrix(0, 0) = camera.fx;
	projectionMatrix(1, 1) = camera.fy;
	projectionMatrix(2, 0) = camera.cx;
	projectionMatrix(2, 1) = camera.cy;
	projectionMatrix(1, 0) = projectionMatrix(0, 1) = projectionMatrix(0, 2) = projectionMatrix(1, 2) = 0;
	projectionMatrix(2, 2) = 1.0;

	double scaleFactor_w = double(width) / double(camera.imageWidth);
	double scaleFactor_h = double(height) / double(camera.imageHeight);
	std::vector<double> zValue(vertices.size());
	std::vector<Point2D<double>> imageVertices(vertices.size());

	for (int i = 0; i < vertices.size(); i++){
		Point3D<double > eyeCoordinates = camera.worldToCameraRotation*vertices[i] + camera.worldToCameraTranslation;
		zValue[i] = eyeCoordinates[2];
		Point3D<double > projectiveCoordinates = projectionMatrix * eyeCoordinates;
		imageVertices[i] = Point2D<double >(projectiveCoordinates[0] * scaleFactor_w, projectiveCoordinates[1] * scaleFactor_h) / projectiveCoordinates[2];
		//printf("%f %f \n", imageVertices[i][0], imageVertices[i][1]);
	}

	ZBuffer.resize(width, height);

	for (int i = 0; i < ZBuffer.size(); i++) ZBuffer[i] = FLT_MAX;
	for (int t = 0; t < triangles.size(); t++){

		Point2D< double > tPos[3];
		for (int i = 0; i < 3; i++) tPos[i] = imageVertices[triangles[t][i]];

		//BBox
		double fminx = std::min< double >(std::min< double >(tPos[0][0], tPos[1][0]), tPos[2][0]);
		double fminy = std::min< double >(std::min< double >(tPos[0][1], tPos[1][1]), tPos[2][1]);
		double fmaxx = std::max< double >(std::max< double >(tPos[0][0], tPos[1][0]), tPos[2][0]);
		double fmaxy = std::max< double >(std::max< double >(tPos[0][1], tPos[1][1]), tPos[2][1]);

		int minx = static_cast<int>(floor(fminx));
		int miny = static_cast<int>(floor(fminy));
		int maxx = static_cast<int>(ceil(fmaxx));
		int maxy = static_cast<int>(ceil(fmaxy));
		SquareMatrix< double, 2 > parametrizationMap;

		parametrizationMap.coords[0][0] = tPos[1][0] - tPos[0][0];
		parametrizationMap.coords[0][1] = tPos[1][1] - tPos[0][1];
		parametrizationMap.coords[1][0] = tPos[2][0] - tPos[0][0];
		parametrizationMap.coords[1][1] = tPos[2][1] - tPos[0][1];
		SquareMatrix< double, 2 > baricentricMap = parametrizationMap.inverse();
		for (int j = miny; j < maxy; j++){
			for (int i = minx; i < maxx; i++){
				if (i >= 0 && i < width && j >= 0 && j < height){
					Point2D< double > texel_pos = Point2D< double >(static_cast<double>(i)+0.5f, static_cast<double>(j)+0.5f) - tPos[0];
					Point2D< double > baricentricCoord = baricentricMap*texel_pos;
					if (baricentricCoord[0] >= 0.f && baricentricCoord[1] >= 0.f && (baricentricCoord[0] + baricentricCoord[1]) <= 1.f){
						double z = zValue[triangles[t][0]] * (1.0 - baricentricCoord[0] - baricentricCoord[1]) + zValue[triangles[t][1]] * baricentricCoord[0] + zValue[triangles[t][2]] * baricentricCoord[1];
						ZBuffer(i, j) = std::min<float>(float(z), ZBuffer(i, j));
					}
				}
			}
		}
	}
}


void InitializeCamera(const SimpleMesh & mesh, StageCamera & camera, bool useDiscontinuity){

	camera.Initialize(mesh.vertices);
	{//Initialize Depth Buffer
		CPURaster(camera, mesh.vertices, mesh.triangles, camera.depthBuffer, 1600, 1600);
		camera.discontinuityScore.resize(camera.depthBuffer.width(), camera.depthBuffer.height());
		for (int i = 0; i < camera.discontinuityScore.width(); i++)for (int j = 0; j < camera.discontinuityScore.height(); j++) camera.discontinuityScore(i, j) = 0.f;
		if (useDiscontinuity){//Identify Discontinuity

			int _width = camera.depthBuffer.width();
			int _height = camera.depthBuffer.height();
			Image<float> eyeZ;
			eyeZ.resize(_width, _height);
			for (int i = 0; i < _width; i++)for (int j = 0; j < _height; j++){
				eyeZ(i, j) = camera.depthBuffer(i, j);
			}

			if (0){
				float minEyeZ = FLT_MAX;
				float maxEyeZ = 0.0;
				for (int i = 0; i < _width; i++)for (int j = 0; j < _height; j++){
					if (eyeZ(i, j) != FLT_MAX){
						minEyeZ = std::min<float>(minEyeZ, eyeZ(i, j));
						maxEyeZ = std::max<float>(maxEyeZ, eyeZ(i, j));
					}
				}
				printf("Min Eye Z = %f \n", minEyeZ);
				printf("Max Eye Z = %f \n", maxEyeZ);
				Image<Point3D<float>> eyeZImage;
				eyeZImage.resize(_width, _height);
				for (int i = 0; i < _width; i++)for (int j = 0; j <_height; j++) eyeZImage(i, j) = Point3D<float>(1.f, 1.f, 1.f)*(eyeZ(i, j) - minEyeZ) / (maxEyeZ - minEyeZ);
				char eyeZImageName[256];
				sprintf(eyeZImageName, "EyeZ-Cam%04d.png", camera.camId);
				eyeZImage.write(eyeZImageName);

			}
			float  max_discontinuity_score = 10.f;
			float currentDiscontinuityScore = max_discontinuity_score;

			for (int i = 0; i < _width; i++)for (int j = 0; j < _height; j++){
				for (int di = -1; di <= 1; di++) for (int dj = -1; dj <= 1; dj++){
					int pi = std::min<int>(_width - 1, std::max<int>(0, i + di));
					int pj = std::min<int>(_height - 1, std::max<int>(0, j + dj));
					if (abs(eyeZ(i, j) - eyeZ(pi, pj)) >  0.03f*camera.depthRange) camera.discontinuityScore(i, j) = currentDiscontinuityScore;
				}
			}



			for (int padIter = 0; padIter < max_discontinuity_score; padIter++){
				Image<float> oldDiscontinuousScore = camera.discontinuityScore;
				currentDiscontinuityScore = std::max<float>(currentDiscontinuityScore - 1.f, 0.f);
				for (int i = 0; i < _width; i++)for (int j = 0; j < _height; j++){
					if (oldDiscontinuousScore(i, j) > 1.f){
						for (int di = 0; di <= 1; di++) for (int dj = 0; dj <= 1; dj++){
							int pi = std::min<int>(_width - 1, std::max<int>(0, i + 2 * di - 1));
							int pj = std::min<int>(_height - 1, std::max<int>(0, j + 2 * dj - 1));
							if (oldDiscontinuousScore(pi, pj) == 0.f) camera.discontinuityScore(pi, pj) = currentDiscontinuityScore;
						}
					}
				}
			}

			if (0){
				Image<Point3D<float>> discontinuityImage;
				discontinuityImage.resize(camera.depthBuffer.width(), camera.depthBuffer.height());
				for (int i = 0; i < camera.depthBuffer.width(); i++)for (int j = 0; j < camera.depthBuffer.height(); j++) discontinuityImage(i, j) = Point3D<float>(1.f, 1.f, 1.f) *camera.discontinuityScore(i, j) / 5.f;
				char discontinuityImageName[256];
				sprintf(discontinuityImageName, "Discontinuity-Cam%04d.png", camera.camId);
				discontinuityImage.write(discontinuityImageName);
			}
		}
	}

	if (0) printf("Depth Buffer Size %d x %d \n", camera.depthBuffer.width(), camera.depthBuffer.height());
	camera.depthBufferWidth = camera.depthBuffer.width();
	camera.depthBufferHeight = camera.depthBuffer.height();
}

void TextureSampling(const StageCamera & camera, ColoredMesh & outputMesh){

	for (int i = 0; i < outputMesh.vertices.size(); i++){
		Point2D<double> st;
		float discontinuityScore;
		bool visibility = camera.TextureCoordinates(outputMesh.vertices[i], st, discontinuityScore, false);
		if (visibility){
			Point3D<float> direction = outputMesh.vertices[i] - camera.position;
			if (Point3D<float>::Length(direction)) direction /= Point3D<float>::Length(direction);
			Point3D<float> tNormal = outputMesh.normals[i];
			double confidenceValue = double(Point3D<float>::Dot(tNormal, -direction));
			if (confidenceValue > 0){
				outputMesh.colors[i] = camera.image.sample(static_cast<double>(camera.image.width())*st[0] - 0.5, static_cast<double>(camera.image.height())*st[1] - 0.5)*255.0;
			}
		}
	}
}

int ParseCameraFile(const char * calibrationFileName, const char * imageName, StageCamera & camera) {

	FILE * calibFile = fopen(calibrationFileName, "r");
	if (!calibFile) {
		printf("Calibration filed not found! \n");
		return 0;
	}

	fscanf(calibFile, "%d", &camera.camId);
	fscanf(calibFile, "%d %d", &camera.imageWidth, &camera.imageHeight);
	if (1) printf("%d \n", camera.camId);
	if (1) printf("%d x %d \n", camera.imageWidth, camera.imageHeight);
	//Extrinsic Parameters
	for (int k = 0; k < 3; k++) {
		fscanf(calibFile, "%lf %lf %lf", &camera.worldToCameraRotation.coords[0][k], &camera.worldToCameraRotation.coords[1][k], &camera.worldToCameraRotation.coords[2][k]);
		fscanf(calibFile, "%lf", &camera.worldToCameraTranslation[k]);
	}

	//Intrinsic Parameters
	double intrinisc_unassigned[4];
	fscanf(calibFile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &camera.fx, &camera.fxy, &camera.cx, &intrinisc_unassigned[0], &camera.fy, &camera.cy, &intrinisc_unassigned[1], &intrinisc_unassigned[2], &intrinisc_unassigned[3]);

	//Distortion Parameters
	for (int j = 0; j < 5; j++) fscanf(calibFile, "%lf", &camera.distortionParams[j]);


	if (1) printf("World to Camera translation: \n");
	if (1) printf(" %f %f %f \n", camera.worldToCameraTranslation[0], camera.worldToCameraTranslation[1], camera.worldToCameraTranslation[2]);
	if (1) printf("World to Camera rotation: \n");
	if (1) {
		for (int k = 0; k < 3; k++) printf(" %f %f %f \n", camera.worldToCameraRotation.coords[0][k], camera.worldToCameraRotation.coords[1][k], camera.worldToCameraRotation.coords[2][k]);
	}
	fclose(calibFile);
	
	camera.image.read(imageName);

	return 1;
}

int main(int argc, char* argv[])
{
	if (argc != 5){
		printf("%s <inputMesh.ply> <image.png> <cameraFile.txt> <outputMesh.ply>",argv[0]);
		return 0;
	}

	SimpleMesh inputMesh;
	if (!ReadSimpleMesh(inputMesh, argv[1])) {
		printf("Unable to read mesh file %s \n", argv[1]);
	}
	UpdateNormals(inputMesh);
	StageCamera camera;
	ParseCameraFile(argv[3], argv[2], camera);


	camera.Initialize(inputMesh.vertices);
	bool useDiscontinuity = true;
	bool smoothDepth = true;
	InitializeCamera(inputMesh, camera, useDiscontinuity);

	ColoredMesh outputMesh;
	outputMesh.vertices = inputMesh.vertices;
	outputMesh.normals = inputMesh.normals;
	outputMesh.triangles = inputMesh.triangles;
	outputMesh.colors.resize(outputMesh.vertices.size(), Point3D<double>(204, 204, 204));
	TextureSampling(camera, outputMesh);

	WriteColoredMesh(outputMesh, argv[4]);

	return 0;
}