To compile in Linux run the Makefile in the CameraProjection folder. Make sure zlib and png libraries are already installed.

To compile in Windows, unzip 4Windows.zip and move /PNG and /ZLIB into /include. Move /lib into the main directory. Compile from the provided solution (tested in VS2017).

Run the example as :

CameraProjection input.ply cameraImage.png calibration.txt output.ply

