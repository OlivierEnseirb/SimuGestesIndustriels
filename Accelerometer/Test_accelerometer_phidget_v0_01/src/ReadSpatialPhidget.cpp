#include "ReadSpatialPhidget.h"

ReadSpatialPhidget::ReadSpatialPhidget(FILE* _file, unsigned int data_rate, DATA_FORMAT data_format)
{
    setAssocietedFilePointer(_file);
    setDataFormat(data_format);
    initializeCPhidgetSpatialHandle();
    writeHeaderInFile(getAssocietedFilePointer(), getDataFormat());
    OpenConnection(-1, 10000);
	display_properties((CPhidgetHandle)spatial);

	//Set the data rate for the spatial events
	setDataRate(spatial, data_rate);
}

ReadSpatialPhidget::~ReadSpatialPhidget()
{
    printf("Closing...\n");
    CloseConnection();
}

void ReadSpatialPhidget::setDataRate (CPhidgetSpatialHandle phid, int milliseconds)
{
    CPhidgetSpatial_setDataRate(phid, milliseconds);
}

void ReadSpatialPhidget::initializeCPhidgetSpatialHandle()
{
	//create the spatial object
	CPhidgetSpatial_create(&spatial);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, AttachHandler_userptr);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, DetachHandler_userptr);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, ErrorHandler_userptr);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called,
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, this);
}

int CCONV AttachHandler(CPhidgetHandle _spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(_spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);

	return 0;
}

int CCONV DetachHandler(CPhidgetHandle _spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(_spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle _spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

int CCONV SpatialDataHandler(CPhidgetSpatialHandle _spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
   // FILE* file = ((FILE**)userptr)[1];
    ReadSpatialPhidget*rsp = (ReadSpatialPhidget*)userptr;//((ReadSpatialPhidget**)userptr)[0];

    for(size_t i = 0; i < (size_t)count; i++)
    {
        rsp->setNewSample(data, i);
        rsp->accelerationIntegration3Axes(rsp->new_sample, rsp->previous_sample);
        rsp->writeDataInFile(rsp->getAssocietedFilePointer(), rsp->getDataFormat(), rsp->new_sample);
    }

    /*for(size_t i = 0; i < (size_t)count; i++)
    {
		printf("=== Data Set: %d ===\n", i);
		printf("Acceleration> x: %6f  y: %6f  x: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
		printf("Angular Rate> x: %6f  y: %6f  x: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
		printf("Magnetic Field> x: %6f  y: %6f  x: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
		printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
	}

	printf("---------------------------------------------\n");
*/
	return 0;
}

int ReadSpatialPhidget::display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);


	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);

	return 0;
}

bool ReadSpatialPhidget::OpenConnection(int serialNumber, int waitingTime)
{
	int result;
	const char *err;

	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, serialNumber);

	//get the program to wait for a spatial device to be attached
	printf("Waiting for spatial to be attached.... \n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, waitingTime)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return false;
	}
	return true;
}

void ReadSpatialPhidget::CloseConnection()
{
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);
}

void ReadSpatialPhidget::convertCPhidgetSpatialToSample(CPhidgetSpatial_SpatialEventDataHandle* data_in, size_t data_indice, Sample& data_out)
{
    data_out.timestamp = data_in[data_indice]->timestamp.seconds + (data_in[data_indice]->timestamp.microseconds)/1000000.0;
    data_out.acceleration.x = data_in[data_indice]->acceleration[0];
    data_out.acceleration.y = data_in[data_indice]->acceleration[1];
    data_out.acceleration.z = data_in[data_indice]->acceleration[2];

    data_out.angularRate.x = data_in[data_indice]->angularRate[0];
    data_out.angularRate.y = data_in[data_indice]->angularRate[1];
    data_out.angularRate.z = data_in[data_indice]->angularRate[2];

    data_out.magneticField.x = data_in[data_indice]->magneticField[0];
    data_out.magneticField.y = data_in[data_indice]->magneticField[1];
    data_out.magneticField.z = data_in[data_indice]->magneticField[2];
}

void ReadSpatialPhidget::setNewSample(CPhidgetSpatial_SpatialEventDataHandle* data_in, size_t data_indice)
{
    previous_sample = new_sample;
    convertCPhidgetSpatialToSample(data_in, data_indice, new_sample);
    new_sample.num_frame = previous_sample.num_frame + 1;
}

