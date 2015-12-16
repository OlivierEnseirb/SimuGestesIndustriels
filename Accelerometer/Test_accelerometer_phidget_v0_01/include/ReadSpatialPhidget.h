// code got from Phidgets Inc. But improved to a C++ by Olivier Hartmann
// Copyright 2010 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License.
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#ifndef READSPATIALPHIDGET_H
#define READSPATIALPHIDGET_H

//#include <stdio.h>
//#include <phidget21.h>
#include "IMU_maths.h"

//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);

/** callback that will run at datarate
* @param data - array of spatial event data structures that holds the spatial data packets that were sent in this event
* @param count - the number of spatial data event packets included in this event
*/
int CCONV SpatialDataHandler(CPhidgetSpatialHandle _spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);



class ReadSpatialPhidget : public IMU_maths
{
    public:
        ReadSpatialPhidget(){}
        ReadSpatialPhidget(FILE* _file, unsigned int data_rate, DATA_FORMAT data_format);
        ~ReadSpatialPhidget();

        /**
         * Sets the data rate. Note that data at rates faster then 8ms will be delivered to events as an array of data.
         * @param phid An attached phidget spatial handle.
         * @param milliseconds The data rate in milliseconds.
         */
        void setDataRate (CPhidgetSpatialHandle phid, int milliseconds);

       /** create spatial object
        * set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
        * set handlers to be run when receiving data.
        */
        void initializeCPhidgetSpatialHandle();

       /** Display the properties of the attached phidget to the screen.
        * We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
        * of the attached Spatial.
        * @param phid - A phidget handle.
        */
        int display_properties(CPhidgetHandle phid);

        /** Opens a Phidget.
         * @param phid - A phidget handle.
         * @param serialNumber - Serial number. Specify -1 to open any.
         * @param waitingTime - a time in millisencond to wait for a a spatial device to be attached
         */
        bool OpenConnection(int serialNumber, int waitingTime);

        /** Close a phidget.
         * @param _spatial - A phidget handle.
         */
        void CloseConnection();

        void convertCPhidgetSpatialToSample(CPhidgetSpatial_SpatialEventDataHandle* data_in, size_t data_indice, Sample& data_out);

        void setNewSample(CPhidgetSpatial_SpatialEventDataHandle* data_in, size_t data_indice);

        Sample new_sample = defaultSample;
        Sample previous_sample = defaultSample;

        void setAssocietedFilePointer(FILE * _file){associatedFile = _file;}
        FILE* getAssocietedFilePointer(){return associatedFile;}
        void setDataFormat(DATA_FORMAT _data_format){data_format = _data_format;}
        DATA_FORMAT getDataFormat(){return data_format;}
    protected:
    private:
      //  void * SpatialDataHandler_userptr = NULL;
        void* AttachHandler_userptr = NULL;
        void* DetachHandler_userptr = NULL;
        void* ErrorHandler_userptr = NULL;

        FILE* associatedFile = NULL;

        DATA_FORMAT data_format = CSV;


        CPhidgetSpatialHandle spatial = 0; //a spatial handle

};

#endif // READSPATIALPHIDGET_H
