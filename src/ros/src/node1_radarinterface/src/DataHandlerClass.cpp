#include <DataHandlerClass.h>

unsigned int frame_num_cnt = 0;

DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {
    //DataUARTHandler_pub = nh->advertise<sensor_msgs::PointCloud2>("/node1_radarinterface/radar_scan_pcl", 100);
    radar_scan_pub = nh->advertise<node1_radarinterface::RadarScan>("/node1_radarinterface/radar_scan", 100);
    marker_pub = nh->advertise<visualization_msgs::Marker>("/node1_radarinterface/radar_scan_markers", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified

    // Wait for parameters
    while(!nh->hasParam("/node1_radarinterface/doppler_vel_resolution")){}

    nh->getParam("/node1_radarinterface/numAdcSamples", nr);
    nh->getParam("/node1_radarinterface/numLoops", nd);
    nh->getParam("/node1_radarinterface/num_TX", ntx);
    nh->getParam("/node1_radarinterface/f_s", fs);
    nh->getParam("/node1_radarinterface/f_c", fc);
    nh->getParam("/node1_radarinterface/fc_chirp", fc_chirp);
    nh->getParam("/node1_radarinterface/BW", BW);
    nh->getParam("/node1_radarinterface/PRI", PRI);
    nh->getParam("/node1_radarinterface/t_fr", tfr);
    nh->getParam("/node1_radarinterface/max_range", max_range);
    nh->getParam("/node1_radarinterface/range_resolution", vrange);
    nh->getParam("/node1_radarinterface/max_doppler_vel", max_vel);
    nh->getParam("/node1_radarinterface/doppler_vel_resolution", vvel);

    ROS_INFO("\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nNumber of TXs: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
        nr, nd, ntx, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{
    
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};
    
    /*Open UART Port and error checking*/
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    
    /*Quick magicWord check to synchronize program with data Stream*/
    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
    }
    
    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);
    
    while(ros::ok())
    {
        /*Start reading UART data and writing to buffer while also checking for magicWord*/
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        
        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        
            /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            /*increment countSync*/
            countSync++;
            
            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }
            
            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
            
            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);
            
            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));
              
        }
      
    }
    
    
    mySerialObject.close();
    
    pthread_exit(NULL);
}


int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataUARTHandler::syncedBufferSwap(void)
{
    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}

void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_POINT_CLOUD_2D;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    uint16_t i = 0, j = 0, tlvCount = 0, offset = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;
    uint8_t rxHeader[52] = {'0'};
    uint8_t tempData[9] = {'0'};
    uint16_t num_Points = 0, num_Targets = 0, num_TargetIdx = 0, num_Points_lastframe = 0;
    uint32_t num_Points_inBuf = 0, num_Targets_inBuf = 0, num_TargetIdx_inBuf = 0;
    uint8_t Point_List_Idx = 0;
    float velocityRes = 0;
    float dopplerRes = 0;
    float point_range_xy_plane = 0;
    char info_str[1000] = {0};
    float max_doppler = 0;
    float min_doppler = 0;
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    node1_radarinterface::RadarScan *radarscan_cf = NULL; // current frame
    node1_radarinterface::RadarScan radarscan_pub; // last frame

    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:

            //ROS_INFO("Read Header");
            
            //make sure packet has at least first four fields (16 bytes) before we read them (does not include magicWord since it was already removed)
            if(currentBufp->size() < 16)
            {
                sorterState = SWAP_BUFFERS;
                break;
            }
            
            //get version (4 bytes)
            memcpy( &mmwData.frameHeader.version, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.version));
            currentDatap += ( sizeof(mmwData.frameHeader.version) );

            //get platform (4 bytes)
            memcpy( &mmwData.frameHeader.platform, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.platform));
            currentDatap += ( sizeof(mmwData.frameHeader.platform) );   
            //ROS_WARN("Platform : %x", mmwData.frameHeader.platform);

            //get timestamp (4 bytes)
            memcpy( &mmwData.frameHeader.timestamp, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.timestamp));
            currentDatap += ( sizeof(mmwData.frameHeader.timestamp) );
            
            //get packetLength (4 bytes)
            memcpy( &mmwData.frameHeader.packetLength, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.packetLength));
            currentDatap += ( sizeof(mmwData.frameHeader.packetLength) );
            //ROS_WARN("packetLength : %d", mmwData.frameHeader.packetLength);

            headerSize = 44; // frame header size without magic word
            if(currentBufp->size() < headerSize) {
                sorterState = SWAP_BUFFERS;
                break;
            }
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.frameHeader.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.frameNumber));
            currentDatap += ( sizeof(mmwData.frameHeader.frameNumber) );

            //get subframeNumber (4 bytes)
            memcpy( &mmwData.frameHeader.subframeNumber, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.subframeNumber));
            currentDatap += ( sizeof(mmwData.frameHeader.subframeNumber) );

            //get chirpMargin (4 bytes)
            memcpy( &mmwData.frameHeader.chirpMargin, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.chirpMargin));
            currentDatap += ( sizeof(mmwData.frameHeader.chirpMargin) );

            //get frameMargin (4 bytes)
            memcpy( &mmwData.frameHeader.frameMargin, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.frameMargin));
            currentDatap += ( sizeof(mmwData.frameHeader.frameMargin) );

            //get uartSentTime (4 bytes)
            memcpy( &mmwData.frameHeader.uartSentTime, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.uartSentTime));
            currentDatap += ( sizeof(mmwData.frameHeader.uartSentTime) );

            //get trackProcessTime (4 bytes)
            memcpy( &mmwData.frameHeader.trackProcessTime, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.trackProcessTime));
            currentDatap += ( sizeof(mmwData.frameHeader.trackProcessTime) );
            
            //get numTLVs (2 bytes)
            memcpy( &mmwData.frameHeader.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.numTLVs));
            currentDatap += ( sizeof(mmwData.frameHeader.numTLVs) );

            //get checksum (2 bytes)
            memcpy( &mmwData.frameHeader.checksum, &currentBufp->at(currentDatap), sizeof(mmwData.frameHeader.checksum));
            currentDatap += ( sizeof(mmwData.frameHeader.checksum) );
            
            //checksum
            //memcpy(&rxHeader, magicWord, sizeof(magicWord));
            //memcpy(&(rxHeader+sizeof(magicWord)), &mmwData.frameHeader, sizeof(mmwData.frameHeader));
            //ROS_INFO("frameHeader : %x", mmwData.frameHeader);
            //ROS_INFO("rxHeader : %d", itoa(rxHeader);

            //if packet lengths do not patch, throw it away
            if(mmwData.frameHeader.packetLength == currentBufp->size() )
            {
                // ROS_WARN("Header synced. Num of TLVs: %d", mmwData.frameHeader.numTLVs);
                sorterState = CHECK_TLV_TYPE;
            }
            else{
                sorterState = SWAP_BUFFERS;
            }
            Point_List_Idx = 0;
            break;

        case READ_POINT_CLOUD_2D:
            Point_List_Idx ++;
            if(radarscan_cf != NULL)
            {   
                free(radarscan_cf);
                radarscan_cf = NULL;
            }
            if(radarscan_cf == NULL)
            {
                radarscan_cf = (node1_radarinterface::RadarScan * )malloc(num_Points * sizeof(node1_radarinterface::RadarScan));
            }
            //num_Points = (tlvLen-sizeof(mmwData.tlvHeader))/sizeof(mmwData.PointCloudData);
            for(i=0; i<num_Points; i++)
            {
                // Refer to C:\ti\mmwave_industrial_toolbox_4_0_0\labs\node1_radarinterface\18xx_68xx_node1_radarinterface\gui\getGtrackPtCloud.m
                //get range
                memcpy( &(radarscan_cf[i].range), &currentBufp->at(currentDatap), sizeof(mmwData.PointCloudData.range));
                currentDatap += ( sizeof(mmwData.PointCloudData.range) );
                //get azimuth
                memcpy( &(radarscan_cf[i].angle), &currentBufp->at(currentDatap), sizeof(mmwData.PointCloudData.angle));
                currentDatap += ( sizeof(mmwData.PointCloudData.angle) );
                //get elevation
                memcpy( &(radarscan_cf[i].elev), &currentBufp->at(currentDatap), sizeof(mmwData.PointCloudData.elev));
                currentDatap += ( sizeof(mmwData.PointCloudData.elev) );
                //get doppler
                memcpy( &(radarscan_cf[i].doppler), &currentBufp->at(currentDatap), sizeof(mmwData.PointCloudData.doppler));
                currentDatap += ( sizeof(mmwData.PointCloudData.doppler) );

                // ROS_INFO("SNR = %lf", radarscan_cf[i].intensity);
                // ROS_INFO("Velocity = %lf", radarscan_cf[i].velocity);

                radarscan_cf[i].point_id    = i;
                point_range_xy_plane        = radarscan_cf[i].range * cos(radarscan_cf[i].elev);
                radarscan_cf[i].x           = point_range_xy_plane * sin(radarscan_cf[i].angle);
                radarscan_cf[i].y           = point_range_xy_plane * cos(radarscan_cf[i].angle);
                radarscan_cf[i].z           = radarscan_cf[i].range * sin(radarscan_cf[i].elev);
                
                velocityRes                 = 3.0e8 / (2.0 * nd * 3 * fc_chirp * PRI);
                dopplerRes                  = velocityRes;
                // ROS_ERROR("BW = %lf", BW);
                // ROS_ERROR("nd = %d", nd);
                // ROS_ERROR("fc_chirp = %lf", fc_chirp);
                // ROS_ERROR("PRI = %lf", PRI);
                // ROS_WARN("velocityRes = %lf", velocityRes);
                // ROS_WARN("dopplerRes = %lf", dopplerRes);
                radarscan_cf[i].doppler_bin = round(radarscan_cf[i].doppler / dopplerRes + nd/2);
                //ROS_WARN("Doppler Bin = %d", radarscan_cf[i].doppler_bin);

                min_doppler = min_doppler < radarscan_cf[i].doppler ? min_doppler : radarscan_cf[i].doppler;
                max_doppler = max_doppler > radarscan_cf[i].doppler ? max_doppler : radarscan_cf[i].doppler;
                ROS_WARN("Doppler range is (%f, %f)", min_doppler, max_doppler);
            }

            // //num_Points = (tlvLen-sizeof(mmwData.tlvHeader))/sizeof(mmwData.PointCloudData);
            // for(i=0; i<num_Points; i++)
            // {
            //     //get point cloud
            //     memcpy( tempData, &currentBufp->at(currentDatap), sizeof(mmwData.PointCloudData));
            //     currentDatap += ( sizeof(mmwData.PointCloudData) );
            // }

            sorterState = CHECK_TLV_TYPE;
            break;

        case READ_TARGET_LIST_2D: 
            Point_List_Idx ++;
            num_Targets = tlvLen/sizeof(mmwData.TargetListData);
            if ( TargetListDataSet != NULL ){
                free(TargetListDataSet);
                TargetListDataSet = NULL;
            }            
            TargetListDataSet = (MmwDemo_output_message_TargetList_t *)malloc(num_Targets * sizeof(mmwData.TargetListData));
            for(i=0; i<num_Targets; i++)
            {
                //get target list
                memcpy( &(TargetListDataSet[i]), &currentBufp->at(currentDatap), sizeof(mmwData.TargetListData));
                currentDatap += ( sizeof(mmwData.TargetListData) );
            }

            sorterState = CHECK_TLV_TYPE;
            break;

        case READ_TARGET_INDEX: 
            Point_List_Idx ++;
            num_TargetIdx = tlvLen/sizeof(mmwData.TargetIndexData);
            for(i=0; i<num_TargetIdx; i++)
            {
                //get target index
                memcpy( &(radarscan_cf[i].target_idx), &currentBufp->at(currentDatap), sizeof(mmwData.TargetIndexData));
                currentDatap += ( sizeof(mmwData.TargetIndexData) );
            }
            if(Point_List_Idx == 4){
                // ROS_INFO("publish targets");
                for(i=0; i<num_TargetIdx; i++)
                {
                    radarscan_pub.header.frame_id       = "node1_radarinterface_radarscan";
            	    radarscan_pub.header.stamp          = ros::Time::now();
                    // Original measurement from radar
                    radarscan_pub.range                 = radarscan_cf[i].range;
                    radarscan_pub.angle                 = radarscan_cf[i].angle;
                    radarscan_pub.elev                  = radarscan_cf[i].elev;
                    radarscan_pub.doppler               = radarscan_cf[i].doppler;
                    radarscan_pub.snr                   = radarscan_cf[i].snr;
                    radarscan_pub.noise                 = radarscan_cf[i].noise;
                    // Point in the Cartesian coordinates  
                    radarscan_pub.frame_num             = frame_num_cnt;           
                    radarscan_pub.point_id              = radarscan_cf[i].point_id;
                    radarscan_pub.target_idx            = radarscan_cf[i].target_idx;
                    radarscan_pub.x                     = radarscan_cf[i].x;
                    radarscan_pub.y                     = radarscan_cf[i].y;
                    radarscan_pub.z                     = radarscan_cf[i].z;
                    radarscan_pub.doppler_bin           = radarscan_cf[i].doppler_bin;
                    // Target centriod in the Cartesian coordinates
                    for(j=0; j<num_Targets; j++)
                    {
                        if( radarscan_cf[i].target_idx == TargetListDataSet[j].tid )
                        {
                            radarscan_pub.posX          = TargetListDataSet[j].posX;
                            radarscan_pub.posY          = TargetListDataSet[j].posY;
                            radarscan_pub.posZ          = TargetListDataSet[j].posZ;
                            radarscan_pub.velX          = TargetListDataSet[j].velX;
                            radarscan_pub.velY          = TargetListDataSet[j].velY;
                            radarscan_pub.velZ          = TargetListDataSet[j].velZ;
                        }
                    } 
                    if (radarscan_pub.target_idx < 253){
                        // // Coordiante transformation due to tilting
                        // height          =  1.80 // meters
                        // tilt_angle      = -15.0 // tilt degrees
                        // transfered_x    = radarscan_pub.x
                        // transfered_y    = radarscan_pub.y * cos(tilt_angle) - radarscan_pub.z * sin(tilt_angle)
                        // transfered_z    = radarscan_pub.y * sin(tilt_angle) + radarscan_pub.z * cos(tilt_angle)
                        // transfered_posX = radarscan_pub.posX
                        // transfered_posY = radarscan_pub.posY * cos(tilt_angle) - radarscan_pub.posZ * sin(tilt_angle)
                        // transfered_posZ = radarscan_pub.posy * sin(tilt_angle) + radarscan_pub.posZ * cos(tilt_angle)
                        // radarscan_pub.x     = transfered_x
                        // radarscan_pub.y     = transfered_y
                        // radarscan_pub.z     = transfered_z
                        // radarscan_pub.posX  = transfered_posX
                        // radarscan_pub.posY  = transfered_posY
                        // radarscan_pub.posZ  = transfered_posZ

                        radar_scan_pub.publish(radarscan_pub); 
                    }
                          
                    if (radarscan_pub.target_idx < 253)           
                        visualize(radarscan_pub);
                }
                // Visualize the centroid
                for(j=0; j<num_Targets; j++)
                {
                    radarscan_pub.target_idx    = 360; // Means it's a centroid
                    radarscan_pub.point_id      = num_TargetIdx + j + 1; // Means it's a centroid
                    radarscan_pub.x             = TargetListDataSet[j].posX;
                    radarscan_pub.y             = TargetListDataSet[j].posY;
                    radarscan_pub.z             = TargetListDataSet[j].posZ;
                    visualize(radarscan_pub);
                } 
                frame_num_cnt = frame_num_cnt + 1;
            }
            else
                Point_List_Idx = 0;
            sorterState = CHECK_TLV_TYPE;  
            break;  

        case READ_SIDE_INFO:
            Point_List_Idx++;
            num_Points = tlvLen/sizeof(mmwData.SideInfoData);
            for(i=0; i<num_Points; i++)
            {
                memcpy( &(radarscan_cf[i].snr), &currentBufp->at(currentDatap), sizeof(mmwData.SideInfoData.snr));
                currentDatap += ( sizeof(mmwData.SideInfoData.snr) );
                memcpy( &(radarscan_cf[i].noise), &currentBufp->at(currentDatap), sizeof(mmwData.SideInfoData.noise));
                currentDatap += ( sizeof(mmwData.SideInfoData.noise) );

                // ROS_WARN("Read SideInfoData. SNR: %d, Noise: %d ", radarscan_cf[i].snr, radarscan_cf[i].snr);
            }
            sorterState = CHECK_TLV_TYPE;
            break;
        
        case CHECK_TLV_TYPE:
        
            // ROS_INFO("DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.frameHeader.numTLVs);
        
            if(tlvCount++ >= mmwData.frameHeader.numTLVs)
            {
                // ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            else
            {
                // get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                // get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                // ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                switch(tlvType)
                {                  
                    case MMWDEMO_OUTPUT_MSG_POINT_CLOUD_2D:
                        if( tlvLen % sizeof(mmwData.PointCloudData) == 0 )
                        {
                            num_Points_lastframe = num_Points;
                            num_Points = tlvLen/sizeof(mmwData.PointCloudData);
                            num_Points_inBuf = (currentBufp->size()) / sizeof(mmwData.PointCloudData);
                            if( num_Points_inBuf >= num_Points){
                                // ROS_INFO("Points Num : %d", num_Points);
                                // ROS_INFO("Go to : Point cloud");
                                // ROS_INFO("Buffer size  = %d", currentBufp->size());
                                sorterState = READ_POINT_CLOUD_2D;
                            }
                            else{
                                ROS_ERROR("Point cloud : buffer empty, points number = %d, points number in buffer = %d", num_Points, num_Points_inBuf);
                                sorterState = SWAP_BUFFERS;
                            }
                        }
                        else{
                            ROS_ERROR("Number of point cloud is not an integer");
                            sorterState = SWAP_BUFFERS;
                        }
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_TARGET_LIST_2D:
                        if( tlvLen % sizeof(mmwData.TargetListData) == 0 )
                        {
                            num_Targets = tlvLen/sizeof(mmwData.TargetListData);
                            num_Targets_inBuf = (currentBufp->size()) / sizeof(mmwData.TargetListData);
                            if( num_Targets_inBuf >= num_Targets){
                                // ROS_INFO("Target Num : %d", num_Targets);
                                // ROS_INFO("Go to : Target list");
                                // ROS_INFO("Buffer size = %d", currentBufp->size());
                                sorterState = READ_TARGET_LIST_2D;
                            }
                            else{
                                ROS_ERROR("Targte list : buffer empty, targets number = %d, targets number in buffer = %d", num_Targets, num_Targets_inBuf);
                                sorterState = SWAP_BUFFERS;
                            }
                        }
                        else{
                            ROS_ERROR("Number of targte list is not an integer");
                            sorterState = SWAP_BUFFERS;
                        }
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_TARGET_INDEX:
                        num_TargetIdx = tlvLen/sizeof(mmwData.TargetIndexData);
                        num_TargetIdx_inBuf = (currentBufp->size()) / sizeof(mmwData.TargetIndexData);
                        if (num_TargetIdx_inBuf >= num_TargetIdx){
                            // ROS_INFO("Target Idx : %d", num_TargetIdx);
                            // ROS_INFO("Go to : Target index");
                            sorterState = READ_TARGET_INDEX;
                        }
                        else{
                            ROS_ERROR("Target index : buffer empty, index number = %d, index number in buffer = %d", num_TargetIdx, num_TargetIdx_inBuf);
                            sorterState = SWAP_BUFFERS;
                        }
                        break;

                    case UART_MSG_MMW_SIDE_INFO:
                        num_Points = tlvLen/sizeof(mmwData.SideInfoData);
                        num_Points_inBuf = (currentBufp->size()) / sizeof(mmwData.SideInfoData);
                        if( num_Points_inBuf >= num_Points){
                            sorterState = READ_SIDE_INFO;
                        }
                        else{
                            ROS_ERROR("Number of point cloud is not an integer");
                            sorterState = SWAP_BUFFERS;
                        }
                        break;
                    
                    default: 
                        ROS_ERROR("TLV header error.");
                        sorterState = SWAP_BUFFERS;
                        break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:

            // ROS_INFO("Swap Data");
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;

            // ROS_INFO("Swap Data DONE");
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;
    
    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}

void DataUARTHandler::visualize(const node1_radarinterface::RadarScan &msg){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "node1_radarinterface_markers";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = msg.point_id;
    marker.lifetime = ros::Duration(tfr);
    marker.action = marker.ADD;

    marker.pose.position.x = msg.x;
    marker.pose.position.y = msg.y;
    marker.pose.position.z = msg.z;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;
    
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;
    // ROS_INFO("Target Idx = %d", msg.target_idx);

    if(msg.target_idx < 253){
        marker.color.a = 1;
        // marker.color.r = color_mat[msg.target_idx][0];
        // marker.color.g = color_mat[msg.target_idx][1];
        // marker.color.b = color_mat[msg.target_idx][2];
        marker.color.r = 255;
        marker.color.g = 0;
        marker.color.b = 0;
    }
    // Centroid
    else if(msg.target_idx == 360){
        marker.color.a = 1;
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 0;
    }
    // Clutter
    else{
        marker.color.a = 1;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
    }

    marker_pub.publish(marker);
}
