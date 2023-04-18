/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Mocap.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <unistd.h>
#include <iomanip>

Mocap::Mocap(std::vector<int> &id) :g_pClient(NULL), g_analogSamplesPerMocapFrame(0), g_discoveredMulticastGroupAddr(NATNET_DEFAULT_MULTICAST_ADDRESS), numBody(id.size())
{
    for(int i=0; i<numBody; i++)
    {
        pose[id[i]] = Sophus::SE3f();
        valid[id[i]] = false;
    }

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( DataHandler, this );	// this function will receive data from the server

    NatNetDiscoveryHandle discovery;
    NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback, this );

    while(g_discoveredServers.empty()) usleep(1e6);
    const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[0];

    // Build the connection parameters.
    snprintf(
        g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
        "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
        discoveredServer.serverDescription.ConnectionMulticastAddress[0],
        discoveredServer.serverDescription.ConnectionMulticastAddress[1],
        discoveredServer.serverDescription.ConnectionMulticastAddress[2],
        discoveredServer.serverDescription.ConnectionMulticastAddress[3]
    );

    g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
    g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
    g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
    g_connectParams.serverAddress = discoveredServer.serverAddress;
    g_connectParams.localAddress = discoveredServer.localAddress;
    g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;

    NatNet_FreeAsyncServerDiscovery( discovery );

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. See log for details. Exiting.\n");
        return;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }

}

// Establish a NatNet Client connection
int Mocap::ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    Mocap* pMocap = (Mocap*) pUserData;
    NatNetClient* pClient = pMocap->GetNatNetClient();

    std::map<int, bool>& valid = pMocap->GetVaild();
    std::map<int, Sophus::SE3f>& pose = pMocap->GetPose();

    pMocap->mMutexPose.lock();
    pMocap->SetFrameTime(data->fTimestamp);
    pMocap->SetFrameId(data->iFrame);
    for(int i=0; i < data->nRigidBodies; i++)
	{
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        if(valid.count(data->RigidBodies[i].ID)){
            Eigen::Quaternionf q(data->RigidBodies[i].qw, data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz);
            Eigen::Vector3f t(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z);
            valid[data->RigidBodies[i].ID] = bTrackingValid;
            pose[data->RigidBodies[i].ID] = Sophus::SE3f(q,t);
        }
	}
    pMocap->mMutexPose.unlock();
}

void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext )
{
    Mocap* pMocap = (Mocap*) pUserContext;
    std::vector< sNatNetDiscoveredServer >& g_discoveredServers = pMocap->GetDiscoveredServers();
    
    char serverHotkey = '.';
    if ( g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    printf( "[%c] %s %d.%d at %s ",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress );

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid )
    {
        printf( "(%s)\n", pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast" );
    }
    else
    {
        printf( "(WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)\n" );
    }

    g_discoveredServers.push_back( *pDiscoveredServer );
}



